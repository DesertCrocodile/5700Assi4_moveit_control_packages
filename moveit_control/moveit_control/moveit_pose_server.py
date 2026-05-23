import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    PlanningScene,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    MoveItErrorCodes,
)


class MoveItPoseServer(Node):
    def __init__(self):
        super().__init__("moveit_pose_server")

        self.move_client = ActionClient(self, MoveGroup, "/move_action")

        self.target_sub = self.create_subscription(
            PoseStamped,
            "/target_pose",
            self.target_callback,
            10
        )

        self.done_pub = self.create_publisher(Bool, "/move_done", 10)
        self.busy_pub = self.create_publisher(Bool, "/move_busy", 10)

        self.scene_pub = self.create_publisher(
            PlanningScene,
            "/planning_scene",
            10
        )

        self.busy = False
        self.queue = deque()

        self.add_cylinder_obstacle()

        self.status_timer = self.create_timer(0.5, self.publish_status)

        self.get_logger().info("MoveIt pose server is ready.")
        self.get_logger().info("Subscribe target: /target_pose")
        self.get_logger().info("Publish done: /move_done")
        self.get_logger().info("Publish busy: /move_busy")

    def publish_status(self):
        busy_msg = Bool()
        busy_msg.data = self.busy
        self.busy_pub.publish(busy_msg)

    def publish_done(self, value: bool):
        msg = Bool()
        msg.data = value
        self.done_pub.publish(msg)

    def add_cylinder_obstacle(self):
        cylinder = CollisionObject()
        cylinder.header.frame_id = "base_link"
        cylinder.id = "safety_cylinder"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.CYLINDER

        # dimensions = [height, radius]
        primitive.dimensions = [2.0, 0.20]

        pose = PoseStamped()
        pose.header.frame_id = "base_link"

        # 圆柱底部在 z=0，高 2m，所以中心 z=1.0
        pose.pose.position.x = -0.30
        pose.pose.position.y = 0.0
        pose.pose.position.z = 1.0
        pose.pose.orientation.w = 1.0

        cylinder.primitives.append(primitive)
        cylinder.primitive_poses.append(pose.pose)
        cylinder.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(cylinder)

        for _ in range(5):
            self.scene_pub.publish(scene)
            time.sleep(0.1)

        self.get_logger().info("Added safety cylinder obstacle.")

    def target_callback(self, msg: PoseStamped):
        if msg.header.frame_id == "":
            msg.header.frame_id = "base_link"

        self.get_logger().info(
            f"Received target: frame={msg.header.frame_id}, "
            f"pos=({msg.pose.position.x:.3f}, "
            f"{msg.pose.position.y:.3f}, "
            f"{msg.pose.position.z:.3f}), "
            f"quat=({msg.pose.orientation.x:.3f}, "
            f"{msg.pose.orientation.y:.3f}, "
            f"{msg.pose.orientation.z:.3f}, "
            f"{msg.pose.orientation.w:.3f})"
        )

        if self.busy:
            self.queue.append(msg)
            self.get_logger().warn("Robot is busy. Target added to queue.")
            return

        self.send_moveit_goal(msg)

    def send_moveit_goal(self, target_pose: PoseStamped):
        self.busy = True
        self.publish_done(False)
        self.publish_status()

        goal = MoveGroup.Goal()

        goal.request.group_name = "ur_manipulator"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2
        goal.request.planner_id = "RRTConnectkConfigDefault"

        # ========= Position Constraint =========
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = target_pose.header.frame_id
        pos_constraint.link_name = "tool0"
        pos_constraint.weight = 1.0

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX

        # 目标位置容差，越小越精确，越大越容易规划成功
        box.dimensions = [0.02, 0.02, 0.02]

        pos_constraint.constraint_region.primitives.append(box)
        pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)

        # ========= Orientation Constraint =========
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = target_pose.header.frame_id
        ori_constraint.link_name = "tool0"
        ori_constraint.orientation = target_pose.pose.orientation

        # 姿态容差，单位 rad
        ori_constraint.absolute_x_axis_tolerance = 0.3
        ori_constraint.absolute_y_axis_tolerance = 0.3
        ori_constraint.absolute_z_axis_tolerance = 0.3
        ori_constraint.weight = 1.0

        constraints = Constraints()
        constraints.name = "target_pose"
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)

        goal.request.goal_constraints.append(constraints)

        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        goal.planning_options.replan_delay = 0.5
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self.get_logger().info("Waiting for /move_action server...")
        self.move_client.wait_for_server()

        self.get_logger().info("Sending MoveIt goal...")
        future = self.move_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by MoveIt.")
            self.finish_motion(False)
            return

        self.get_logger().info("Goal accepted by MoveIt.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        code = result.error_code.val

        self.get_logger().info(f"MoveIt result code: {code}")

        success = code == MoveItErrorCodes.SUCCESS

        if success:
            self.get_logger().info("Motion finished successfully.")
        else:
            self.get_logger().error(f"Motion failed with error code: {code}")

        self.finish_motion(success)

    def finish_motion(self, success: bool):
        self.busy = False
        self.publish_done(success)
        self.publish_status()

        if len(self.queue) > 0:
            next_target = self.queue.popleft()
            self.get_logger().info("Executing next queued target.")
            self.send_moveit_goal(next_target)


def main(args=None):
    rclpy.init(args=args)

    node = MoveItPoseServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()