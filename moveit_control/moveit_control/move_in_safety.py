import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

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


class MoveInSafety(Node):
    def __init__(self):
        super().__init__("move_in_safety")

        self.move_client = ActionClient(self, MoveGroup, "/move_action")

        self.scene_pub = self.create_publisher(
            PlanningScene,
            "/planning_scene",
            10
        )

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

        # 圆柱中心位置：
        # 实际圆柱底部在 z=0，高度 2m，所以中心 z=1.0
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

        # 多发几次，确保 MoveIt 收到
        for _ in range(5):
            self.scene_pub.publish(scene)
            time.sleep(0.2)

        self.get_logger().info("Added safety cylinder obstacle.")

    def send_goal(self):
        self.add_cylinder_obstacle()

        goal = MoveGroup.Goal()

        goal.request.group_name = "ur_manipulator"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2
        goal.request.planner_id = "RRTConnectkConfigDefault"

        # ========= Target Pose =========
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"

        # 目标点，单位 m
        target_pose.pose.position.x = 0.36725
        target_pose.pose.position.y = -0.2750
        target_pose.pose.position.z = 0.2500

        # 末端姿态：先用较常见的竖直朝下测试
        target_pose.pose.orientation.x = 1.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 0.0

        # ========= Position Constraint =========
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link"
        pos_constraint.link_name = "tool0"
        pos_constraint.weight = 1.0

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.02, 0.02, 0.02]

        pos_constraint.constraint_region.primitives.append(box)
        pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)

        # ========= Orientation Constraint =========
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = "base_link"
        ori_constraint.link_name = "tool0"
        ori_constraint.orientation = target_pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.3
        ori_constraint.absolute_y_axis_tolerance = 0.3
        ori_constraint.absolute_z_axis_tolerance = 0.3
        ori_constraint.weight = 1.0

        constraints = Constraints()
        constraints.name = "safe_target_pose"
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

        self.get_logger().info("Sending safe MoveIt goal...")
        future = self.move_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by MoveIt.")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted by MoveIt.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        code = result.error_code.val

        self.get_logger().info(f"MoveIt result code: {code}")

        if code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("Safe motion executed successfully.")
        else:
            self.get_logger().error(f"MoveIt failed with error code: {code}")

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = MoveInSafety()
    node.send_goal()

    rclpy.spin(node)


if __name__ == "__main__":
    main()