import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive


class MoveToPose(Node):
    def __init__(self):
        super().__init__("move_to_pose")
        self.client = ActionClient(self, MoveGroup, "/move_action")

    def send_goal(self):
        goal = MoveGroup.Goal()

        # =============================
        # MoveIt planning request
        # =============================
        goal.request.group_name = "ur_manipulator"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2
        goal.request.start_state.is_diff = True

        # 如果这个 planner 报错，可以注释掉这一行
        goal.request.planner_id = "RRTConnectkConfigDefault"

        # =============================
        # Target pose: 单位是 m
        # frame_id 是机器人 base_link
        # =============================
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"

        target_pose.pose.position.x = 0.36725
        target_pose.pose.position.y = -0.2750
        target_pose.pose.position.z = 0.2500

        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 1.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 0.0

        # =============================
        # Position constraint
        # =============================
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link"
        pos_constraint.link_name = "tool0"
        pos_constraint.weight = 1.0

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX

        # 容差区域，越大越容易规划成功
        box.dimensions = [0.02, 0.02, 0.02]

        pos_constraint.constraint_region.primitives.append(box)
        pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)

        # =============================
        # Orientation constraint
        # =============================
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = "base_link"
        ori_constraint.link_name = "tool0"
        ori_constraint.orientation = target_pose.pose.orientation

        # 姿态容差，越大越容易成功
        ori_constraint.absolute_x_axis_tolerance = 0.2
        ori_constraint.absolute_y_axis_tolerance = 0.2
        ori_constraint.absolute_z_axis_tolerance = 0.2
        ori_constraint.weight = 1.0

        constraints = Constraints()
        constraints.name = "target_pose_constraints"
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)

        goal.request.goal_constraints.append(constraints)

        # =============================
        # Planning options
        # =============================
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        goal.planning_options.replan_delay = 0.5

        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self.get_logger().info("Waiting for /move_action server...")
        self.client.wait_for_server()

        self.get_logger().info("Sending MoveIt goal...")
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by MoveIt")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted by MoveIt")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        code = result.error_code.val

        self.get_logger().info(f"MoveIt result code: {code}")

        if code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("Motion executed successfully")
        elif code == MoveItErrorCodes.PLANNING_FAILED:
            self.get_logger().error("Planning failed")
        elif code == MoveItErrorCodes.INVALID_MOTION_PLAN:
            self.get_logger().error("Invalid motion plan")
        elif code == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
            self.get_logger().error("Motion plan invalidated by environment change")
        elif code == MoveItErrorCodes.CONTROL_FAILED:
            self.get_logger().error("Controller execution failed")
        elif code == MoveItErrorCodes.START_STATE_IN_COLLISION:
            self.get_logger().error("Start state is in collision")
        elif code == MoveItErrorCodes.GOAL_IN_COLLISION:
            self.get_logger().error("Goal state is in collision")
        elif code == MoveItErrorCodes.NO_IK_SOLUTION:
            self.get_logger().error("No IK solution")
        else:
            self.get_logger().error(f"MoveIt failed with error code: {code}")

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = MoveToPose()
    node.send_goal()

    rclpy.spin(node)


if __name__ == "__main__":
    main()