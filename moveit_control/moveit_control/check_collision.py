import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
from rclpy.qos import qos_profile_sensor_data

class CollisionChecker(Node):
    def __init__(self):
        super().__init__("check_collision")

        self.joint_state = None

        self.sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            qos_profile_sensor_data
        )

        self.client = self.create_client(
            GetStateValidity,
            "/check_state_validity"
        )

    def joint_state_callback(self, msg):
        self.joint_state = msg

    def check(self):
        while self.joint_state is None:
            self.get_logger().info("Waiting for /joint_states...")
            rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info("Waiting for /check_state_validity service...")
        self.client.wait_for_service()

        req = GetStateValidity.Request()

        state = RobotState()
        state.joint_state = self.joint_state
        state.is_diff = False

        req.robot_state = state
        req.group_name = "ur_manipulator"

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        res = future.result()

        print("\n========== Collision Check Result ==========")
        print("Valid:", res.valid)

        if hasattr(res, "contacts") and len(res.contacts) > 0:
            print("\nColliding links:")
            for c in res.contacts:
                print(f"- {c.contact_body_1}  <-->  {c.contact_body_2}")
        else:
            print("No contact details returned by this service version.")
            print("But if Valid is False, MoveIt still thinks the current state is invalid/colliding.")

        print("===========================================\n")


def main(args=None):
    rclpy.init(args=args)

    node = CollisionChecker()
    node.check()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()