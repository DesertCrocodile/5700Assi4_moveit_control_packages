import tkinter as tk
from tkinter import messagebox

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class TargetPoseGUI(Node):
    def __init__(self):
        super().__init__("target_pose_gui")

        self.pub = self.create_publisher(PoseStamped, "/target_pose", 10)

        self.done_sub = self.create_subscription(
            Bool,
            "/move_done",
            self.done_callback,
            10
        )

        self.busy_sub = self.create_subscription(
            Bool,
            "/move_busy",
            self.busy_callback,
            10
        )

        self.busy = False
        self.last_done = True

        self.root = tk.Tk()
        self.root.title("MoveIt Target Pose Sender")

        self.entries = {}

        fields = [
            ("x", "0.36725"),
            ("y", "-0.275"),
            ("z", "0.25"),
            ("qx", "1.0"),
            ("qy", "0.0"),
            ("qz", "0.0"),
            ("qw", "0.0"),
        ]

        for i, (name, default) in enumerate(fields):
            tk.Label(self.root, text=name).grid(row=i, column=0, padx=8, pady=4)

            entry = tk.Entry(self.root, width=16)
            entry.insert(0, default)
            entry.grid(row=i, column=1, padx=8, pady=4)

            self.entries[name] = entry

        self.status_label = tk.Label(
            self.root,
            text="Status: Ready",
            fg="green"
        )
        self.status_label.grid(row=len(fields), column=0, columnspan=2, pady=8)

        self.send_button = tk.Button(
            self.root,
            text="Send Target Pose",
            command=self.send_target
        )
        self.send_button.grid(row=len(fields) + 1, column=0, columnspan=2, pady=8)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.timer = self.create_timer(0.05, self.gui_update)

    def gui_update(self):
        self.root.update()

    def busy_callback(self, msg: Bool):
        self.busy = msg.data

        if self.busy:
            self.status_label.config(text="Status: Moving...", fg="orange")
            self.send_button.config(state=tk.DISABLED)
        else:
            if self.last_done:
                self.status_label.config(text="Status: Ready", fg="green")
            self.send_button.config(state=tk.NORMAL)

    def done_callback(self, msg: Bool):
        self.last_done = msg.data

        if msg.data:
            self.status_label.config(text="Status: Move Done", fg="green")
            self.send_button.config(state=tk.NORMAL)
        else:
            self.status_label.config(text="Status: Move Failed / Running", fg="red")

    def send_target(self):
        if self.busy:
            messagebox.showwarning("Busy", "Robot is still moving.")
            return

        try:
            x = float(self.entries["x"].get())
            y = float(self.entries["y"].get())
            z = float(self.entries["z"].get())
            qx = float(self.entries["qx"].get())
            qy = float(self.entries["qy"].get())
            qz = float(self.entries["qz"].get())
            qw = float(self.entries["qw"].get())
        except ValueError:
            messagebox.showerror("Input Error", "All values must be numbers.")
            return

        norm = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
        if norm < 1e-6:
            messagebox.showerror("Input Error", "Quaternion norm cannot be zero.")
            return

        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.pub.publish(msg)

        self.status_label.config(text="Status: Target Sent", fg="blue")
        self.send_button.config(state=tk.DISABLED)

        self.get_logger().info(
            f"Published target: "
            f"pos=({x:.3f}, {y:.3f}, {z:.3f}), "
            f"quat=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})"
        )

    def on_close(self):
        self.root.destroy()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = TargetPoseGUI()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()


if __name__ == "__main__":
    main()