"""Small Tk GUI for sending six-axis joint targets to the robot driver."""

from __future__ import annotations

import math
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


POSITION_TOPIC = "forward_position_controller/commands"

# Limits from ros2_6dof_robot_description/urdf/6dof-robot.urdf, in degrees.
JOINT_LIMITS_DEG = (
    ("J1", -95.0, 95.0),
    ("J2", -75.0, 70.0),
    ("J3", -50.0, 54.0),
    ("J4", -160.0, 160.0),
    ("J5", -100.0, 100.0),
    ("J6", -120.0, 120.0),
)


class JointGoalGui(Node):
    """ROS publisher with a Tk front end."""

    def __init__(self) -> None:
        super().__init__("pico_6dof_joint_goal_gui")
        self._publisher = self.create_publisher(Float64MultiArray, POSITION_TOPIC, 10)
        self._root = tk.Tk()
        self._root.title("Pico 6-DOF Joint Goal")
        self._root.resizable(False, False)
        self._root.protocol("WM_DELETE_WINDOW", self.close)
        self._scales: list[tk.Scale] = []
        self._status = tk.StringVar(value="Choose a target and press Send target")
        self._build_widgets()

    def _build_widgets(self) -> None:
        frame = ttk.Frame(self._root, padding=12)
        frame.grid(sticky="nsew")

        for row, (name, lower, upper) in enumerate(JOINT_LIMITS_DEG):
            ttk.Label(frame, text=name, width=3).grid(row=row, column=0, padx=(0, 8))
            scale = tk.Scale(
                frame,
                from_=lower,
                to=upper,
                resolution=0.5,
                orient=tk.HORIZONTAL,
                length=360,
                showvalue=True,
                label=f"{lower:g}° … {upper:g}°",
            )
            scale.set(0.0)
            scale.grid(row=row, column=1, sticky="ew")
            self._scales.append(scale)

        ttk.Button(frame, text="Send target", command=self.send_target).grid(
            row=len(JOINT_LIMITS_DEG), column=0, columnspan=2, pady=(12, 6), sticky="ew"
        )
        ttk.Label(frame, textvariable=self._status).grid(
            row=len(JOINT_LIMITS_DEG) + 1, column=0, columnspan=2
        )

    def send_target(self) -> None:
        """Publish the selected joint configuration in ROS-standard radians."""
        degrees = [float(scale.get()) for scale in self._scales]
        message = Float64MultiArray()
        message.data = [math.radians(value) for value in degrees]
        self._publisher.publish(message)
        formatted = ", ".join(f"{value:.1f}°" for value in degrees)
        self._status.set(f"Sent: {formatted}")
        self.get_logger().info(f"Sent joint target: {formatted}")

    def close(self) -> None:
        """Close the window and end its event loop."""
        self._root.quit()
        self._root.destroy()

    def run(self) -> None:
        """Run Tk while periodically servicing ROS events."""
        def spin_ros() -> None:
            if rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                self._root.after(20, spin_ros)

        self._root.after(20, spin_ros)
        self._root.mainloop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointGoalGui()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
