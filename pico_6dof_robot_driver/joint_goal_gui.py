"""Tkinter ROS 2 GUI for robot control and recorded joint sequences."""

from __future__ import annotations

import json
import math
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from .joint_sequence import (
    SequenceStep,
    interpolate_sequence,
    load_sequence,
    save_sequence,
)


POSITION_TOPIC = "forward_position_controller/commands"
TRAJECTORY_ACTION = (
    "scaled_joint_trajectory_controller/follow_joint_trajectory"
)
JOINT_NAMES = tuple(f"joint_{index}" for index in range(1, 7))

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
    """ROS publishers, subscribers, and service clients with a Tk front end."""

    def __init__(self) -> None:
        super().__init__("pico_6dof_joint_goal_gui")
        self._position_publisher = self.create_publisher(
            Float64MultiArray, POSITION_TOPIC, 10
        )
        self._gripper_publisher = self.create_publisher(
            Float64, "gripper_position_cmd", 10
        )
        self._trajectory_client = ActionClient(
            self, FollowJointTrajectory, TRAJECTORY_ACTION
        )
        self.create_subscription(
            JointState, "joint_states", self._on_joint_state, 10
        )
        self.create_subscription(
            String, "control_state", self._on_control_state, 10
        )
        self.create_subscription(
            Float64, "gripper_position", self._on_gripper_state, 10
        )
        self._service_clients = {
            name: self.create_client(Trigger, name)
            for name in (
                "connect",
                "disconnect",
                "stop",
                "home_j1",
                "save_zeros",
                "load_zeros",
            )
        }

        self._measured_positions_deg: tuple[float, ...] | None = None
        self._gripper_state: float | None = None
        self._steps: list[SequenceStep] = []
        self._playback_steps: tuple[SequenceStep, ...] = ()
        self._playback_timers: list[str] = []
        self._trajectory_goal_handle = None
        self._playback_generation = 0
        self._sequence_path: Path | None = None

        self._root = tk.Tk()
        self._root.title("Pico 6-DOF ROS 2 Control")
        self._root.minsize(1040, 650)
        self._root.protocol("WM_DELETE_WINDOW", self.close)
        self._status = tk.StringVar(
            value="Waiting for ROS 2 driver state and joint telemetry"
        )
        self._connection_status = tk.StringVar(value="Driver: waiting")
        self._sequence_status = tk.StringVar(value="No recorded steps")
        self._hold_seconds = tk.StringVar(value="2.0")
        self._trajectory_speed = tk.StringVar(value="10.0")
        self._joint_values = [
            tk.DoubleVar(value=0.0) for _ in JOINT_LIMITS_DEG
        ]
        self._measured_labels = [
            tk.StringVar(value="--.-\N{DEGREE SIGN}") for _ in JOINT_LIMITS_DEG
        ]
        self._gripper_value = tk.DoubleVar(value=0.0)
        self._gripper_state_label = tk.StringVar(value="State: --")
        self._build_widgets()

    def _build_widgets(self) -> None:
        root_frame = ttk.Frame(self._root, padding=10)
        root_frame.grid(row=0, column=0, sticky="nsew")
        self._root.rowconfigure(0, weight=1)
        self._root.columnconfigure(0, weight=1)
        root_frame.rowconfigure(1, weight=1)
        root_frame.columnconfigure(0, weight=1)
        root_frame.columnconfigure(1, weight=2)

        status_frame = ttk.Frame(root_frame)
        status_frame.grid(
            row=0,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=(0, 8),
        )
        status_frame.columnconfigure(1, weight=1)
        ttk.Label(status_frame, textvariable=self._connection_status).grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(status_frame, textvariable=self._status, anchor="e").grid(
            row=0, column=1, sticky="ew", padx=(20, 0)
        )

        controls = ttk.LabelFrame(root_frame, text="Robot control", padding=10)
        controls.grid(row=1, column=0, sticky="nsew", padx=(0, 6))
        controls.columnconfigure(1, weight=1)

        ttk.Label(
            controls, text="Target", font=("TkDefaultFont", 9, "bold")
        ).grid(row=0, column=1)
        ttk.Label(
            controls, text="Measured", font=("TkDefaultFont", 9, "bold")
        ).grid(row=0, column=2, padx=(8, 0))
        for row, (name, lower, upper) in enumerate(JOINT_LIMITS_DEG, start=1):
            ttk.Label(controls, text=name, width=3).grid(
                row=row, column=0, sticky="w"
            )
            scale = tk.Scale(
                controls,
                variable=self._joint_values[row - 1],
                from_=lower,
                to=upper,
                resolution=0.5,
                orient=tk.HORIZONTAL,
                length=315,
                showvalue=True,
            )
            scale.grid(row=row, column=1, sticky="ew")
            ttk.Label(
                controls,
                textvariable=self._measured_labels[row - 1],
                width=9,
                anchor="e",
            ).grid(row=row, column=2, padx=(8, 0))

        joint_button_row = len(JOINT_LIMITS_DEG) + 1
        ttk.Button(
            controls, text="Send target", command=self.send_target
        ).grid(
            row=joint_button_row,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=(8, 3),
        )
        ttk.Button(
            controls,
            text="Measured \N{RIGHTWARDS ARROW} sliders",
            command=self.use_measured,
        ).grid(
            row=joint_button_row,
            column=2,
            sticky="ew",
            padx=(8, 0),
            pady=(8, 3),
        )

        gripper = ttk.LabelFrame(
            controls, text="Gripper (0 open, 1 closed)", padding=6
        )
        gripper.grid(
            row=joint_button_row + 1,
            column=0,
            columnspan=3,
            sticky="ew",
            pady=(8, 0),
        )
        gripper.columnconfigure(0, weight=1)
        tk.Scale(
            gripper,
            variable=self._gripper_value,
            from_=0.0,
            to=1.0,
            resolution=0.05,
            orient=tk.HORIZONTAL,
            showvalue=True,
        ).grid(row=0, column=0, sticky="ew")
        ttk.Button(
            gripper, text="Set gripper", command=self.send_gripper
        ).grid(row=0, column=1, padx=(6, 0))
        ttk.Label(
            gripper, textvariable=self._gripper_state_label, width=12
        ).grid(row=0, column=2, padx=(6, 0))

        services = ttk.LabelFrame(controls, text="Driver", padding=6)
        services.grid(
            row=joint_button_row + 2,
            column=0,
            columnspan=3,
            sticky="ew",
            pady=(8, 0),
        )
        service_buttons = (
            ("Connect", "connect"),
            ("Disconnect", "disconnect"),
            ("STOP / hold", "stop"),
            ("Home J1", "home_j1"),
            ("Save zeros", "save_zeros"),
            ("Load zeros", "load_zeros"),
        )
        for index, (label, service) in enumerate(service_buttons):
            command = (
                self.stop_playback
                if service == "stop"
                else lambda name=service: self._driver_service(name)
            )
            ttk.Button(
                services,
                text=label,
                command=command,
            ).grid(
                row=index // 3,
                column=index % 3,
                padx=2,
                pady=2,
                sticky="ew",
            )
            services.columnconfigure(index % 3, weight=1)

        sequence = ttk.LabelFrame(
            root_frame, text="Recorded joint sequence", padding=10
        )
        sequence.grid(row=1, column=1, sticky="nsew", padx=(6, 0))
        sequence.rowconfigure(0, weight=1)
        sequence.columnconfigure(0, weight=1)

        columns = (
            "step",
            "j1",
            "j2",
            "j3",
            "j4",
            "j5",
            "j6",
            "gripper",
            "hold",
        )
        self._tree = ttk.Treeview(
            sequence, columns=columns, show="headings", height=16
        )
        headings = (
            "#",
            "J1",
            "J2",
            "J3",
            "J4",
            "J5",
            "J6",
            "Grip",
            "Hold (s)",
        )
        widths = (38, 55, 55, 55, 55, 55, 55, 55, 72)
        for column, heading, width in zip(columns, headings, widths):
            self._tree.heading(column, text=heading)
            self._tree.column(
                column,
                width=width,
                minwidth=width,
                anchor="center",
                stretch=True,
            )
        scrollbar = ttk.Scrollbar(
            sequence, orient="vertical", command=self._tree.yview
        )
        self._tree.configure(yscrollcommand=scrollbar.set)
        self._tree.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")
        self._tree.bind(
            "<Double-1>", lambda _event: self.load_selected_into_sliders()
        )

        record_bar = ttk.Frame(sequence)
        record_bar.grid(
            row=1, column=0, columnspan=2, sticky="ew", pady=(8, 3)
        )
        ttk.Label(record_bar, text="Hold after arrival:").grid(
            row=0, column=0
        )
        ttk.Spinbox(
            record_bar,
            from_=0.0,
            to=3600.0,
            increment=0.5,
            width=7,
            textvariable=self._hold_seconds,
        ).grid(row=0, column=1, padx=(4, 3))
        ttk.Label(record_bar, text="s").grid(row=0, column=2, padx=(0, 8))
        ttk.Button(
            record_bar,
            text="Record measured position",
            command=self.record_position,
        ).grid(row=0, column=3, padx=3)
        ttk.Button(
            record_bar, text="Replace selected", command=self.replace_selected
        ).grid(row=0, column=4, padx=3)
        ttk.Label(record_bar, text="Trajectory speed:").grid(
            row=1, column=0, pady=(6, 0)
        )
        ttk.Spinbox(
            record_bar,
            from_=0.5,
            to=30.0,
            increment=0.5,
            width=7,
            textvariable=self._trajectory_speed,
        ).grid(row=1, column=1, padx=(4, 3), pady=(6, 0))
        ttk.Label(record_bar, text="deg/s").grid(
            row=1, column=2, padx=(0, 8), pady=(6, 0)
        )
        ttk.Label(
            record_bar,
            text="Used to time synchronized arm trajectories",
        ).grid(row=1, column=3, columnspan=2, sticky="w", pady=(6, 0))

        edit_bar = ttk.Frame(sequence)
        edit_bar.grid(row=2, column=0, columnspan=2, sticky="ew", pady=3)
        actions = (
            ("To sliders", self.load_selected_into_sliders),
            ("Move up", lambda: self.move_selected(-1)),
            ("Move down", lambda: self.move_selected(1)),
            ("Delete", self.delete_selected),
            ("Clear", self.clear_sequence),
        )
        for column, (label, command) in enumerate(actions):
            ttk.Button(edit_bar, text=label, command=command).grid(
                row=0, column=column, padx=3, sticky="ew"
            )
            edit_bar.columnconfigure(column, weight=1)

        play_bar = ttk.Frame(sequence)
        play_bar.grid(row=3, column=0, columnspan=2, sticky="ew", pady=3)
        ttk.Button(
            play_bar, text="Execute sequence", command=self.start_playback
        ).grid(row=0, column=0, padx=3, sticky="ew")
        ttk.Button(
            play_bar, text="Stop playback + hold", command=self.stop_playback
        ).grid(row=0, column=1, padx=3, sticky="ew")
        ttk.Button(
            play_bar, text="Save JSON...", command=self.save_sequence_file
        ).grid(row=0, column=2, padx=3, sticky="ew")
        ttk.Button(
            play_bar, text="Load JSON...", command=self.load_sequence_file
        ).grid(row=0, column=3, padx=3, sticky="ew")
        for column in range(4):
            play_bar.columnconfigure(column, weight=1)
        ttk.Label(
            sequence, textvariable=self._sequence_status, anchor="w"
        ).grid(row=4, column=0, columnspan=2, sticky="ew", pady=(6, 0))

    def _on_joint_state(self, message: JointState) -> None:
        by_name = dict(zip(message.name, message.position))
        if not all(name in by_name for name in JOINT_NAMES):
            return
        measured = tuple(
            math.degrees(float(by_name[name])) for name in JOINT_NAMES
        )
        self._measured_positions_deg = measured
        for label, value in zip(self._measured_labels, measured):
            label.set(f"{value:.1f}\N{DEGREE SIGN}")

    def _on_control_state(self, message: String) -> None:
        try:
            state = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        connected = bool(state.get("connected"))
        port = state.get("serial_port", "?")
        telemetry_age = state.get("telemetry_age")
        telemetry_fresh = (
            connected
            and isinstance(telemetry_age, (int, float))
            and telemetry_age <= 1.0
        )
        if not telemetry_fresh:
            self._measured_positions_deg = None
            self._gripper_state = None
            for label in self._measured_labels:
                label.set("--.-\N{DEGREE SIGN}")
            self._gripper_state_label.set("State: --")
        if connected and isinstance(telemetry_age, (int, float)):
            detail = f", telemetry {telemetry_age:.2f} s old"
        else:
            detail = ""
        self._connection_status.set(
            f"Driver: {'connected' if connected else 'disconnected'} "
            f"({port}{detail})"
        )

    def _on_gripper_state(self, message: Float64) -> None:
        value = float(message.data)
        if not math.isfinite(value) or not 0.0 <= value <= 1.0:
            return
        self._gripper_state = value
        self._gripper_value.set(value)
        self._gripper_state_label.set(f"State: {value:.2f}")

    def _publish_positions_deg(
        self, positions: tuple[float, ...], source: str
    ) -> None:
        message = Float64MultiArray()
        message.data = [math.radians(value) for value in positions]
        self._position_publisher.publish(message)
        formatted = ", ".join(
            f"{value:.1f}\N{DEGREE SIGN}" for value in positions
        )
        self._status.set(f"{source}: {formatted}")

    def send_target(self) -> None:
        """Publish the six slider targets in ROS-standard radians."""
        positions = tuple(float(value.get()) for value in self._joint_values)
        self._publish_positions_deg(positions, "Sent slider target")

    def send_gripper(self) -> None:
        value = float(self._gripper_value.get())
        self._publish_gripper(value)
        self._status.set(f"Sent gripper target {value:.2f}")

    def _publish_gripper(self, value: float) -> None:
        self._gripper_publisher.publish(Float64(data=value))
        self._gripper_state = value
        self._gripper_value.set(value)
        self._gripper_state_label.set(f"State: {value:.2f}")

    def use_measured(self) -> None:
        if self._measured_positions_deg is None:
            self._status.set("Cannot copy: no measured joint state received")
            return
        for variable, value in zip(
            self._joint_values, self._measured_positions_deg
        ):
            variable.set(value)
        self._status.set("Copied measured position to sliders")

    def _read_hold_seconds(self) -> float | None:
        try:
            value = float(self._hold_seconds.get())
            if not math.isfinite(value) or value < 0.0:
                raise ValueError
            return value
        except ValueError:
            messagebox.showerror(
                "Invalid delay",
                "Delay must be a finite number of seconds >= 0.",
            )
            return None

    def _read_trajectory_speed(self) -> float | None:
        try:
            value = float(self._trajectory_speed.get())
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError
            return value
        except ValueError:
            messagebox.showerror(
                "Invalid trajectory speed",
                "Trajectory speed must be a finite number greater than zero.",
            )
            return None

    def record_position(self) -> None:
        """Append the latest measured robot pose to the sequence."""
        if self._measured_positions_deg is None:
            messagebox.showwarning(
                "No joint state",
                "No measured joint state has arrived. Start/connect the ROS 2 "
                "driver first.",
            )
            return
        if self._gripper_state is None:
            messagebox.showwarning(
                "No gripper state",
                "No gripper state has arrived from the ROS 2 driver yet.",
            )
            return
        hold_seconds = self._read_hold_seconds()
        if hold_seconds is None:
            return
        self._steps.append(
            SequenceStep(
                self._measured_positions_deg,
                hold_seconds,
                self._gripper_state,
            )
        )
        self._refresh_tree(select_index=len(self._steps) - 1)
        self._status.set(
            f"Recorded measured position as step {len(self._steps)}"
        )

    def _selected_index(self) -> int | None:
        selection = self._tree.selection()
        if not selection:
            return None
        return self._tree.index(selection[0])

    def replace_selected(self) -> None:
        index = self._selected_index()
        if index is None:
            self._status.set("Select a sequence step to replace")
            return
        if self._measured_positions_deg is None:
            self._status.set(
                "Cannot replace: no measured joint state received"
            )
            return
        if self._gripper_state is None:
            self._status.set("Cannot replace: no gripper state received")
            return
        hold_seconds = self._read_hold_seconds()
        if hold_seconds is None:
            return
        self._steps[index] = SequenceStep(
            self._measured_positions_deg,
            hold_seconds,
            self._gripper_state,
        )
        self._refresh_tree(select_index=index)
        self._status.set(
            f"Replaced step {index + 1} with measured position"
        )

    def load_selected_into_sliders(self) -> None:
        index = self._selected_index()
        if index is None:
            self._status.set("Select a sequence step first")
            return
        step = self._steps[index]
        for variable, value in zip(self._joint_values, step.positions_deg):
            variable.set(value)
        if step.gripper_position is not None:
            self._gripper_value.set(step.gripper_position)
        self._hold_seconds.set(f"{step.hold_seconds:g}")
        self._status.set(f"Loaded step {index + 1} into sliders")

    def move_selected(self, offset: int) -> None:
        index = self._selected_index()
        if index is None:
            self._status.set("Select a sequence step first")
            return
        destination = index + offset
        if destination < 0 or destination >= len(self._steps):
            return
        self._steps[index], self._steps[destination] = (
            self._steps[destination],
            self._steps[index],
        )
        self._refresh_tree(select_index=destination)

    def delete_selected(self) -> None:
        index = self._selected_index()
        if index is None:
            self._status.set("Select a sequence step to delete")
            return
        del self._steps[index]
        next_index = min(index, len(self._steps) - 1) if self._steps else None
        self._refresh_tree(select_index=next_index)

    def clear_sequence(self) -> None:
        if self._steps and not messagebox.askyesno(
            "Clear sequence", "Remove every recorded step?"
        ):
            return
        self.stop_playback(hold_robot=False)
        self._steps.clear()
        self._sequence_path = None
        self._refresh_tree()

    def _refresh_tree(self, select_index: int | None = None) -> None:
        self._tree.delete(*self._tree.get_children())
        for index, step in enumerate(self._steps):
            values = [
                index + 1,
                *(f"{value:.1f}" for value in step.positions_deg),
                (
                    f"{step.gripper_position:.2f}"
                    if step.gripper_position is not None
                    else "--"
                ),
                f"{step.hold_seconds:g}",
            ]
            item = self._tree.insert("", "end", values=values)
            if index == select_index:
                self._tree.selection_set(item)
                self._tree.focus(item)
                self._tree.see(item)
        filename = (
            self._sequence_path.name if self._sequence_path else "unsaved"
        )
        self._sequence_status.set(f"{len(self._steps)} step(s) — {filename}")

    def start_playback(self) -> None:
        if not self._steps:
            self._status.set("Cannot execute an empty sequence")
            return
        if self._measured_positions_deg is None:
            self._status.set(
                "Cannot execute: no current measured joint state received"
            )
            return
        speed = self._read_trajectory_speed()
        if speed is None:
            return
        if not self._trajectory_client.server_is_ready():
            self._status.set(
                f"Trajectory action {TRAJECTORY_ACTION} is unavailable"
            )
            return

        self.stop_playback(hold_robot=False)
        self._playback_steps = tuple(self._steps)
        samples, arrivals = interpolate_sequence(
            self._measured_positions_deg,
            self._playback_steps,
            speed_deg_s=speed,
        )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        for sample in samples:
            point = JointTrajectoryPoint()
            point.positions = [
                math.radians(value) for value in sample.positions_deg
            ]
            point.time_from_start = self._duration_message(
                sample.time_seconds
            )
            goal.trajectory.points.append(point)

        self._playback_generation += 1
        generation = self._playback_generation
        send_future = self._trajectory_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda future: self._on_trajectory_goal_response(
                future, generation, arrivals
            )
        )
        self._sequence_status.set(
            f"Sending synchronized trajectory with {len(samples)} points "
            f"at {speed:g} deg/s"
        )
        self._status.set(
            f"Executing {len(self._playback_steps)} recorded step(s) as a "
            "FollowJointTrajectory goal"
        )

    @staticmethod
    def _duration_message(seconds: float) -> Duration:
        whole_seconds = int(seconds)
        nanoseconds = round((seconds - whole_seconds) * 1_000_000_000)
        if nanoseconds >= 1_000_000_000:
            whole_seconds += 1
            nanoseconds -= 1_000_000_000
        return Duration(sec=whole_seconds, nanosec=nanoseconds)

    def _on_trajectory_goal_response(
        self, future, generation: int, arrival_times: list[float]
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            if generation == self._playback_generation:
                self._playback_steps = ()
                self._status.set(f"Could not send trajectory: {exc}")
            return

        if generation != self._playback_generation:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self._playback_steps = ()
            self._status.set("The robot driver rejected the trajectory")
            self._sequence_status.set("Trajectory rejected")
            return

        self._trajectory_goal_handle = goal_handle
        for index, arrival_time in enumerate(arrival_times):
            timer = self._root.after(
                max(1, round(arrival_time * 1000.0)),
                lambda step_index=index: self._on_waypoint_arrival(
                    step_index, generation
                ),
            )
            self._playback_timers.append(timer)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._on_trajectory_result(result, generation)
        )
        self._sequence_status.set(
            f"Executing synchronized trajectory: "
            f"{len(self._playback_steps)} step(s)"
        )

    def _on_waypoint_arrival(self, index: int, generation: int) -> None:
        if generation != self._playback_generation or not self._playback_steps:
            return
        step = self._playback_steps[index]
        if step.gripper_position is not None:
            self._publish_gripper(step.gripper_position)
        children = self._tree.get_children()
        if index < len(children):
            self._tree.selection_set(children[index])
            self._tree.see(children[index])
        self._sequence_status.set(
            f"Reached nominal step {index + 1}/{len(self._playback_steps)}; "
            f"holding for {step.hold_seconds:g} s"
        )

    def _on_trajectory_result(self, future, generation: int) -> None:
        if generation != self._playback_generation:
            return
        try:
            result = future.result().result
        except Exception as exc:
            self._status.set(f"Trajectory execution failed: {exc}")
            self._sequence_status.set("Trajectory execution failed")
        else:
            if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                count = len(self._playback_steps)
                self._status.set("Synchronized trajectory completed")
                self._sequence_status.set(
                    f"Playback finished ({count} step(s))"
                )
            else:
                detail = result.error_string or str(result.error_code)
                self._status.set(f"Trajectory execution failed: {detail}")
                self._sequence_status.set("Trajectory execution failed")
        self._trajectory_goal_handle = None
        self._playback_steps = ()
        self._playback_timers.clear()

    def stop_playback(self, hold_robot: bool = True) -> None:
        was_playing = bool(self._playback_steps)
        self._playback_generation += 1
        for timer in self._playback_timers:
            self._root.after_cancel(timer)
        self._playback_timers.clear()
        if self._trajectory_goal_handle is not None:
            self._trajectory_goal_handle.cancel_goal_async()
            self._trajectory_goal_handle = None
        self._playback_steps = ()
        if hold_robot:
            self._call_service("stop")
        if was_playing:
            self._sequence_status.set(
                f"Playback stopped — {len(self._steps)} step(s)"
            )

    def save_sequence_file(self) -> None:
        if not self._steps:
            self._status.set("Cannot save an empty sequence")
            return
        initial_name = (
            self._sequence_path.name
            if self._sequence_path
            else "robot_sequence.json"
        )
        path = filedialog.asksaveasfilename(
            title="Save robot sequence",
            defaultextension=".json",
            filetypes=(("JSON files", "*.json"), ("All files", "*.*")),
            initialfile=initial_name,
        )
        if not path:
            return
        try:
            save_sequence(path, self._steps)
        except OSError as exc:
            messagebox.showerror("Could not save sequence", str(exc))
            return
        self._sequence_path = Path(path)
        self._refresh_tree()
        self._status.set(f"Saved {len(self._steps)} steps to {path}")

    def load_sequence_file(self) -> None:
        path = filedialog.askopenfilename(
            title="Load robot sequence",
            filetypes=(("JSON files", "*.json"), ("All files", "*.*")),
        )
        if not path:
            return
        try:
            steps = load_sequence(path)
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            messagebox.showerror("Could not load sequence", str(exc))
            return
        self.stop_playback(hold_robot=False)
        self._steps = steps
        self._sequence_path = Path(path)
        self._refresh_tree(select_index=0 if steps else None)
        self._status.set(f"Loaded {len(steps)} steps from {path}")

    def _call_service(self, name: str) -> None:
        client = self._service_clients[name]
        if not client.service_is_ready():
            self._status.set(
                f"Service /{name} is unavailable; is the driver running?"
            )
            return
        self._status.set(f"Calling /{name}...")
        future = client.call_async(Trigger.Request())

        def completed(result_future) -> None:
            try:
                response = result_future.result()
                outcome = "OK" if response.success else "failed"
                self._status.set(
                    f"/{name}: {outcome} — {response.message}"
                )
            except Exception as exc:
                self._status.set(f"/{name} failed: {exc}")

        future.add_done_callback(completed)

    def _driver_service(self, name: str) -> None:
        """Cancel playback when needed, then invoke a driver service."""
        if name == "disconnect":
            self.stop_playback(hold_robot=False)
        self._call_service(name)

    def close(self) -> None:
        """Cancel local timers and close the window."""
        self.stop_playback(hold_robot=False)
        self._root.quit()
        self._root.destroy()

    def run(self) -> None:
        """Run Tk while periodically servicing ROS events on the UI thread."""
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
