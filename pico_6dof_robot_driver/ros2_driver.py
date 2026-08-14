"""ROS 2 node exposing the Pico robot through standard controller interfaces."""

from __future__ import annotations

import json
import math
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from control_msgs.msg import GripperCommand as GripperCommandMsg
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, String
from std_srvs.srv import Trigger

from .protocol import JOINT_COUNT
from .serial_robot import SerialRobot


TRAJECTORY_ACTION = "scaled_joint_trajectory_controller/follow_joint_trajectory"
POSITION_TOPIC = "forward_position_controller/commands"
GRIPPER_ACTION = "robotiq_2f_urcap_adapter/gripper_command"
GRIPPER_TOPIC = "robotiq_2f_urcap_adapter/gripper_command_topic"


def duration_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


class Pico6DOFDriver(Node):
    def __init__(self) -> None:
        super().__init__("pico_6dof_driver")
        self._declare_parameters()
        self._joint_names = tuple(str(name) for name in self._param("joint_names"))
        if len(self._joint_names) != JOINT_COUNT or len(set(self._joint_names)) != JOINT_COUNT:
            raise ValueError("joint_names must contain six unique names")

        self._robot: SerialRobot | None = None
        self._connection_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._last_positions: tuple[float, ...] | None = None
        self._last_state_time = 0.0
        self._last_gripper_position = 0.0
        self._active_goal = None

        state_group = MutuallyExclusiveCallbackGroup()
        command_group = ReentrantCallbackGroup()
        self._joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        self._control_state_pub = self.create_publisher(String, "control_state", 10)
        self._gripper_pub = self.create_publisher(Float64, "gripper_position", 10)
        self.create_subscription(Float64MultiArray, POSITION_TOPIC, self._on_position_command, 10,
                                 callback_group=command_group)
        self.create_subscription(Float64, "gripper_position_cmd", self._on_gripper_command, 10,
                                 callback_group=command_group)
        self.create_subscription(GripperCommandMsg, GRIPPER_TOPIC, self._on_gripper_adapter_topic, 10,
                                 callback_group=command_group)

        self._trajectory_server = ActionServer(
            self, FollowJointTrajectory, TRAJECTORY_ACTION,
            execute_callback=self._execute_trajectory,
            goal_callback=self._trajectory_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=command_group,
        )
        self._gripper_server = ActionServer(
            self, GripperCommand, GRIPPER_ACTION,
            execute_callback=self._execute_gripper,
            goal_callback=self._gripper_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=command_group,
        )

        self.create_service(Trigger, "connect", self._on_connect, callback_group=command_group)
        self.create_service(Trigger, "disconnect", self._on_disconnect, callback_group=command_group)
        self.create_service(Trigger, "stop", self._on_stop, callback_group=command_group)
        self.create_service(Trigger, "home_j1", self._command_service("CMD_J1_INIT", "J1 homing started"),
                            callback_group=command_group)
        self.create_service(Trigger, "save_zeros", self._command_service("SAVE_ZEROS", "Encoder zeros saved"),
                            callback_group=command_group)
        self.create_service(Trigger, "load_zeros", self._command_service("LOAD_ZEROS", "Encoder zeros loaded"),
                            callback_group=command_group)

        publish_rate = max(float(self._param("state_publish_rate")), 1.0)
        self.create_timer(1.0 / publish_rate, self._publish_state, callback_group=state_group)
        self.create_timer(0.5, self._publish_control_state, callback_group=state_group)

        if bool(self._param("auto_connect")):
            try:
                self._connect()
            except Exception as exc:
                self.get_logger().error(f"Automatic serial connection failed: {exc}")

    def _declare_parameters(self) -> None:
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("serial_timeout", 0.05)
        self.declare_parameter("auto_connect", True)
        self.declare_parameter("set_auto_mode_on_connect", True)
        self.declare_parameter("state_publish_rate", 30.0)
        self.declare_parameter("state_timeout", 0.5)
        self.declare_parameter("joint_names", [f"joint_{index}" for index in range(1, 7)])
        self.declare_parameter("default_velocity", math.radians(10.0))
        self.declare_parameter("max_velocity", math.radians(30.0))
        self.declare_parameter("trajectory_goal_tolerance", 0.05)
        self.declare_parameter("trajectory_goal_time_tolerance", 2.0)
        self.declare_parameter("gripper_open_degrees", 10.0)
        self.declare_parameter("gripper_close_degrees", 170.0)
        self.declare_parameter("gripper_settle_time", 0.5)

    def _param(self, name: str):
        return self.get_parameter(name).value

    def _connect(self) -> None:
        with self._connection_lock:
            if self._robot is not None and self._robot.is_connected:
                return
            from .protocol import FirmwareProtocol

            robot = SerialRobot(
                port=str(self._param("serial_port")),
                baudrate=int(self._param("baudrate")),
                timeout=float(self._param("serial_timeout")),
                protocol=FirmwareProtocol(),
            )
            robot.connect()
            try:
                if bool(self._param("set_auto_mode_on_connect")):
                    robot.send("SET_MODE_AUTO")
            except Exception:
                robot.disconnect()
                raise
            self._robot = robot
            self.get_logger().info(f"Connected to Pico robot on {robot.port} at {robot.baudrate} baud")

    def _require_robot(self) -> SerialRobot:
        if self._robot is None or not self._robot.is_connected:
            self._connect()
        assert self._robot is not None
        return self._robot

    def _on_connect(self, _request, response):
        try:
            already = self._robot is not None and self._robot.is_connected
            self._connect()
            response.success = True
            response.message = "Already connected" if already else "Connected"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _on_disconnect(self, _request, response):
        with self._connection_lock:
            if self._robot is not None:
                self._robot.disconnect()
                self._robot = None
        response.success = True
        response.message = "Disconnected"
        return response

    def _on_stop(self, _request, response):
        try:
            with self._command_lock:
                held = self._require_robot().hold()
            response.success = held
            response.message = "Holding current measured position" if held else "No telemetry available"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _command_service(self, command: str, success_message: str):
        def callback(_request, response):
            try:
                self._require_robot().send(command)
                response.success = True
                response.message = success_message
            except Exception as exc:
                response.success = False
                response.message = str(exc)
            return response
        return callback

    def _send_positions(self, positions, velocity: float | None = None) -> None:
        target = [float(value) for value in positions]
        if len(target) != JOINT_COUNT or not all(math.isfinite(value) for value in target):
            raise ValueError("joint command must contain six finite positions in radians")
        default_speed = float(self._param("default_velocity"))
        requested_speed = default_speed if velocity is None else abs(float(velocity))
        if requested_speed <= 1e-9:
            requested_speed = default_speed
        speed = min(requested_speed, float(self._param("max_velocity")))
        if not math.isfinite(speed) or speed <= 0.0:
            raise ValueError("command velocity and max_velocity must be finite and greater than zero")
        with self._command_lock:
            self._require_robot().send_configuration(target, speed)

    def _on_position_command(self, msg: Float64MultiArray) -> None:
        try:
            self._send_positions(msg.data)
        except Exception as exc:
            self.get_logger().error(f"Rejected {POSITION_TOPIC} command: {exc}")

    def _gripper_norm_from_adapter(self, position_m: float) -> float:
        # Match the common Robotiq adapter convention: 0.0 m closed, 0.085 m open.
        return min(max(1.0 - float(position_m) / 0.085, 0.0), 1.0)

    def _send_gripper(self, normalized: float) -> None:
        position = float(normalized)
        command = self._require_robot().protocol.gripper_command(
            position,
            float(self._param("gripper_open_degrees")),
            float(self._param("gripper_close_degrees")),
        )
        with self._command_lock:
            self._require_robot().send(command)
        self._last_gripper_position = position

    def _on_gripper_command(self, msg: Float64) -> None:
        try:
            self._send_gripper(float(msg.data))
        except Exception as exc:
            self.get_logger().error(f"Rejected gripper command: {exc}")

    def _on_gripper_adapter_topic(self, msg: GripperCommandMsg) -> None:
        try:
            self._send_gripper(self._gripper_norm_from_adapter(msg.position))
        except Exception as exc:
            self.get_logger().error(f"Rejected adapter gripper command: {exc}")

    def _gripper_goal(self, goal) -> GoalResponse:
        return GoalResponse.ACCEPT if math.isfinite(float(goal.command.position)) else GoalResponse.REJECT

    def _execute_gripper(self, goal_handle):
        result = GripperCommand.Result()
        try:
            normalized = self._gripper_norm_from_adapter(goal_handle.request.command.position)
            self._send_gripper(normalized)
            deadline = time.monotonic() + max(float(self._param("gripper_settle_time")), 0.0)
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.position = float(goal_handle.request.command.position)
                    return result
                time.sleep(0.02)
            goal_handle.succeed()
            result.position = float(goal_handle.request.command.position)
            result.reached_goal = True
        except Exception as exc:
            goal_handle.abort()
            self.get_logger().error(f"Gripper action failed: {exc}")
        return result

    def _validate_trajectory(self, goal) -> None:
        trajectory = goal.trajectory
        if not trajectory.points:
            raise ValueError("trajectory must contain at least one point")
        if set(trajectory.joint_names) != set(self._joint_names):
            raise ValueError(f"joint_names must match {list(self._joint_names)}")
        previous = -1.0
        for point in trajectory.points:
            if len(point.positions) != JOINT_COUNT:
                raise ValueError("each trajectory point must contain six positions")
            if not all(math.isfinite(float(value)) for value in point.positions):
                raise ValueError("trajectory positions must be finite")
            current = duration_seconds(point.time_from_start)
            if current < 0.0 or current <= previous:
                raise ValueError("time_from_start values must be non-negative and strictly increasing")
            previous = current

    def _trajectory_goal(self, goal) -> GoalResponse:
        try:
            self._validate_trajectory(goal)
        except ValueError as exc:
            self.get_logger().error(f"Rejecting trajectory: {exc}")
            return GoalResponse.REJECT
        if self._active_goal is not None:
            self.get_logger().error("Rejecting trajectory because another trajectory is active")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _ordered_positions(self, positions, names) -> list[float]:
        by_name = dict(zip(names, positions))
        return [float(by_name[name]) for name in self._joint_names]

    def _publish_trajectory_feedback(self, goal_handle, desired: list[float]) -> None:
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = list(self._joint_names)
        feedback.desired.positions = desired
        telemetry, _ = self._require_robot().latest_telemetry()
        if telemetry is not None:
            feedback.actual.positions = list(telemetry.positions_rad)
            feedback.error.positions = [wanted - actual for wanted, actual in zip(desired, telemetry.positions_rad)]
        goal_handle.publish_feedback(feedback)

    def _execute_trajectory(self, goal_handle):
        result = FollowJointTrajectory.Result()
        self._active_goal = goal_handle
        try:
            self._validate_trajectory(goal_handle.request)
            names = list(goal_handle.request.trajectory.joint_names)
            telemetry, _ = self._require_robot().latest_telemetry()
            previous_positions = list(telemetry.positions_rad) if telemetry else None
            previous_time = 0.0
            trajectory_start = time.monotonic()

            for point in goal_handle.request.trajectory.points:
                target = self._ordered_positions(point.positions, names)
                target_time = duration_seconds(point.time_from_start)
                segment_time = max(target_time - previous_time, 0.001)
                if previous_positions is None:
                    speed = float(self._param("default_velocity"))
                else:
                    speed = max(abs(a - b) for a, b in zip(target, previous_positions)) / segment_time
                self._send_positions(target, speed)
                deadline = trajectory_start + target_time
                while time.monotonic() < deadline:
                    if goal_handle.is_cancel_requested:
                        with self._command_lock:
                            self._require_robot().hold()
                        goal_handle.canceled()
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        result.error_string = "Trajectory canceled; holding measured position"
                        return result
                    self._publish_trajectory_feedback(goal_handle, target)
                    time.sleep(min(0.05, max(deadline - time.monotonic(), 0.001)))
                previous_positions = target
                previous_time = target_time

            tolerance = float(self._param("trajectory_goal_tolerance"))
            deadline = time.monotonic() + max(float(self._param("trajectory_goal_time_tolerance")), 0.0)
            final_target = previous_positions
            while final_target is not None:
                telemetry, _ = self._require_robot().latest_telemetry()
                if telemetry and max(abs(a - b) for a, b in zip(final_target, telemetry.positions_rad)) <= tolerance:
                    goal_handle.succeed()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "Trajectory reached final point"
                    return result
                if time.monotonic() >= deadline:
                    break
                if goal_handle.is_cancel_requested:
                    with self._command_lock:
                        self._require_robot().hold()
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "Trajectory canceled; holding measured position"
                    return result
                self._publish_trajectory_feedback(goal_handle, final_target)
                time.sleep(0.05)

            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            result.error_string = "Robot did not reach final point within tolerance"
        except ValueError as exc:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
        except Exception as exc:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
        finally:
            self._active_goal = None
        return result

    def _publish_state(self) -> None:
        robot = self._robot
        if robot is None or not robot.is_connected:
            return
        telemetry, received_at = robot.latest_telemetry()
        if telemetry is None:
            return
        now_monotonic = time.monotonic()
        if now_monotonic - received_at > float(self._param("state_timeout")):
            return
        velocities = [0.0] * JOINT_COUNT
        if self._last_positions is not None and received_at > self._last_state_time:
            dt = received_at - self._last_state_time
            velocities = [(current - previous) / dt for current, previous in zip(
                telemetry.positions_rad, self._last_positions)]
        self._last_positions = telemetry.positions_rad
        self._last_state_time = received_at

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self._joint_names)
        message.position = list(telemetry.positions_rad)
        message.velocity = velocities
        self._joint_state_pub.publish(message)
        self._gripper_pub.publish(Float64(data=self._last_gripper_position))

    def _publish_control_state(self) -> None:
        robot = self._robot
        telemetry_age = None
        last_error = None
        if robot is not None:
            _, received_at = robot.latest_telemetry()
            telemetry_age = time.monotonic() - received_at if received_at else None
            last_error = str(robot.last_error) if robot.last_error else None
        state = {
            "connected": bool(robot and robot.is_connected),
            "serial_port": str(self._param("serial_port")),
            "telemetry_age": telemetry_age,
            "trajectory_active": self._active_goal is not None,
            "last_error": last_error,
        }
        self._control_state_pub.publish(String(data=json.dumps(state)))

    def destroy_node(self) -> bool:
        if self._robot is not None:
            self._robot.disconnect()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Pico6DOFDriver()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
