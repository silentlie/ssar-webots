import math
from dataclasses import dataclass
from typing import cast

from controller import PositionSensor, Robot


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    distance: float = 0.0

    def copy(self) -> "Pose":
        return Pose(
            x=self.x,
            y=self.y,
            theta=self.theta,
            distance=self.distance,
        )


def normalize_angle(angle: float) -> float:
    """Normalize an angle to the range [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class Odometry:
    """
    Tracks robot movement using Pioneer 3AT wheel encoders.

    This class only measures movement.
    It does not control the wheel motors.
    """

    def __init__(
        self,
        robot: Robot,
        tile_size: float,
        wheel_radius: float = 0.11,
        axle_length: float = 0.585,
        forward_tolerance: float = 0.01,
        turn_tolerance: float = math.radians(1),
        forward_end_margin: float = 0.20,
        debug: bool = False,
    ) -> None:
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())

        self.tile_size = tile_size
        self.wheel_radius = wheel_radius
        self.axle_length = axle_length
        self.forward_tolerance = forward_tolerance
        self.turn_tolerance = turn_tolerance
        self.forward_end_margin = forward_end_margin
        self.debug = debug

        self.left_sensors = [
            cast(PositionSensor, robot.getDevice("front left wheel sensor")),
            cast(PositionSensor, robot.getDevice("back left wheel sensor")),
        ]

        self.right_sensors = [
            cast(PositionSensor, robot.getDevice("front right wheel sensor")),
            cast(PositionSensor, robot.getDevice("back right wheel sensor")),
        ]

        for sensor in self.left_sensors + self.right_sensors:
            sensor.enable(self.timestep)

        # Do not read encoder values here.
        # Webots may not have valid sensor values before the first step.
        self.previous_left_angle: float | None = None
        self.previous_right_angle: float | None = None

        self.current_pose = Pose()
        self.action_start_pose = self.current_pose.copy()

        self._debug(
            "Odometry initialized: "
            f"tile_size={self.tile_size:.3f}, "
            f"wheel_radius={self.wheel_radius:.3f}, "
            f"axle_length={self.axle_length:.3f}, "
            f"forward_tolerance={self.forward_tolerance:.3f}, "
            f"turn_tolerance={math.degrees(self.turn_tolerance):.1f}deg, "
            f"forward_end_margin={self.forward_end_margin:.3f}"
        )

    def update(self) -> None:
        """
        Update the estimated pose from wheel encoder readings.

        Call this once per simulation loop.
        """
        current_left_angle = self._average_left_angle()
        current_right_angle = self._average_right_angle()

        if not math.isfinite(current_left_angle) or not math.isfinite(
            current_right_angle
        ):
            self._debug(
                f"Skipping odometry update because encoder value is invalid: "
                f"left={current_left_angle}, right={current_right_angle}"
            )
            return

        if self.previous_left_angle is None or self.previous_right_angle is None:
            self.previous_left_angle = current_left_angle
            self.previous_right_angle = current_right_angle
            self._debug(
                f"Encoder baseline set: "
                f"left={current_left_angle:.3f}, right={current_right_angle:.3f}"
            )
            return

        delta_left_angle = current_left_angle - self.previous_left_angle
        delta_right_angle = current_right_angle - self.previous_right_angle

        self.previous_left_angle = current_left_angle
        self.previous_right_angle = current_right_angle

        left_distance = delta_left_angle * self.wheel_radius
        right_distance = delta_right_angle * self.wheel_radius

        delta_distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.axle_length

        midpoint_theta = self.current_pose.theta + delta_theta / 2.0

        self.current_pose.x += delta_distance * math.cos(midpoint_theta)
        self.current_pose.y += delta_distance * math.sin(midpoint_theta)
        self.current_pose.theta = normalize_angle(self.current_pose.theta + delta_theta)
        self.current_pose.distance += delta_distance

    def start_action(self) -> None:
        """
        Mark the current pose as the start of a new movement action.

        Call this before each one-tile forward movement or turn.
        """
        if not self._pose_is_valid(self.current_pose):
            self._debug("Cannot start action because current_pose is invalid")
            return

        self.action_start_pose = self.current_pose.copy()
        self._debug(f"Action started at {self._format_pose(self.action_start_pose)}")

    def signed_forward_distance(self) -> float:
        return self.current_pose.distance - self.action_start_pose.distance

    def forward_almost_complete(self) -> bool:
        return (
            self.signed_forward_distance() >= self.tile_size - self.forward_end_margin
        )

    def signed_turned_angle(self) -> float:
        """
        Return signed angle turned since start_action(), in radians.

        Positive usually means left turn.
        Negative usually means right turn.
        """
        return normalize_angle(self.current_pose.theta - self.action_start_pose.theta)

    def turned_angle(self) -> float:
        """Return absolute angle turned since start_action(), in radians."""
        return abs(self.signed_turned_angle())

    def forward_complete(self) -> bool:
        """Return True when approximately one tile has been traveled."""
        return self.signed_forward_distance() >= self.tile_size - self.forward_tolerance

    def turn_90_complete(self) -> bool:
        """Return True when approximately 90 degrees has been turned."""
        return self.turned_angle() >= math.pi / 2 - self.turn_tolerance

    def turn_180_complete(self) -> bool:
        """Return True when approximately 180 degrees has been turned."""
        return self.turned_angle() >= math.pi - self.turn_tolerance

    def forward_error(self) -> float:
        """
        Return signed distance from the action start pose.

        Positive: robot is ahead of action start.
        Negative: robot is behind action start.
        """
        return self.current_pose.distance - self.action_start_pose.distance

    def turn_error(self) -> float:
        """
        Return signed angle from the action start pose.

        Positive: robot has turned left from action start.
        Negative: robot has turned right from action start.
        """
        return normalize_angle(self.current_pose.theta - self.action_start_pose.theta)

    def recovery_complete(self) -> bool:
        return (
            abs(self.forward_error()) <= self.forward_tolerance
            and abs(self.turn_error()) <= self.turn_tolerance
        )

    def debug_status(self) -> None:
        """
        Print the current odometry state.

        Call this manually when debugging instead of printing every update loop.
        """
        self._debug(
            f"current={self._format_pose(self.current_pose)}, "
            f"action_start={self._format_pose(self.action_start_pose)}, "
            f"forward_error={self.forward_error():.3f}, "
            f"turn_error={math.degrees(self.turn_error()):.1f}deg"
        )

    def _average_left_angle(self) -> float:
        values = [sensor.getValue() for sensor in self.left_sensors]
        return sum(values) / len(values)

    def _average_right_angle(self) -> float:
        values = [sensor.getValue() for sensor in self.right_sensors]
        return sum(values) / len(values)

    def _pose_is_valid(self, pose: Pose) -> bool:
        return (
            math.isfinite(pose.x)
            and math.isfinite(pose.y)
            and math.isfinite(pose.theta)
            and math.isfinite(pose.distance)
        )

    def _debug(self, message: str) -> None:
        if self.debug:
            print(f"[Odometry] {message}")

    def _format_pose(self, pose: Pose) -> str:
        return (
            f"Pose(x={pose.x:.3f}, "
            f"y={pose.y:.3f}, "
            f"theta={math.degrees(pose.theta):.1f}deg, "
            f"distance={pose.distance:.3f})"
        )
