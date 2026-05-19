import math
from dataclasses import dataclass
from typing import cast

from config import OdometryConfig
from controller import PositionSensor, Robot
from debug_logger import DebugLevel, DebugLogger


@dataclass
class Pose:
    """Continuous robot pose integrated from wheel encoders."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    distance: float = 0.0

    def copy(self) -> "Pose":
        """Return a detached copy of this pose."""
        return Pose(
            x=self.x,
            y=self.y,
            theta=self.theta,
            distance=self.distance,
        )


def normalise_angle(angle: float) -> float:
    """Normalise an angle to the range [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class Odometry:
    """Track robot movement using Pioneer 3AT wheel position sensors."""

    LEFT_SENSOR_NAMES = (
        "front left wheel sensor",
        "back left wheel sensor",
    )

    RIGHT_SENSOR_NAMES = (
        "front right wheel sensor",
        "back right wheel sensor",
    )

    def __init__(
        self,
        robot: Robot,
        config: OdometryConfig | None = None,
        debug_level: DebugLevel = DebugLevel.NONE,
    ) -> None:
        """Create odometry and enable the wheel position sensors."""
        self.config = config if config is not None else OdometryConfig()
        self.timestep = int(robot.getBasicTimeStep())
        self.logger = DebugLogger("Odometry", debug_level)
        self.left_sensors = self._load_sensors(robot, self.LEFT_SENSOR_NAMES)
        self.right_sensors = self._load_sensors(robot, self.RIGHT_SENSOR_NAMES)
        for sensor in self.left_sensors + self.right_sensors:
            sensor.enable(self.timestep)
        self.prev_left_angle: float | None = None
        self.prev_right_angle: float | None = None
        self.pose = Pose()
        self.start_pose = self.pose.copy()
        self.logger.debug(
            "__init__",
            f"tile_size={self.config.tile_size:.3f}, "
            f"wheel_radius={self.config.wheel_radius:.3f}, "
            f"axle_length={self.config.axle_length:.3f}, "
            f"forward_tolerance={self.config.forward_tolerance:.3f}, "
            f"turn_tolerance={math.degrees(self.config.turn_tolerance):.1f}deg, "
            f"forward_end_margin={self.config.forward_end_margin:.3f}",
        )

    @property
    def forward_tolerance(self) -> float:
        """Return the configured forward movement tolerance."""
        return self.config.forward_tolerance

    @property
    def turn_tolerance(self) -> float:
        """Return the configured turn angle tolerance."""
        return self.config.turn_tolerance

    def update(self) -> None:
        """Integrate one simulation step of encoder readings into pose."""
        left_angle = self._avg_angle(self.left_sensors)
        right_angle = self._avg_angle(self.right_sensors)
        if not math.isfinite(left_angle) or not math.isfinite(right_angle):
            self.logger.error(
                "update",
                f"invalid encoder value: left={left_angle}, right={right_angle}",
            )
            return
        if self.prev_left_angle is None or self.prev_right_angle is None:
            self.prev_left_angle = left_angle
            self.prev_right_angle = right_angle
            self.logger.debug(
                "update",
                f"encoder baseline set: left={left_angle:.3f}, right={right_angle:.3f}",
            )
            return
        delta_left_angle = left_angle - self.prev_left_angle
        delta_right_angle = right_angle - self.prev_right_angle
        self.prev_left_angle = left_angle
        self.prev_right_angle = right_angle
        left_distance = delta_left_angle * self.config.wheel_radius
        right_distance = delta_right_angle * self.config.wheel_radius
        # Differential-drive integration uses average wheel travel for forward
        # motion and left/right travel difference for heading change.
        delta_distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.config.axle_length
        # Integrating at the midpoint heading reduces drift on curved motion.
        midpoint_theta = self.pose.theta + delta_theta / 2.0
        self.pose.x += delta_distance * math.cos(midpoint_theta)
        self.pose.y += delta_distance * math.sin(midpoint_theta)
        self.pose.theta = normalise_angle(self.pose.theta + delta_theta)
        self.pose.distance += delta_distance
        self.logger.trace(
            "update",
            f"left_angle={left_angle:.3f}, "
            f"right_angle={right_angle:.3f}, "
            f"delta_distance={delta_distance:.3f}, "
            f"delta_theta={math.degrees(delta_theta):.2f}deg, "
            f"pose={self._format_pose(self.pose)}",
        )

    def start_action(self) -> None:
        """Record the current pose as the baseline for a movement action."""
        if not self._pose_is_valid(self.pose):
            self.logger.warn(
                "start_action",
                "cannot start action because current pose is invalid",
            )
            return
        self.start_pose = self.pose.copy()
        self.logger.debug(
            "start_action",
            f"started at {self._format_pose(self.start_pose)}",
        )

    def forward_almost_complete(self) -> bool:
        """Return True when it is too late to safely abort a one-tile move."""
        return (
            self.forward_error()
            >= self.config.tile_size - self.config.forward_end_margin
        )

    def turned_angle(self) -> float:
        """Return absolute angle turned since the current action started."""
        return abs(self.turn_error())

    def forward_complete(self) -> bool:
        """Return True when approximately one tile has been travelled."""
        return self.forward_error() >= (
            self.config.tile_size - self.config.forward_tolerance
        )

    def turn_90_complete(self) -> bool:
        """Return True when approximately 90 degrees have been turned."""
        return self.turned_angle() >= math.pi / 2 - self.config.turn_tolerance

    def turn_180_complete(self) -> bool:
        """Return True when approximately 180 degrees have been turned."""
        return self.turned_angle() >= math.pi - self.config.turn_tolerance

    def forward_error(self) -> float:
        """Return signed distance travelled since the current action started."""
        return self.pose.distance - self.start_pose.distance

    def turn_error(self) -> float:
        """Return signed heading change since the current action started."""
        return normalise_angle(self.pose.theta - self.start_pose.theta)

    def recovery_complete(self) -> bool:
        """Return True when recovery has returned to the action start pose."""
        return (
            abs(self.forward_error()) <= self.config.forward_tolerance
            and abs(self.turn_error()) <= self.config.turn_tolerance
        )

    def debug_status(self) -> None:
        """Log the current pose and action error."""
        self.logger.debug(
            "debug_status",
            f"current={self._format_pose(self.pose)}, "
            f"start={self._format_pose(self.start_pose)}, "
            f"forward_error={self.forward_error():.3f}, "
            f"turn_error={math.degrees(self.turn_error()):.1f}deg",
        )

    def _load_sensors(
        self,
        robot: Robot,
        names: tuple[str, ...],
    ) -> list[PositionSensor]:
        """Load named Webots position sensors from the robot."""
        return [cast(PositionSensor, robot.getDevice(name)) for name in names]

    def _avg_angle(self, sensors: list[PositionSensor]) -> float:
        """Return the average angle reading across paired wheel sensors."""
        values = [sensor.getValue() for sensor in sensors]
        return sum(values) / len(values)

    def _pose_is_valid(self, pose: Pose) -> bool:
        """Return True when every pose component is finite."""
        return (
            math.isfinite(pose.x)
            and math.isfinite(pose.y)
            and math.isfinite(pose.theta)
            and math.isfinite(pose.distance)
        )

    def _format_pose(self, pose: Pose) -> str:
        """Format a pose for debug output."""
        return (
            f"Pose(x={pose.x:.3f}, "
            f"y={pose.y:.3f}, "
            f"theta={math.degrees(pose.theta):.1f}deg, "
            f"distance={pose.distance:.3f})"
        )
