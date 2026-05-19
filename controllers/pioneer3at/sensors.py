from dataclasses import dataclass
from typing import cast

from config import SensorConfig
from controller import DistanceSensor, Robot
from debug_logger import DebugLevel, DebugLogger
from domain import RelativeDirection

SonarGroup = tuple[str, ...]


@dataclass(frozen=True)
class SideReadings:
    """Averaged raw sonar values for the four side sensor groups."""

    left_front: float
    left_rear: float
    right_front: float
    right_rear: float


class Sensors:
    """Read Pioneer 3AT sonars and derive grid/blocking alignment signals."""

    DIR_SONARS: dict[RelativeDirection, SonarGroup] = {
        RelativeDirection.FRONT: ("so2", "so3", "so4", "so5"),
        RelativeDirection.LEFT: ("so0", "so1", "so14", "so15"),
        RelativeDirection.BACK: ("so10", "so11", "so12", "so13"),
        RelativeDirection.RIGHT: ("so6", "so7", "so8", "so9"),
    }

    SIDE_SONARS: dict[str, SonarGroup] = {
        "left_front": ("so0", "so1"),
        "left_rear": ("so14", "so15"),
        "right_front": ("so6", "so7"),
        "right_rear": ("so8", "so9"),
    }

    def __init__(
        self,
        robot: Robot,
        config: SensorConfig = SensorConfig(),
        debug_level: DebugLevel = DebugLevel.NONE,
    ) -> None:
        """Enable sonar sensors and store threshold configuration."""
        self.config = config
        self.logger = DebugLogger("Sensors", debug_level)
        self.sonars: dict[str, DistanceSensor] = {}
        for i in range(16):
            name = f"so{i}"
            sensor = cast(DistanceSensor, robot.getDevice(name))
            timestep = int(robot.getBasicTimeStep())
            sensor.enable(timestep)
            self.sonars[name] = sensor
        self._parallel_filtered: float | None = None
        self._centre_filtered: float | None = None
        self.logger.debug(
            "__init__",
            f"blocked={self.config.blocked}, "
            f"too_close={self.config.too_close}, "
            f"parallel_conflict={self.config.parallel_conflict}, "
            f"parallel_alpha={self.config.parallel_alpha}, "
            f"centre_alpha={self.config.centre_alpha}",
        )

    def _raw(self, names: SonarGroup) -> list[float]:
        """Return raw Webots sonar readings for a sensor group."""
        return [self.sonars[name].getValue() for name in names]

    def _avg(self, names: SonarGroup) -> float:
        """Return the average non-zero raw proximity for a sensor group."""
        values = self._raw(names)
        active = [value for value in values if value > 0.0]
        if len(active) == 0:
            return 0.0
        return sum(active) / len(active)

    def is_direction_blocked(self, direction: RelativeDirection) -> bool:
        """Return True when a relative direction is blocked by sonar."""
        proximity = self._avg(self.DIR_SONARS[direction])
        return proximity >= self.config.blocked

    def is_direction_too_close(self, direction: RelativeDirection) -> bool:
        """Return True when a relative direction is dangerously close."""
        proximity = self._avg(self.DIR_SONARS[direction])
        return proximity >= self.config.too_close

    def is_direction_free(self, direction: RelativeDirection) -> bool:
        """Return True when a relative direction is clear enough to enter."""
        return not self.is_direction_blocked(direction)

    def scan_neighbours(self) -> dict[RelativeDirection, bool]:
        """Return enterability for all adjacent grid directions."""
        neighbours = {
            direction: self.is_direction_free(direction)
            for direction in RelativeDirection
        }
        self.logger.debug(
            "scan_neighbours",
            ", ".join(
                f"{direction.name}={'FREE' if is_free else 'BLOCKED'}"
                for direction, is_free in neighbours.items()
            ),
        )
        return neighbours

    def _sides(self) -> SideReadings:
        """Return averaged side-wall sonar readings."""
        return SideReadings(
            left_front=self._avg(self.SIDE_SONARS["left_front"]),
            left_rear=self._avg(self.SIDE_SONARS["left_rear"]),
            right_front=self._avg(self.SIDE_SONARS["right_front"]),
            right_rear=self._avg(self.SIDE_SONARS["right_rear"]),
        )

    def _left_ok(self, sides: SideReadings) -> bool:
        """Return True when both left side groups see a reliable wall."""
        return (
            sides.left_front >= self.config.blocked
            and sides.left_rear >= self.config.blocked
        )

    def _right_ok(self, sides: SideReadings) -> bool:
        """Return True when both right side groups see a reliable wall."""
        return (
            sides.right_front >= self.config.blocked
            and sides.right_rear >= self.config.blocked
        )

    def parallel_error(self) -> float | None:
        """Estimate angular error against nearby side walls."""
        sides = self._sides()
        left_ok = self._left_ok(sides)
        right_ok = self._right_ok(sides)
        errors: list[float] = []
        if left_ok:
            errors.append(sides.left_front - sides.left_rear)
        if right_ok:
            errors.append(-(sides.right_front - sides.right_rear))
        self.logger.trace(
            "parallel_error",
            f"LF={sides.left_front:.2f}, "
            f"LR={sides.left_rear:.2f}, "
            f"RF={sides.right_front:.2f}, "
            f"RR={sides.right_rear:.2f}, "
            f"left_ok={left_ok}, "
            f"right_ok={right_ok}, "
            f"errors={errors}",
        )
        if len(errors) == 0:
            self._parallel_filtered = None
            return None
        if (
            len(errors) == 2
            and errors[0] * errors[1] < 0.0
            and abs(errors[0]) > self.config.parallel_conflict
            and abs(errors[1]) > self.config.parallel_conflict
        ):
            # Strong opposite-sign side errors are ambiguous, so skip correction.
            self.logger.debug(
                "parallel_error",
                "conflict detected, ignoring correction",
            )
            return None
        raw = sum(errors) / len(errors)
        if self._parallel_filtered is None:
            self._parallel_filtered = raw
        else:
            self._parallel_filtered = (
                self.config.parallel_alpha * self._parallel_filtered
                + (1.0 - self.config.parallel_alpha) * raw
            )
        self.logger.trace(
            "parallel_error",
            f"raw={raw:.2f}, filtered={self._parallel_filtered:.2f}",
        )
        return self._parallel_filtered

    def left_right_diff(self) -> float | None:
        """Estimate lateral offset between reliable left and right walls."""
        sides = self._sides()
        left_ok = self._left_ok(sides)
        right_ok = self._right_ok(sides)
        if not left_ok or not right_ok:
            self._centre_filtered = None
            self.logger.trace(
                "left_right_diff",
                f"unavailable left_ok={left_ok}, right_ok={right_ok}",
            )
            return None
        left = (sides.left_front + sides.left_rear) / 2.0
        right = (sides.right_front + sides.right_rear) / 2.0
        raw = left - right
        if self._centre_filtered is None:
            self._centre_filtered = raw
        else:
            self._centre_filtered = (
                self.config.centre_alpha * self._centre_filtered
                + (1.0 - self.config.centre_alpha) * raw
            )
        self.logger.trace(
            "left_right_diff",
            f"left={left:.2f}, "
            f"right={right:.2f}, "
            f"raw={raw:.2f}, "
            f"filtered={self._centre_filtered:.2f}",
        )
        return self._centre_filtered

    def front_back_diff(self) -> float | None:
        """Estimate fore/aft offset after the robot turns 90 degrees."""
        front = self._avg(self.DIR_SONARS[RelativeDirection.FRONT])
        back = self._avg(self.DIR_SONARS[RelativeDirection.BACK])
        front_ok = front >= self.config.blocked
        back_ok = back >= self.config.blocked
        if not front_ok or not back_ok:
            self.logger.trace(
                "front_back_diff",
                f"unavailable front_ok={front_ok}, back_ok={back_ok}",
            )
            return None
        diff = front - back

        self.logger.trace(
            "front_back_diff",
            f"front={front:.2f}, back={back:.2f}, diff={diff:.2f}",
        )
        return diff
