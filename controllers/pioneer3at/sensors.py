from typing import cast

from controller import DistanceSensor, Robot
from gridMap import RelativeDirection


class Sensors:
    """
    Groups the Pioneer 3AT sonars into grid-facing directions and alignment signals.

    Webots distance sensors return raw proximity values here. Higher values mean
    closer obstacles, so thresholds are calibrated against those raw readings.
    """

    def __init__(
        self,
        robot: Robot,
        blocked_threshold: float = 900.0,
        too_close_threshold: float = 980.0,
        debug: bool = False,
    ) -> None:
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())
        self.debug = debug

        self.sonars: dict[str, DistanceSensor] = {}

        for i in range(16):
            name = f"so{i}"
            sensor = cast(DistanceSensor, self.robot.getDevice(name))
            sensor.enable(self.timestep)
            self.sonars[name] = sensor

        self.direction_sonars: dict[RelativeDirection, list[str]] = {
            RelativeDirection.FRONT: ["so2", "so3", "so4", "so5"],
            RelativeDirection.LEFT: ["so0", "so1", "so14", "so15"],
            RelativeDirection.BACK: ["so10", "so11", "so12", "so13"],
            RelativeDirection.RIGHT: ["so6", "so7", "so8", "so9"],
        }

        # These are sonar raw-value thresholds, not metres.
        self.blocked_threshold = blocked_threshold
        self.too_close_threshold = too_close_threshold

        self.side_sonars: dict[str, list[str]] = {
            "left_front": ["so0", "so1"],
            "left_rear": ["so14", "so15"],
            "right_front": ["so6", "so7"],
            "right_rear": ["so8", "so9"],
        }

        # Use a slightly lower threshold than blocked_threshold if wall detection
        # drops in and out too often.
        self.side_wall_threshold = self.blocked_threshold

        self.parallel_conflict_threshold = 15.0
        self.filtered_parallel_error: float | None = None
        self.parallel_smoothing = 0.7

        self.filtered_left_right_diff: float | None = None
        self.left_right_smoothing = 0.8

    def _raw_values(self, sensor_names: list[str]) -> list[float]:
        return [self.sonars[name].getValue() for name in sensor_names]

    def _average_proximity(self, sensor_names: list[str]) -> float:
        values = self._raw_values(sensor_names)
        detected_values = [value for value in values if value > 0.0]

        if len(detected_values) == 0:
            return 0.0

        return sum(detected_values) / len(detected_values)

    def is_direction_blocked(self, direction: RelativeDirection) -> bool:
        proximity = self._average_proximity(self.direction_sonars[direction])
        return proximity >= self.blocked_threshold

    def is_direction_too_close(self, direction: RelativeDirection) -> bool:
        proximity = self._average_proximity(self.direction_sonars[direction])
        return proximity >= self.too_close_threshold

    def is_direction_free(self, direction: RelativeDirection) -> bool:
        return not self.is_direction_blocked(direction)

    def scan_neighbors(self) -> dict[RelativeDirection, bool]:
        """Return whether each adjacent grid direction is clear enough to enter."""
        neighbors: dict[RelativeDirection, bool] = {}

        for direction in RelativeDirection:
            neighbors[direction] = self.is_direction_free(direction)

        return neighbors

    def _side_values(self) -> dict[str, float]:
        return {
            "left_front": self._average_proximity(self.side_sonars["left_front"]),
            "left_rear": self._average_proximity(self.side_sonars["left_rear"]),
            "right_front": self._average_proximity(self.side_sonars["right_front"]),
            "right_rear": self._average_proximity(self.side_sonars["right_rear"]),
        }

    def _left_wall_reliable(self, side_values: dict[str, float]) -> bool:
        return (
            side_values["left_front"] >= self.side_wall_threshold
            and side_values["left_rear"] >= self.side_wall_threshold
        )

    def _right_wall_reliable(self, side_values: dict[str, float]) -> bool:
        return (
            side_values["right_front"] >= self.side_wall_threshold
            and side_values["right_rear"] >= self.side_wall_threshold
        )

    def parallel_error(self) -> float | None:
        """
        Estimate whether the robot is angled relative to nearby side walls.

        Positive means the nose is closer to the left wall or farther from the
        right wall, so navigation should steer right. Returns None when the side
        wall readings are not reliable enough to use.
        """
        side_values = self._side_values()

        left_reliable = self._left_wall_reliable(side_values)
        right_reliable = self._right_wall_reliable(side_values)

        errors: list[float] = []

        if left_reliable:
            left_error = side_values["left_front"] - side_values["left_rear"]
            errors.append(left_error)

        if right_reliable:
            right_error = -(side_values["right_front"] - side_values["right_rear"])
            errors.append(right_error)

        self._debug_print(
            "[Sensors.parallel] "
            f"LF={side_values['left_front']:.2f}, "
            f"LR={side_values['left_rear']:.2f}, "
            f"RF={side_values['right_front']:.2f}, "
            f"RR={side_values['right_rear']:.2f}, "
            f"left_ok={left_reliable}, "
            f"right_ok={right_reliable}, "
            f"errors={errors}"
        )

        if len(errors) == 0:
            self.filtered_parallel_error = None
            return None

        if (
            len(errors) == 2
            and errors[0] * errors[1] < 0.0
            and abs(errors[0]) > self.parallel_conflict_threshold
            and abs(errors[1]) > self.parallel_conflict_threshold
        ):
            self._debug_print(
                "[Sensors.parallel] conflict detected, ignoring correction"
            )
            return None

        raw_error = sum(errors) / len(errors)

        if self.filtered_parallel_error is None:
            self.filtered_parallel_error = raw_error
        else:
            self.filtered_parallel_error = (
                self.parallel_smoothing * self.filtered_parallel_error
                + (1.0 - self.parallel_smoothing) * raw_error
            )

        self._debug_print(
            "[Sensors.parallel] "
            f"raw={raw_error:.2f}, "
            f"filtered={self.filtered_parallel_error:.2f}"
        )

        return self.filtered_parallel_error

    def left_right_diff(self) -> float | None:
        """
        Estimate lateral offset between left and right walls.

        Positive means the robot is closer to the left wall than the right wall.
        Both walls must be visible because this value is used for corridor
        centering, not single-wall following.
        """
        side_values = self._side_values()

        left_reliable = self._left_wall_reliable(side_values)
        right_reliable = self._right_wall_reliable(side_values)

        if not left_reliable or not right_reliable:
            self.filtered_left_right_diff = None

            self._debug_print(
                "[Sensors.left_right] unavailable "
                f"left_ok={left_reliable}, right_ok={right_reliable}"
            )

            return None

        left_average = (side_values["left_front"] + side_values["left_rear"]) / 2.0
        right_average = (side_values["right_front"] + side_values["right_rear"]) / 2.0

        raw_diff = left_average - right_average

        if self.filtered_left_right_diff is None:
            self.filtered_left_right_diff = raw_diff
        else:
            self.filtered_left_right_diff = (
                self.left_right_smoothing * self.filtered_left_right_diff
                + (1.0 - self.left_right_smoothing) * raw_diff
            )

        self._debug_print(
            "[Sensors.left_right] "
            f"left={left_average:.2f}, "
            f"right={right_average:.2f}, "
            f"raw={raw_diff:.2f}, "
            f"filtered={self.filtered_left_right_diff:.2f}"
        )

        return self.filtered_left_right_diff

    def front_back_diff(self) -> float | None:
        """
        positive -> closer to front wall, need move backward
        negative -> closer to back wall, need move forward

        Used after robot rotates 90 degrees for center calibration.
        """
        front = self._average_proximity(self.direction_sonars[RelativeDirection.FRONT])
        back = self._average_proximity(self.direction_sonars[RelativeDirection.BACK])

        front_reliable = front >= self.side_wall_threshold
        back_reliable = back >= self.side_wall_threshold

        if not front_reliable or not back_reliable:
            return None

        return front - back

    def _debug_print(self, message: str) -> None:
        if self.debug:
            print(message)
