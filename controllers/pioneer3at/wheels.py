from typing import cast

from controller import Motor, Robot
from debug_logger import DebugLevel, DebugLogger
from utils import clamp


class Wheels:
    """Motor helper for the Pioneer 3AT four-wheel differential drive."""

    MAX_SPEED = 6.4

    def __init__(
        self,
        robot: Robot,
        default_speed: float = MAX_SPEED,
        default_turn_speed: float = MAX_SPEED,
        curve_ratio: float = 0.15,
        debug_level: DebugLevel = DebugLevel.NONE,
    ) -> None:
        self.logger = DebugLogger("Wheels", debug_level)

        self.default_speed = clamp(default_speed, 0.0, self.MAX_SPEED)
        self.default_turn_speed = clamp(default_turn_speed, 0.0, self.MAX_SPEED)
        self.curve_ratio = clamp(curve_ratio, 0.0, 1.0)

        self.logger.debug(
            "__init__",
            f"default_speed={self.default_speed:.2f}, "
            f"default_turn_speed={self.default_turn_speed:.2f}, "
            f"curve_ratio={self.curve_ratio:.2f}",
        )

        self.front_left = cast(Motor, robot.getDevice("front left wheel"))
        self.front_right = cast(Motor, robot.getDevice("front right wheel"))
        self.back_left = cast(Motor, robot.getDevice("back left wheel"))
        self.back_right = cast(Motor, robot.getDevice("back right wheel"))

        self.left_motors = [
            self.front_left,
            self.back_left,
        ]

        self.right_motors = [
            self.front_right,
            self.back_right,
        ]

        self.all_motors = self.left_motors + self.right_motors

        for motor in self.all_motors:
            # Infinite position switches Webots motors into velocity-control mode.
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

        self.logger.debug("__init__", "Wheel motors initialised")

    def clamp_speed(self, speed: float) -> float:
        clamped = clamp(speed, -self.MAX_SPEED, self.MAX_SPEED)

        if clamped != speed:
            self.logger.debug(
                "clamp_speed",
                f"Clamped wheel speed from {speed:.2f} to {clamped:.2f}",
            )

        return clamped

    def speed_magnitude(self, speed: float, context: str) -> float:
        if speed < 0.0:
            self.logger.warn(
                context,
                f"Negative speed {speed:.2f} was passed; using magnitude instead",
            )

        magnitude = abs(speed)
        clamped = clamp(magnitude, 0.0, self.MAX_SPEED)

        if clamped != magnitude:
            self.logger.debug(
                context,
                f"Clamped speed magnitude from {magnitude:.2f} to {clamped:.2f}",
            )

        return clamped

    def clamp_turn_ratio(self, turn_ratio: float) -> float:
        clamped = clamp(turn_ratio, 0.0, 1.0)

        if clamped != turn_ratio:
            self.logger.debug(
                "clamp_turn_ratio",
                f"Clamped turn_ratio from {turn_ratio:.2f} to {clamped:.2f}",
            )

        return clamped

    def set_speed(self, left_speed: float, right_speed: float) -> None:
        raw_left_speed = left_speed
        raw_right_speed = right_speed

        left_speed = self.clamp_speed(left_speed)
        right_speed = self.clamp_speed(right_speed)

        self.logger.trace(
            "set_speed",
            f"raw_left={raw_left_speed:.2f}, "
            f"raw_right={raw_right_speed:.2f}, "
            f"left={left_speed:.2f}, "
            f"right={right_speed:.2f}",
        )

        for motor in self.left_motors:
            motor.setVelocity(left_speed)

        for motor in self.right_motors:
            motor.setVelocity(right_speed)

    def stop(self) -> None:
        self.logger.trace("stop", "Stopping all wheels")
        self.set_speed(0.0, 0.0)

    def forward(self, speed: float | None = None) -> None:
        requested_speed = speed
        speed = self.default_speed if speed is None else speed
        speed = self.speed_magnitude(speed, "forward")

        self.logger.trace(
            "forward",
            f"requested_speed={requested_speed}, speed={speed:.2f}",
        )

        self.set_speed(speed, speed)

    def backward(self, speed: float | None = None) -> None:
        requested_speed = speed
        speed = self.default_speed if speed is None else speed
        speed = self.speed_magnitude(speed, "backward")

        self.logger.trace(
            "backward",
            f"requested_speed={requested_speed}, speed={speed:.2f}",
        )

        self.set_speed(-speed, -speed)

    def turn_left(self, speed: float | None = None) -> None:
        requested_speed = speed
        speed = self.default_turn_speed if speed is None else speed
        speed = self.speed_magnitude(speed, "turn_left")

        self.logger.trace(
            "turn_left",
            f"requested_speed={requested_speed}, speed={speed:.2f}",
        )

        self.set_speed(-speed, speed)

    def turn_right(self, speed: float | None = None) -> None:
        requested_speed = speed
        speed = self.default_turn_speed if speed is None else speed
        speed = self.speed_magnitude(speed, "turn_right")

        self.logger.trace(
            "turn_right",
            f"requested_speed={requested_speed}, speed={speed:.2f}",
        )

        self.set_speed(speed, -speed)

    def curve_left(
        self,
        speed: float | None = None,
        turn_ratio: float | None = None,
    ) -> None:
        """Drive an arc with the left side scaled by turn_ratio."""
        requested_speed = speed
        requested_turn_ratio = turn_ratio

        speed = self.default_speed if speed is None else speed
        speed = self.speed_magnitude(speed, "curve_left")

        turn_ratio = self.curve_ratio if turn_ratio is None else turn_ratio
        turn_ratio = self.clamp_turn_ratio(turn_ratio)

        left_speed = speed * turn_ratio
        right_speed = speed

        self.logger.trace(
            "curve_left",
            f"requested_speed={requested_speed}, "
            f"requested_turn_ratio={requested_turn_ratio}, "
            f"speed={speed:.2f}, "
            f"turn_ratio={turn_ratio:.2f}, "
            f"left={left_speed:.2f}, "
            f"right={right_speed:.2f}",
        )

        self.set_speed(left_speed, right_speed)

    def curve_right(
        self,
        speed: float | None = None,
        turn_ratio: float | None = None,
    ) -> None:
        """Drive an arc with the right side scaled by turn_ratio."""
        requested_speed = speed
        requested_turn_ratio = turn_ratio

        speed = self.default_speed if speed is None else speed
        speed = self.speed_magnitude(speed, "curve_right")

        turn_ratio = self.curve_ratio if turn_ratio is None else turn_ratio
        turn_ratio = self.clamp_turn_ratio(turn_ratio)

        left_speed = speed
        right_speed = speed * turn_ratio

        self.logger.trace(
            "curve_right",
            f"requested_speed={requested_speed}, "
            f"requested_turn_ratio={requested_turn_ratio}, "
            f"speed={speed:.2f}, "
            f"turn_ratio={turn_ratio:.2f}, "
            f"left={left_speed:.2f}, "
            f"right={right_speed:.2f}",
        )

        self.set_speed(left_speed, right_speed)
