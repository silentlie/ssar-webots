from typing import cast

from controller import Motor, Robot
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
    ) -> None:
        self.default_speed = clamp(default_speed, 0.0, self.MAX_SPEED)
        self.default_turn_speed = clamp(default_turn_speed, 0.0, self.MAX_SPEED)
        self.curve_ratio = clamp(curve_ratio, 0.0, 1.0)

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

    def clamp_speed(self, speed: float) -> float:
        return clamp(speed, -self.MAX_SPEED, self.MAX_SPEED)

    def clamp_turn_ratio(self, turn_ratio: float) -> float:
        return clamp(turn_ratio, 0.0, 1.0)

    def set_speed(self, left_speed: float, right_speed: float) -> None:
        left_speed = self.clamp_speed(left_speed)
        right_speed = self.clamp_speed(right_speed)

        for motor in self.left_motors:
            motor.setVelocity(left_speed)

        for motor in self.right_motors:
            motor.setVelocity(right_speed)

    def stop(self) -> None:
        self.set_speed(0.0, 0.0)

    def forward(self, speed: float | None = None) -> None:
        speed = self.default_speed if speed is None else speed
        speed = self.clamp_speed(speed)
        self.set_speed(speed, speed)

    def backward(self, speed: float | None = None) -> None:
        speed = self.default_speed if speed is None else speed
        speed = self.clamp_speed(speed)
        self.set_speed(-speed, -speed)

    def turn_left(self, speed: float | None = None) -> None:
        speed = self.default_turn_speed if speed is None else speed
        speed = self.clamp_speed(speed)
        self.set_speed(-speed, speed)

    def turn_right(self, speed: float | None = None) -> None:
        speed = self.default_turn_speed if speed is None else speed
        speed = self.clamp_speed(speed)
        self.set_speed(speed, -speed)

    def curve_left(
        self, speed: float | None = None, turn_ratio: float | None = None
    ) -> None:
        """Drive an arc with the left side scaled by turn_ratio."""
        speed = self.default_speed if speed is None else speed
        speed = self.clamp_speed(speed)
        turn_ratio = self.curve_ratio if turn_ratio is None else turn_ratio
        turn_ratio = self.clamp_turn_ratio(turn_ratio)
        self.set_speed(speed * turn_ratio, speed)

    def curve_right(
        self, speed: float | None = None, turn_ratio: float | None = None
    ) -> None:
        """Drive an arc with the right side scaled by turn_ratio."""
        speed = self.default_speed if speed is None else speed
        speed = self.clamp_speed(speed)
        turn_ratio = self.curve_ratio if turn_ratio is None else turn_ratio
        turn_ratio = self.clamp_turn_ratio(turn_ratio)
        self.set_speed(speed, speed * turn_ratio)
