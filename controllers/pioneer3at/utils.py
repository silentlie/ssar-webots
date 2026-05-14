from typing import cast

from controller import Camera, Keyboard, Motor, Robot


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))


def getCamera(robot: Robot) -> Camera:
    timestep = int(robot.getBasicTimeStep())

    camera = cast(Camera, robot.getDevice("camera"))
    camera.enable(timestep)
    return camera


class Wheels:
    def __init__(self, robot: Robot, max_speed: float = 5.0) -> None:
        self.max_speed = max_speed

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
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

    def set_speed(self, left_speed: float, right_speed: float) -> None:
        left_speed = clamp(left_speed, -self.max_speed, self.max_speed)
        right_speed = clamp(right_speed, -self.max_speed, self.max_speed)

        for motor in self.left_motors:
            motor.setVelocity(left_speed)

        for motor in self.right_motors:
            motor.setVelocity(right_speed)

    def stop(self) -> None:
        self.set_speed(0.0, 0.0)

    def forward(self, speed: float | None = None) -> None:
        speed = self.max_speed if speed is None else speed
        self.set_speed(speed, speed)

    def backward(self, speed: float | None = None) -> None:
        speed = self.max_speed if speed is None else speed
        self.set_speed(-speed, -speed)

    def turn_left(self, speed: float | None = None) -> None:
        speed = self.max_speed if speed is None else speed
        self.set_speed(-speed, speed)

    def turn_right(self, speed: float | None = None) -> None:
        speed = self.max_speed if speed is None else speed
        self.set_speed(speed, -speed)

    def curve_left(self, speed: float | None = None, turn_ratio: float = 0.5) -> None:
        speed = self.max_speed if speed is None else speed
        turn_ratio = clamp(turn_ratio, 0.0, 1.0)

        self.set_speed(speed * turn_ratio, speed)

    def curve_right(self, speed: float | None = None, turn_ratio: float = 0.5) -> None:
        speed = self.max_speed if speed is None else speed
        turn_ratio = clamp(turn_ratio, 0.0, 1.0)

        self.set_speed(speed, speed * turn_ratio)


class KeyboardController:
    def __init__(
        self,
        robot: Robot,
        drive_speed: float = 3.0,
        turn_speed: float = 2.0,
    ) -> None:
        self.keyboard = robot.getKeyboard()
        self.keyboard.enable(int(robot.getBasicTimeStep()))
        self.wheels = Wheels(robot, max_speed=max(drive_speed, turn_speed))

        self.drive_speed = drive_speed
        self.turn_speed = turn_speed

        print(
            "KeyboardController initialized with drive_speed =",
            drive_speed,
            "and turn_speed =",
            turn_speed,
        )
        print("Use arrow keys or WASD to control the robot, and space to stop.")

    def get_pressed_keys(self) -> set[int]:
        keys: set[int] = set()

        key = self.keyboard.getKey()
        while key != -1:
            keys.add(key)
            key = self.keyboard.getKey()

        return keys

    def update(self) -> None:
        keys = self.get_pressed_keys()

        forward = Keyboard.UP in keys or ord("W") in keys or ord("w") in keys
        backward = Keyboard.DOWN in keys or ord("S") in keys or ord("s") in keys
        left = Keyboard.LEFT in keys or ord("A") in keys or ord("a") in keys
        right = Keyboard.RIGHT in keys or ord("D") in keys or ord("d") in keys
        space = ord(" ") in keys

        if forward and left:
            self.wheels.set_speed(self.drive_speed * 0.4, self.drive_speed)

        elif forward and right:
            self.wheels.set_speed(self.drive_speed, self.drive_speed * 0.4)

        elif backward and left:
            self.wheels.set_speed(-self.drive_speed * 0.4, -self.drive_speed)

        elif backward and right:
            self.wheels.set_speed(-self.drive_speed, -self.drive_speed * 0.4)

        elif forward:
            self.wheels.forward(self.drive_speed)

        elif backward:
            self.wheels.backward(self.drive_speed)

        elif left:
            self.wheels.turn_left(self.turn_speed)

        elif right:
            self.wheels.turn_right(self.turn_speed)

        elif space:
            self.wheels.stop()

        else:
            self.wheels.stop()
