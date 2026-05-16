from controller import Keyboard, Robot
from wheels import Wheels


class KeyboardController:
    def __init__(
        self,
        robot: Robot,
        drive_speed: float = 6.4,
        turn_speed: float = 6.4,
        curve_ratio: float = 0.5,
    ) -> None:
        self.keyboard = robot.getKeyboard()
        self.keyboard.enable(int(robot.getBasicTimeStep()))
        self.wheels = Wheels(
            robot, default_speed=drive_speed, default_turn_speed=turn_speed, curve_ratio=curve_ratio
        )

        self.drive_speed = drive_speed
        self.turn_speed = turn_speed
        self.curve_ratio = curve_ratio

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
            self.wheels.curve_left(self.drive_speed, self.curve_ratio)

        elif forward and right:
            self.wheels.curve_right(self.drive_speed, self.curve_ratio)

        elif backward and left:
            self.wheels.curve_left(-self.drive_speed, self.curve_ratio)

        elif backward and right:
            self.wheels.curve_right(-self.drive_speed, self.curve_ratio)

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
