from typing import Final, cast

from controller import Keyboard, Robot


class OperatorInput:
    """Read simple operator confirmation commands from the Webots keyboard."""

    KEY_MASK: Final[int] = cast(int, Keyboard.KEY)
    CONTINUE_KEYS: Final[set[int]] = {
        ord(" "),
        10,  # Enter / line feed
        13,  # Enter / carriage return
    }

    def __init__(self, robot: Robot) -> None:
        """Enable the Webots keyboard device."""
        self.keyboard = robot.getKeyboard()
        self.keyboard.enable(int(robot.getBasicTimeStep()))

    def continue_requested(self) -> bool:
        """Return True when space or enter has been pressed."""
        while True:
            key = self.keyboard.getKey()
            if not isinstance(key, int):
                return False
            if key == -1:
                return False
            base_key = key & self.KEY_MASK
            if base_key in self.CONTINUE_KEYS:
                return True
