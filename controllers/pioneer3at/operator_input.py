from dataclasses import dataclass
from typing import Final, cast

from controller import Keyboard, Robot


@dataclass(frozen=True)
class OperatorRequests:
    """Operator commands collected during one keyboard poll."""

    continue_requested: bool = False
    cancel_requested: bool = False


class OperatorInput:
    """Read operator continue/cancel commands from the Webots keyboard."""

    KEY_MASK: Final[int] = cast(int, Keyboard.KEY)
    CONTINUE_KEYS: Final[set[int]] = {
        ord(" "),
        10,  # Enter / line feed
        13,  # Enter / carriage return
    }
    CANCEL_KEYS: Final[set[int]] = {
        27,  # Escape
        ord("C"),
        ord("c"),
    }

    def __init__(self, robot: Robot) -> None:
        """Enable the Webots keyboard device."""
        self.keyboard = robot.getKeyboard()
        self.keyboard.enable(int(robot.getBasicTimeStep()))

    def continue_requested(self) -> bool:
        """Return True when Space or Enter has been pressed."""
        return self.requests().continue_requested

    def requests(self) -> OperatorRequests:
        """Drain queued key presses and return all recognised operator requests."""
        continue_requested = False
        cancel_requested = False
        key = self.keyboard.getKey()

        while key != -1:
            base_key = key & self.KEY_MASK

            if base_key in self.CONTINUE_KEYS:
                continue_requested = True

            if base_key in self.CANCEL_KEYS:
                cancel_requested = True

            key = self.keyboard.getKey()

        return OperatorRequests(
            continue_requested=continue_requested,
            cancel_requested=cancel_requested,
        )
