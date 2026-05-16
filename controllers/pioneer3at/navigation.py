from enum import Enum

from gridMap import GridMap
from odometry import Odometry
from sensors import Sensors
from wheels import Wheels


class NavigationCommand(Enum):
    MOVE_FORWARD = 0
    TURN_LEFT = 1
    TURN_RIGHT = 2
    TURN_AROUND = 3
    RECOVER = 4
    ALIGN_PARALLEL = 5
    ALIGN_CENTER_TURN_LEFT = 6
    ALIGN_CENTER_MOVE = 7
    ALIGN_CENTER_TURN_BACK = 8


class Navigation:
    """
    Turns high-level grid commands into wheel motion using odometry and sonar.

    The class owns short-lived movement commands. Explorer decides what to do
    next; Navigation decides how to execute one command and when it has settled.
    """

    PARALLEL_THRESHOLD = 10.0
    ALIGN_PARALLEL_STABLE_STEPS = 30
    ALIGN_PARALLEL_INVALID_LIMIT = 30
    PARALLEL_FORWARD_DEADBAND = 5.0
    PARALLEL_FORWARD_KP = 0.01
    MAX_PARALLEL_FORWARD_CORRECTION = 0.5
    CENTER_THRESHOLD = 5.0
    ALIGN_CENTER_INVALID_LIMIT = 30

    def __init__(
        self,
        wheels: Wheels,
        odometry: Odometry,
        grid_map: GridMap,
        sensors: Sensors,
        center_threshold: float = 30.0,
        debug: bool = False,
    ) -> None:
        self.wheels = wheels
        self.odometry = odometry
        self.grid_map = grid_map
        self.sensors = sensors

        self.center_threshold = center_threshold
        self.debug = debug

        self.active_command: NavigationCommand | None = None

        self._debug("Navigation initialized")

    def is_idle(self) -> bool:
        return self.active_command is None

    def is_recovering(self) -> bool:
        return self.active_command == NavigationCommand.RECOVER

    def send_command(self, command: NavigationCommand) -> bool:
        """Start a command if the navigation state machine is idle."""
        if command == NavigationCommand.RECOVER:
            return self._start_recovery()

        if not self.is_idle():
            self._debug(
                f"Rejected command={command.name}; "
                f"busy with active_command={self.active_command}"
            )
            return False

        self.odometry.start_action()
        self.active_command = command

        self._debug(
            f"Accepted command={command.name}, "
            f"position={self.grid_map.robot_position}, "
            f"direction={self.grid_map.robot_direction.name}"
        )

        return True

    def cancel_command(self) -> None:
        self._debug(f"Cancelled command={self._active_command_name()}")
        self.wheels.stop()
        self.active_command = None

    def update(self) -> None:
        self.odometry.update()

        if self.active_command is None:
            self.wheels.stop()
            return

        if self.active_command == NavigationCommand.RECOVER:
            if self.odometry.recovery_complete():
                self._debug(
                    "Recovery complete; "
                    f"position={self.grid_map.robot_position}, "
                    f"direction={self.grid_map.robot_direction.name}"
                )
                self.wheels.stop()
                self.active_command = None
                return

            self._proceed_recovery()
            return

        if self._command_completed():
            self._finish_command()
            return

        self._proceed_command()

    def _start_recovery(self) -> bool:
        """Interrupt the current command and drive back to its start pose."""
        if self.active_command is None:
            self._debug("Rejected RECOVER command; no active command")
            return False

        if self.active_command == NavigationCommand.RECOVER:
            self._debug("Rejected RECOVER command; already recovering")
            return False

        interrupted_command = self.active_command

        self.wheels.stop()
        self.active_command = NavigationCommand.RECOVER

        self._debug(
            f"Started recovery from interrupted_command={interrupted_command.name}, "
            f"forward_error={self.odometry.forward_error():.3f}, "
            f"turn_error={self.odometry.turn_error():.3f}"
        )

        return True

    def _start_parallel_alignment(self) -> None:
        self.align_parallel_stable_steps = 0
        self.align_parallel_invalid_steps = 0
        self.active_command = NavigationCommand.ALIGN_PARALLEL

        self._debug(
            "Started ALIGN_PARALLEL, "
            f"position={self.grid_map.robot_position}, "
            f"direction={self.grid_map.robot_direction.name}"
        )

    def _start_center_alignment_if_needed(self) -> None:
        """
        After entering a cell, optionally rotate sideways and center in corridor.

        Centering only works when both side walls are visible. Otherwise the robot
        falls back to parallel alignment against any reliable wall.
        """
        diff = self.sensors.left_right_diff()

        if diff is None:
            self._debug("Center alignment unavailable; starting ALIGN_PARALLEL")
            self._start_parallel_alignment()
            return

        if abs(diff) <= self.center_threshold:
            self._debug(f"Center alignment not needed; left_right_diff={diff:.2f}")
            self._start_parallel_alignment()
            return

        self.align_center_invalid_steps = 0
        self.align_center_move_steps = 0

        self.odometry.start_action()
        self.active_command = NavigationCommand.ALIGN_CENTER_TURN_LEFT

        self._debug(
            f"Started ALIGN_CENTER; left_right_diff={diff:.2f}, "
            f"threshold={self.center_threshold:.2f}"
        )

    def _proceed_command(self) -> None:
        if self.active_command == NavigationCommand.MOVE_FORWARD:
            self._proceed_forward()
            return

        if self.active_command == NavigationCommand.TURN_LEFT:
            self.wheels.turn_left()
            return

        if self.active_command == NavigationCommand.TURN_RIGHT:
            self.wheels.turn_right()
            return

        if self.active_command == NavigationCommand.TURN_AROUND:
            self.wheels.turn_right()
            return

        if self.active_command == NavigationCommand.ALIGN_PARALLEL:
            self._proceed_parallel_alignment()
            return

        if self.active_command == NavigationCommand.ALIGN_CENTER_TURN_LEFT:
            self.wheels.turn_left()
            return

        if self.active_command == NavigationCommand.ALIGN_CENTER_MOVE:
            self._proceed_center_move()
            return

        if self.active_command == NavigationCommand.ALIGN_CENTER_TURN_BACK:
            self.wheels.turn_right()
            return

    def _proceed_forward(self) -> None:
        """Move one grid tile while making small wall-following corrections."""
        error = self.sensors.parallel_error()

        if error is None:
            self.wheels.forward()
            return
        self._debug(f"Parallel forward error: {error:.2f}")
        absolute_error = abs(error)
        if absolute_error <= self.PARALLEL_FORWARD_DEADBAND:
            error = 0.0

        if error == 0.0:
            self.wheels.forward()
            return
        absolute_error -= 5.0
        correction = absolute_error * self.PARALLEL_FORWARD_KP
        correction = min(correction, self.MAX_PARALLEL_FORWARD_CORRECTION)
        turn_ratio = 1.0 - correction
        if error > 0.0:
            self.wheels.curve_right(turn_ratio=turn_ratio)
        else:
            self.wheels.curve_left(turn_ratio=turn_ratio)

    def _proceed_recovery(self) -> None:
        """Undo partial movement before the map position is advanced."""
        turn_error = self.odometry.turn_error()
        forward_error = self.odometry.forward_error()

        self._debug(
            f"Proceeding recovery with turn_error={turn_error:.2f}, forward_error={forward_error:.2f}"
        )

        if abs(turn_error) > self.odometry.turn_tolerance:
            if turn_error > 0:
                self.wheels.turn_right()
            else:
                self.wheels.turn_left()
            return

        if abs(forward_error) > self.odometry.forward_tolerance:
            if forward_error > 0:
                self.wheels.backward()
            else:
                self.wheels.forward()
            return

        self.wheels.stop()

    def _proceed_parallel_alignment(self) -> None:
        parallel_error = self.sensors.parallel_error()

        # No reliable wall, so skip calibration.
        if parallel_error is None:
            self.wheels.stop()
            if self.align_parallel_invalid_steps < self.ALIGN_PARALLEL_INVALID_LIMIT:
                self.align_parallel_invalid_steps += 1
                self._debug(
                    f"Parallel error unavailable; "
                    f"invalid_steps={self.align_parallel_invalid_steps}"
                )
                return
            self.active_command = None
            self._debug("No reliable wall for parallel alignment; skipping")
            return

        if abs(parallel_error) <= self.PARALLEL_THRESHOLD:
            self.align_parallel_stable_steps += 1
            self.wheels.stop()

            if self.align_parallel_stable_steps >= self.ALIGN_PARALLEL_STABLE_STEPS:
                self.active_command = None
                self._debug(f"Parallel alignment complete; error={parallel_error:.2f}")
            return

        self.align_parallel_stable_steps = 0

        # Sign convention:
        # positive parallel_error -> nose angled toward left wall -> turn right
        # negative parallel_error -> nose angled toward right wall -> turn left
        if parallel_error > 0.0:
            self.wheels.turn_right()
        else:
            self.wheels.turn_left()

    def _proceed_center_move(self) -> None:
        """
        Move laterally through a temporary 90-degree turn until centered.

        During this phase the map direction is not updated; the matching
        ALIGN_CENTER_TURN_BACK command restores the physical heading before the
        explorer receives another command.
        """
        diff = self.sensors.front_back_diff()

        if diff is None:
            self.wheels.stop()

            if self.align_center_invalid_steps < self.ALIGN_CENTER_INVALID_LIMIT:
                self.align_center_invalid_steps += 1
                self._debug(
                    f"Center front_back_diff unavailable; "
                    f"invalid_steps={self.align_center_invalid_steps}"
                )
                return

            self._debug("Center alignment skipped; front_back_diff unavailable")
            self.odometry.start_action()
            self.active_command = NavigationCommand.ALIGN_CENTER_TURN_BACK
            return

        self.align_center_invalid_steps = 0

        self._debug(f"Center alignment front_back_diff={diff:.2f}")

        if abs(diff) <= self.center_threshold:
            self.wheels.stop()
            self.odometry.start_action()
            self.active_command = NavigationCommand.ALIGN_CENTER_TURN_BACK
            self._debug(f"Center alignment complete; front_back_diff={diff:.2f}")
            return

        self.align_center_move_steps += 1

        if diff > 0.0:
            self.wheels.backward()
        else:
            self.wheels.forward()

    def _command_completed(self) -> bool:
        if self.active_command == NavigationCommand.MOVE_FORWARD:
            return self.odometry.forward_complete()

        if self.active_command == NavigationCommand.TURN_LEFT:
            return self.odometry.turn_90_complete()

        if self.active_command == NavigationCommand.TURN_RIGHT:
            return self.odometry.turn_90_complete()

        if self.active_command == NavigationCommand.TURN_AROUND:
            return self.odometry.turn_180_complete()

        if self.active_command == NavigationCommand.ALIGN_CENTER_TURN_LEFT:
            return self.odometry.turn_90_complete()

        if self.active_command == NavigationCommand.ALIGN_CENTER_TURN_BACK:
            return self.odometry.turn_90_complete()

        return False

    def _finish_command(self) -> None:
        finished_command = self.active_command

        self.wheels.stop()

        self._debug(
            f"Finished command={finished_command.name if finished_command else None}, "
            f"position={self.grid_map.robot_position}, "
            f"direction={self.grid_map.robot_direction.name}"
        )

        if finished_command == NavigationCommand.MOVE_FORWARD:
            self.grid_map.forwarded()
            self.active_command = None
            self._start_center_alignment_if_needed()

        elif finished_command == NavigationCommand.TURN_LEFT:
            self.grid_map.turned_left()
            self._start_parallel_alignment()

        elif finished_command == NavigationCommand.TURN_RIGHT:
            self.grid_map.turned_right()
            self._start_parallel_alignment()

        elif finished_command == NavigationCommand.TURN_AROUND:
            self.grid_map.turned_around()
            self._start_parallel_alignment()

        elif finished_command == NavigationCommand.ALIGN_CENTER_TURN_LEFT:
            self.odometry.start_action()
            self.active_command = NavigationCommand.ALIGN_CENTER_MOVE
            self._debug("Center alignment turn complete; moving to center")

        elif finished_command == NavigationCommand.ALIGN_CENTER_TURN_BACK:
            self._debug("Center alignment turn-back complete; starting ALIGN_PARALLEL")
            self._start_parallel_alignment()

    def _debug(self, message: str) -> None:
        if self.debug:
            print(f"[Navigation] {message}")

    def _active_command_name(self) -> str:
        if self.active_command is None:
            return "None"
        return self.active_command.name
