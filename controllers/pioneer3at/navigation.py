from dataclasses import dataclass
from enum import Enum, auto

from config import NavigationConfig
from debug_logger import DebugLevel, DebugLogger
from grid_map import GridMap
from odometry import Odometry
from sensors import Sensors
from wheels import Wheels


class NavigationCommand(Enum):
    MOVE_FORWARD = auto()
    TURN_LEFT = auto()
    TURN_RIGHT = auto()
    TURN_AROUND = auto()
    RECOVER = auto()


class NavigationPhase(Enum):
    MOVE_FORWARD = auto()
    TURN_LEFT = auto()
    TURN_RIGHT = auto()
    TURN_AROUND = auto()
    RECOVER = auto()
    ALIGN_PARALLEL = auto()
    ALIGN_CENTRE_TURN_LEFT = auto()
    ALIGN_CENTRE_MOVE = auto()
    ALIGN_CENTRE_TURN_BACK = auto()


@dataclass
class AlignmentState:
    """Counters used while wall alignment phases wait for stable readings."""

    parallel_stable_steps: int = 0
    parallel_invalid_steps: int = 0
    centre_invalid_steps: int = 0
    centre_move_steps: int = 0

    def reset_parallel(self) -> None:
        """Reset counters used by parallel wall alignment."""
        self.parallel_stable_steps = 0
        self.parallel_invalid_steps = 0

    def reset_centre(self) -> None:
        """Reset counters used by centre alignment."""
        self.centre_invalid_steps = 0
        self.centre_move_steps = 0


class Navigation:
    """Execute high-level movement commands with odometry and sonar feedback."""

    def __init__(
        self,
        wheels: Wheels,
        odometry: Odometry,
        grid_map: GridMap,
        sensors: Sensors,
        config: NavigationConfig = NavigationConfig(),
        debug_level: DebugLevel = DebugLevel.NONE,
    ) -> None:
        """Create a navigation state machine for one robot."""
        self.wheels = wheels
        self.odometry = odometry
        self.grid_map = grid_map
        self.sensors = sensors
        self.config = config

        self.logger = DebugLogger("Navigation", debug_level)
        self._active_phase: NavigationPhase | None = None
        self.alignment = AlignmentState()
        self.logger.debug("__init__", "Navigation initialised")

    @property
    def active_phase(self) -> NavigationPhase | None:
        """Return the current internal navigation phase, if any."""
        return self._active_phase

    def is_idle(self) -> bool:
        """Return True when navigation can accept a public command."""
        return self._active_phase is None

    def send_command(self, command: NavigationCommand) -> bool:
        """Start a public navigation command when the state machine is idle."""
        if command == NavigationCommand.RECOVER:
            return self._start_recovery()
        if not self.is_idle():
            self.logger.debug(
                "send_command",
                f"Rejected command={command.name}; "
                f"busy with active_phase={self._active_phase.name if self._active_phase else None} ",
            )
            return False
        self.odometry.start_action()
        self._set_phase(self._phase_from_command(command), "send_command")
        self.logger.debug(
            "send_command",
            f"Accepted command={command.name}, "
            f"position={self.grid_map.robot_position}, "
            f"direction={self.grid_map.robot_direction.name}",
        )
        return True

    def update(self) -> None:
        """Advance odometry and progress the active navigation phase."""
        self.odometry.update()
        if self._active_phase is None:
            self.logger.trace("update", "Idle; stopping wheels")
            self.wheels.stop()
            return
        if self._active_phase_complete():
            self._finish_active_phase()
            return
        self._proceed_active_phase()

    def _start_recovery(self) -> bool:
        """Interrupt the current phase and return to the action start pose."""
        if self._active_phase is None:
            self.logger.debug(
                "_start_recovery",
                "Rejected RECOVER command; no interruptible public command",
            )
            return False
        if self._active_phase == NavigationPhase.RECOVER:
            self.logger.debug(
                "_start_recovery",
                "Rejected RECOVER command; already recovering",
            )
            return False
        interrupted_phase = self._active_phase
        self.wheels.stop()
        self._set_phase(NavigationPhase.RECOVER, "_start_recovery")
        self.logger.debug(
            "_start_recovery",
            f"Started recovery from interrupted_phase={interrupted_phase.name}, "
            f"forward_error={self.odometry.forward_error():.3f}, "
            f"turn_error={self.odometry.turn_error():.3f}",
        )
        return True

    def _start_parallel_alignment(self) -> None:
        """Start the wall-parallel alignment phase."""
        self.wheels.stop()
        self.alignment.reset_parallel()
        self._set_phase(NavigationPhase.ALIGN_PARALLEL, "_start_parallel_alignment")
        self.logger.debug(
            "_start_parallel_alignment",
            "Started ALIGN_PARALLEL, "
            f"position={self.grid_map.robot_position}, "
            f"direction={self.grid_map.robot_direction.name}",
        )

    def _start_centre_alignment_if_needed(self) -> None:
        """Start centre alignment when both corridor side walls are visible."""
        diff = self.sensors.left_right_diff()
        if diff is None:
            self.logger.debug(
                "_start_centre_alignment_if_needed",
                "Centre alignment unavailable; starting ALIGN_PARALLEL",
            )
            self._start_parallel_alignment()
            return
        if abs(diff) <= self.config.side_centre_threshold:
            self.logger.debug(
                "_start_centre_alignment_if_needed",
                f"Centre alignment not needed; left_right_diff={diff:.2f}",
            )
            self._start_parallel_alignment()
            return
        self.alignment.reset_centre()
        self.odometry.start_action()
        self._set_phase(
            NavigationPhase.ALIGN_CENTRE_TURN_LEFT, "_start_centre_alignment_if_needed"
        )
        self.logger.debug(
            "_start_centre_alignment_if_needed",
            f"Started ALIGN_CENTRE; left_right_diff={diff:.2f}, "
            f"threshold={self.config.side_centre_threshold:.2f}",
        )

    def _proceed_active_phase(self) -> None:
        """Dispatch wheel control for the current internal phase."""
        match self._active_phase:
            case NavigationPhase.MOVE_FORWARD:
                self._proceed_forward()
            case NavigationPhase.TURN_LEFT:
                self.wheels.turn_left()
            case NavigationPhase.TURN_RIGHT:
                self.wheels.turn_right()
            case NavigationPhase.TURN_AROUND:
                self.wheels.turn_right()
            case NavigationPhase.ALIGN_PARALLEL:
                self._proceed_align_parallel()
            case NavigationPhase.ALIGN_CENTRE_TURN_LEFT:
                self.wheels.turn_left()
            case NavigationPhase.ALIGN_CENTRE_MOVE:
                self._proceed_align_centre()
            case NavigationPhase.ALIGN_CENTRE_TURN_BACK:
                self.wheels.turn_right()
            case NavigationPhase.RECOVER:
                self._proceed_recovery()
            case _:
                self.wheels.stop()

    def _proceed_forward(self) -> None:
        """Drive forward while applying small wall-parallel corrections."""
        error = self.sensors.parallel_error()
        if error is None:
            self.logger.trace(
                "_proceed_forward",
                "Parallel error unavailable; moving forward without correction",
            )
            self.wheels.forward()
            return
        self.logger.trace("_proceed_forward", f"Parallel forward error={error:.2f}")
        if abs(error) <= self.config.parallel_forward_deadband:
            self.logger.trace(
                "_proceed_forward",
                f"Parallel error within deadband; moving straight, error={error:.2f}",
            )
            self.wheels.forward()
            return
        turn_ratio = self._parallel_turn_ratio(error)
        self.logger.trace(
            "_proceed_forward",
            f"Applying parallel correction; error={error:.2f}, "
            f"turn_ratio={turn_ratio:.2f}",
        )
        if error > 0.0:
            self.wheels.curve_right(turn_ratio=turn_ratio)
        else:
            self.wheels.curve_left(turn_ratio=turn_ratio)

    def _parallel_turn_ratio(self, error: float) -> float:
        """Convert a parallel wall error into a wheel speed ratio."""
        correction = abs(error) * self.config.parallel_forward_kp
        correction = min(correction, self.config.max_parallel_forward_correction)
        return 1.0 - correction

    def _proceed_recovery(self) -> None:
        """Undo partial movement before the map position is advanced."""
        turn_error = self.odometry.turn_error()
        forward_error = self.odometry.forward_error()

        self.logger.trace(
            "_proceed_recovery",
            f"turn_error={turn_error:.2f}, forward_error={forward_error:.2f}",
        )

        if abs(turn_error) > self.odometry.turn_tolerance:
            if turn_error > 0.0:
                self.logger.trace(
                    "_proceed_recovery", "Correcting turn error: turn_right"
                )
                self.wheels.turn_right()
            else:
                self.logger.trace(
                    "_proceed_recovery", "Correcting turn error: turn_left"
                )
                self.wheels.turn_left()
            return

        if abs(forward_error) > self.odometry.forward_tolerance:
            if forward_error > 0.0:
                self.logger.trace(
                    "_proceed_recovery", "Correcting forward error: backward"
                )
                self.wheels.backward()
            else:
                self.logger.trace(
                    "_proceed_recovery", "Correcting forward error: forward"
                )
                self.wheels.forward()
            return

        self.wheels.stop()

    def _proceed_align_parallel(self) -> None:
        """Rotate until side-wall readings are parallel or unavailable."""
        parallel_error = self.sensors.parallel_error()
        if parallel_error is None:
            self.wheels.stop()
            if (
                self.alignment.parallel_invalid_steps
                < self.config.align_parallel_invalid_limit
            ):
                self.alignment.parallel_invalid_steps += 1
                self.logger.trace(
                    "_proceed_align_parallel",
                    "Parallel error unavailable; "
                    f"invalid_steps={self.alignment.parallel_invalid_steps}",
                )
                return
            self._set_phase(None, "_proceed_align_parallel")
            self.logger.debug(
                "_proceed_align_parallel",
                "No reliable wall for parallel alignment; skipping",
            )
            return
        if abs(parallel_error) <= self.config.parallel_threshold:
            self.alignment.parallel_stable_steps += 1
            self.wheels.stop()
            if (
                self.alignment.parallel_stable_steps
                >= self.config.align_parallel_stable_steps
            ):
                self._set_phase(None, "_proceed_align_parallel")

                self.logger.debug(
                    "_proceed_align_parallel",
                    f"Parallel alignment complete; error={parallel_error:.2f}",
                )
            return
        self.alignment.parallel_stable_steps = 0
        if parallel_error > 0.0:
            self.logger.trace(
                "_proceed_align_parallel",
                f"Turning right to reduce parallel_error={parallel_error:.2f}",
            )
            self.wheels.turn_right()
        else:
            self.logger.trace(
                "_proceed_align_parallel",
                f"Turning left to reduce parallel_error={parallel_error:.2f}",
            )
            self.wheels.turn_left()

    def _proceed_align_centre(self) -> None:
        """Move sideways after a 90-degree turn until the corridor is centred."""
        diff = self.sensors.front_back_diff()
        if diff is None:
            self.wheels.stop()
            if (
                self.alignment.centre_invalid_steps
                < self.config.align_centre_invalid_limit
            ):
                self.alignment.centre_invalid_steps += 1
                self.logger.trace(
                    "_proceed_align_centre",
                    "Centre front_back_diff unavailable; "
                    f"invalid_steps={self.alignment.centre_invalid_steps}",
                )
                return
            self.logger.debug(
                "_proceed_align_centre",
                "Centre alignment skipped; front_back_diff unavailable",
            )
            self.odometry.start_action()
            self._set_phase(
                NavigationPhase.ALIGN_CENTRE_TURN_BACK, "_proceed_align_centre"
            )
            return
        self.alignment.centre_invalid_steps = 0
        self.logger.trace(
            "_proceed_align_centre",
            f"Centre alignment front_back_diff={diff:.2f}",
        )
        if abs(diff) <= self.config.centre_move_threshold:
            self.wheels.stop()
            self.odometry.start_action()
            self._set_phase(
                NavigationPhase.ALIGN_CENTRE_TURN_BACK, "_proceed_align_centre"
            )

            self.logger.debug(
                "_proceed_align_centre",
                f"Centre alignment complete; front_back_diff={diff:.2f}",
            )
            return
        self.alignment.centre_move_steps += 1
        if diff > 0.0:
            self.logger.trace(
                "_proceed_align_centre",
                f"Moving backward to centre; front_back_diff={diff:.2f}",
            )
            self.wheels.backward()
        else:
            self.logger.trace(
                "_proceed_align_centre",
                f"Moving forward to centre; front_back_diff={diff:.2f}",
            )
            self.wheels.forward()

    def _active_phase_complete(self) -> bool:
        """Return True when odometry says the active phase is complete."""
        match self._active_phase:
            case NavigationPhase.MOVE_FORWARD:
                return self.odometry.forward_complete()

            case NavigationPhase.TURN_LEFT:
                return self.odometry.turn_90_complete()

            case NavigationPhase.TURN_RIGHT:
                return self.odometry.turn_90_complete()

            case NavigationPhase.TURN_AROUND:
                return self.odometry.turn_180_complete()

            case NavigationPhase.ALIGN_CENTRE_TURN_LEFT:
                return self.odometry.turn_90_complete()

            case NavigationPhase.ALIGN_CENTRE_TURN_BACK:
                return self.odometry.turn_90_complete()

            case NavigationPhase.RECOVER:
                return self.odometry.recovery_complete()

            case _:
                return False

    def _finish_active_phase(self) -> None:
        """Apply grid-map effects and transition after a completed phase."""
        finished_phase = self._active_phase
        self.wheels.stop()
        self.logger.debug(
            "_finish_active_phase",
            f"Finished phase={finished_phase.name if finished_phase else None}, "
            f"position={self.grid_map.robot_position}, "
            f"direction={self.grid_map.robot_direction.name}",
        )
        self._set_phase(None, "_finish_active_phase")
        match finished_phase:
            case NavigationPhase.MOVE_FORWARD:
                self.grid_map.forwarded()
                self._start_centre_alignment_if_needed()

            case NavigationPhase.TURN_LEFT:
                self.grid_map.turned_left()
                self._start_parallel_alignment()

            case NavigationPhase.TURN_RIGHT:
                self.grid_map.turned_right()
                self._start_parallel_alignment()

            case NavigationPhase.TURN_AROUND:
                self.grid_map.turned_around()
                self._start_parallel_alignment()

            case NavigationPhase.RECOVER:
                self._start_parallel_alignment()

            case NavigationPhase.ALIGN_CENTRE_TURN_LEFT:
                self.odometry.start_action()
                self._set_phase(
                    NavigationPhase.ALIGN_CENTRE_MOVE, "_finish_active_phase"
                )
                self.logger.debug(
                    "_finish_active_phase",
                    "Centre alignment turn complete; moving to centre",
                )

            case NavigationPhase.ALIGN_CENTRE_MOVE:
                pass

            case NavigationPhase.ALIGN_CENTRE_TURN_BACK:
                self.logger.debug(
                    "_finish_active_phase",
                    "Centre alignment turn-back complete; starting ALIGN_PARALLEL",
                )
                self._start_parallel_alignment()
            case _:
                raise ValueError(f"Unsupported navigation phase: {finished_phase}")

    def _phase_from_command(self, command: NavigationCommand) -> NavigationPhase:
        """Map a public command onto its initial internal phase."""
        match command:
            case NavigationCommand.MOVE_FORWARD:
                return NavigationPhase.MOVE_FORWARD
            case NavigationCommand.TURN_LEFT:
                return NavigationPhase.TURN_LEFT
            case NavigationCommand.TURN_RIGHT:
                return NavigationPhase.TURN_RIGHT
            case NavigationCommand.TURN_AROUND:
                return NavigationPhase.TURN_AROUND
            case NavigationCommand.RECOVER:
                return NavigationPhase.RECOVER
            case _:
                raise ValueError(f"Unsupported navigation command: {command}")

    def _set_phase(self, phase: NavigationPhase | None, context: str) -> None:
        """Store the active phase and log meaningful transitions."""
        if self._active_phase == phase:
            return

        old_phase = self._active_phase
        self._active_phase = phase

        old_name = old_phase.name if old_phase is not None else "None"
        new_name = phase.name if phase is not None else "None"

        self.logger.debug(
            context,
            f"Phase changed: {old_name} -> {new_name}",
        )
