from dataclasses import dataclass
from enum import Enum
from math import ceil

from config import ExplorerConfig
from debug_logger import DebugLevel, DebugLogger
from grid_map import (
    Cell,
    Direction,
    GridMap,
    Position,
    RelativeDirection,
    left_of,
    move,
    opposite_of,
    right_of,
)
from navigation import Navigation, NavigationCommand, NavigationPhase
from path_planner import PathPlanner
from sensors import Sensors
from vision_perception import VisionPerception


class ExplorerState(Enum):
    """Top-level states for the exploration finite-state machine."""

    SCANNING = 0
    PLANNING_PATH = 1
    FOLLOWING_PATH = 2
    RECOVERY = 3
    FINISHED = 4
    TARGET_FOUND = 5


@dataclass(frozen=True)
class ExplorerSnapshot:
    """Read-only explorer state exported to the display and observers."""

    grid: dict[Position, Cell]
    visited: set[Position]
    robot_position: Position
    robot_direction: Direction
    path: list[Direction]
    state: ExplorerState
    target_position: Position | None
    current_command: NavigationCommand | None
    navigation_phase: NavigationPhase | None


class Explorer:
    """High-level exploration FSM coordinating mapping, planning, and navigation."""

    def __init__(
        self,
        sensors: Sensors,
        grid_map: GridMap,
        navigation: Navigation,
        perception: VisionPerception,
        config: ExplorerConfig = ExplorerConfig(),
        debug: bool = False,
        debug_level: DebugLevel = DebugLevel.NONE,
    ) -> None:
        """Create an explorer around the shared robot subsystems."""
        self.sensors = sensors
        self.grid_map = grid_map
        self.path_planner = PathPlanner(grid_map)
        self.navigation = navigation
        self.perception = perception
        self.config = config
        effective_debug_level = DebugLevel.DEBUG if debug else debug_level
        self.logger = DebugLogger("Explorer", effective_debug_level)

        self.state = ExplorerState.SCANNING

        self.target_position: Position | None = None
        self.path: list[Direction] = []

        # Explorer keeps the public command because Navigation exposes internal
        # phases while commands are in progress.
        self.current_command: NavigationCommand | None = None
        self._scan_goal_directions: list[Direction] | None = None
        self._finished_return_home_at: float | None = None
        self._finished_last_countdown: int | None = None
        self._target_found_continue_at: float | None = None
        self._target_found_last_countdown: int | None = None

        self.logger.debug("__init__", "Explorer initialised")

    def update(
        self,
        current_time: float = 0.0,
        continue_requested: bool = False,
        cancel_requested: bool = False,
    ) -> None:
        """
        Main Explorer FSM update.

        Call this once per simulation loop.
        Pass Webots simulation time so terminal countdowns use the same clock.

        Important:
        Explorer calls navigation.update(), so do not also call
        navigation.update() separately in the main loop.
        """
        self.navigation.update()

        if self.state == ExplorerState.SCANNING:
            self._proceed_scanning()
            return

        if self.state == ExplorerState.PLANNING_PATH:
            self._proceed_planning_path()
            return

        if self.state == ExplorerState.FOLLOWING_PATH:
            self._proceed_following_path()
            return

        if self.state == ExplorerState.RECOVERY:
            self._proceed_recovery()
            return

        if self.state == ExplorerState.FINISHED:
            self._proceed_finished(
                current_time=current_time,
                continue_requested=continue_requested,
                cancel_requested=cancel_requested,
            )
            return

        if self.state == ExplorerState.TARGET_FOUND:
            self._proceed_target_found(current_time, continue_requested)
            return

    def _proceed_finished(
        self,
        current_time: float,
        continue_requested: bool,
        cancel_requested: bool,
    ) -> None:
        """Wait for operator choice, then return home when the timeout expires."""
        if self._finished_return_home_at is None:
            self._start_finished_countdown(current_time)
            return

        if cancel_requested:
            self._return_home_after_finished()
            return

        if continue_requested:
            self._reload_after_finished()
            return

        remaining = max(0, ceil(self._finished_return_home_at - current_time))

        if remaining != self._finished_last_countdown:
            self._finished_last_countdown = remaining
            self.logger.info(
                "_proceed_finished",
                "Exploration finished. Space/Enter reloads frontiers; "
                f"Esc/C returns to {self.config.home_position}. "
                f"Returning home in {remaining}...",
            )

        if remaining == 0:
            self._return_home_after_finished()
            return

    def _reload_after_finished(self) -> None:
        """Reload visited cells as frontiers and resume scanning."""
        self.logger.info(
            "_reload_after_finished",
            "Reloading frontier queue and resuming exploration",
        )
        self.grid_map.reload_frontier()
        self._clear_finished_countdown()
        self.target_position = None
        self.path = []
        self.current_command = None
        self.perception.reset_all()
        self._set_state(ExplorerState.SCANNING)

    def _return_home_after_finished(self) -> None:
        """Plan a path back to the configured home position."""
        if self.grid_map.robot_position == self.config.home_position:
            self.logger.info(
                "_return_home_after_finished",
                f"Already at home position={self.config.home_position}; resuming scan",
            )
            self._clear_finished_countdown()
            self.target_position = None
            self.path = []
            self.current_command = None
            self.perception.reset_all()
            self._set_state(ExplorerState.SCANNING)
            return

        path = self.path_planner.find_path(
            self.grid_map.robot_position,
            self.config.home_position,
        )

        if path is None:
            self.logger.warn(
                "_return_home_after_finished",
                f"Cannot find path from {self.grid_map.robot_position} "
                f"to {self.config.home_position}",
            )
            return

        self.logger.info(
            "_return_home_after_finished",
            f"Returning to home position={self.config.home_position}: "
            f"{self._format_path(path)}",
        )
        self._clear_finished_countdown()
        self.target_position = self.config.home_position
        self.path = path
        self.current_command = None
        self.perception.reset_all()
        self._set_state(ExplorerState.FOLLOWING_PATH)

    def _start_finished_countdown(self, current_time: float) -> None:
        """Start the finished-state timeout before returning home."""
        self._finished_return_home_at = (
            current_time + self.config.finished_return_home_timeout_seconds
        )
        self._finished_last_countdown = None

    def _clear_finished_countdown(self) -> None:
        """Clear finished-state return-home countdown state."""
        self._finished_return_home_at = None
        self._finished_last_countdown = None

    def _proceed_target_found(
        self,
        current_time: float,
        continue_requested: bool,
    ) -> None:
        """Pause at the target, then continue scanning automatically or on input."""
        if self._target_found_continue_at is None:
            self._target_found_continue_at = (
                current_time + self.config.target_found_auto_continue_seconds
            )
            self._target_found_last_countdown = None
            self.logger.info(
                "_proceed_target_found",
                "Target found; press Space/Enter to continue now.",
            )

        if continue_requested:
            self._continue_after_target_found("operator input")
            return

        remaining = max(0, ceil(self._target_found_continue_at - current_time))

        if remaining != self._target_found_last_countdown:
            self._target_found_last_countdown = remaining
            self.logger.info(
                "_proceed_target_found",
                f"Continuing exploration in {remaining}...",
            )

        if remaining == 0:
            self._continue_after_target_found("countdown complete")
            return

    def _continue_after_target_found(self, reason: str) -> None:
        """Reset the target-found pause and resume scanning from the target cell."""
        self.logger.info(
            "_continue_after_target_found",
            f"Continuing exploration after target found: {reason}",
        )
        self._target_found_continue_at = None
        self._target_found_last_countdown = None
        self.target_position = None
        self.path = []
        self.current_command = None
        self.perception.reset_all()
        self._set_state(ExplorerState.SCANNING)

    def snapshot(self) -> ExplorerSnapshot:
        """Return the current explorer state without mutating the FSM."""
        return ExplorerSnapshot(
            grid=dict(self.grid_map.grid),
            visited=set(self.grid_map.visited),
            robot_position=self.grid_map.robot_position,
            robot_direction=self.grid_map.robot_direction,
            path=list(self.path),
            state=self.state,
            target_position=self.target_position,
            current_command=self.current_command,
            navigation_phase=self.navigation.active_phase,
        )

    def _proceed_scanning(self) -> None:
        """
        Scan sonar neighbours once, then rotate through unvisited enterable
        neighbour directions to check for a far visible green goal.
        """
        if not self.navigation.is_idle():
            return

        # None means this scan cycle has not started yet.
        if self._scan_goal_directions is None:
            self._scan_neighbours()
            self._scan_goal_directions = self._scan_get_unvisited_neighbours()
            self.perception.reset_goal_visible()

            self.logger.debug(
                "_proceed_scanning",
                "Goal-visible scan directions: "
                f"{self._format_path(self._scan_goal_directions)}",
            )

        # Empty list means all candidate directions have been checked.
        if len(self._scan_goal_directions) == 0:
            self.logger.debug("_proceed_scanning", "Scan complete; planning path")
            self._scan_goal_directions = None
            self.perception.reset_goal_visible()
            self._set_state(ExplorerState.PLANNING_PATH)
            return

        # Scan order is not semantically important; use the end as a stack so
        # finishing a direction stays O(1).
        scan_direction = self._scan_goal_directions[-1]
        scan_position = move(self.grid_map.robot_position, scan_direction)

        # Rotate until facing the direction being checked.
        if self.grid_map.robot_direction != scan_direction:
            command = self._command_for_direction(scan_direction)
            accepted = self.navigation.send_command(command)

            if not accepted:
                self.logger.debug(
                    "_proceed_scanning",
                    f"Navigation rejected scan turn command={command.name}",
                )
                self._scan_goal_directions.pop()
                self.perception.reset_goal_visible()

            return

        # Now facing this neighbour; check far goal.
        goal_visible_ahead = self.perception.check_goal_visible_ahead()

        if goal_visible_ahead.uncertain:
            self.logger.debug(
                "_proceed_scanning", "Goal-visible scan uncertain; waiting"
            )
            return

        if goal_visible_ahead.detected:
            self.logger.debug(
                "_proceed_scanning",
                f"Far goal visible towards {scan_direction.name}; "
                f"prioritising next cell {scan_position}",
            )

            self.path = [scan_direction]
            self.target_position = scan_position
            self.current_command = None

            self._scan_goal_directions = None
            self.perception.reset_goal_visible()
            self._set_state(ExplorerState.FOLLOWING_PATH)
            return

        # A clear result means this direction is finished for the current scan.
        self.logger.debug(
            "_proceed_scanning",
            f"No far goal visible towards {scan_direction.name}",
        )
        self._scan_goal_directions.pop()
        self.perception.reset_goal_visible()

    def _proceed_planning_path(self) -> None:
        """
        Choose the next frontier and plan a path to it.

        The frontier is not removed until it is reached or proven unreachable;
        this lets temporary navigation failures re-scan before abandoning cells.
        """
        target = self.grid_map.peek_frontier()

        if target is None:
            self.target_position = None
            self.path = []
            self.current_command = None
            self.logger.debug(
                "_proceed_planning_path",
                "No frontier left; exploration finished",
            )
            self._set_state(ExplorerState.FINISHED)
            return

        if target == self.grid_map.robot_position:
            self.logger.debug(
                "_proceed_planning_path",
                f"Discarding current-position frontier: {target}",
            )
            self.grid_map.discard_frontier(target)
            self._set_state(ExplorerState.SCANNING)
            return

        path = self.path_planner.find_path(
            self.grid_map.robot_position,
            target,
        )

        if path is None:
            self.logger.debug(
                "_proceed_planning_path",
                f"Frontier unreachable; discarding target={target}",
            )
            self.grid_map.discard_frontier(target)
            self.target_position = None
            self.path = []
            self._set_state(ExplorerState.PLANNING_PATH)
            return

        self.target_position = target
        self.path = path
        self.current_command = None

        self.logger.debug(
            "_proceed_planning_path",
            f"Planned path from {self.grid_map.robot_position} "
            f"to {self.target_position}: {self._format_path(self.path)}",
        )

        self._set_state(ExplorerState.FOLLOWING_PATH)

    def _proceed_following_path(self) -> None:
        """
        Follow the planned path one NavigationCommand at a time.
        """
        if not self.navigation.is_idle():
            self._follow_path_check_safety()
            return

        if self.current_command is not None:
            self._follow_path_finish_command()

        if len(self.path) == 0:
            if self.grid_map.get_cell(self.grid_map.robot_position) == Cell.GOAL:
                self.logger.debug(
                    "_follow_path_complete",
                    f"Target found at position={self.grid_map.robot_position}",
                )
                self.target_position = self.grid_map.robot_position
                self.current_command = None
                self.perception.reset_all()
                self._set_state(ExplorerState.TARGET_FOUND)
                return

            self.logger.debug(
                "_follow_path_complete",
                "Path complete; scanning current cell",
            )
            self.target_position = None
            self.current_command = None
            self._set_state(ExplorerState.SCANNING)
            return

        target_direction = self.path[0]
        command = self._command_for_direction(target_direction)
        command = self._follow_path_maybe_override(command)

        if command is None:
            return

        if command == NavigationCommand.MOVE_FORWARD:
            if not self._follow_path_forward_prepare():
                return

        accepted = self.navigation.send_command(command)

        if not accepted:
            self.logger.debug(
                "_follow_path",
                f"Navigation rejected command={command.name}",
            )
            return

        self.current_command = command
        self.perception.reset_all()

        self.logger.debug(
            "_follow_path",
            f"Sent command={command.name}, "
            f"position={self.grid_map.robot_position}, "
            f"direction={self.grid_map.robot_direction.name}",
        )

    def _follow_path_maybe_override(
        self,
        command: NavigationCommand,
    ) -> NavigationCommand | None:
        """
        Prefer a visible far goal directly ahead over a planned turn.

        Clear far-goal checks keep following the planned path. Uncertain checks
        wait so the frame-score filter can confirm or clear the detection.
        """
        if command == NavigationCommand.MOVE_FORWARD:
            return command

        goal_visible_ahead = self.perception.check_goal_visible_ahead()

        if goal_visible_ahead.uncertain:
            self.logger.debug(
                "_follow_path_maybe_override",
                "Goal-visible override uncertain; waiting before planned turn",
            )
            return None

        if not goal_visible_ahead.detected:
            return command

        forward_position = move(
            self.grid_map.robot_position,
            self.grid_map.robot_direction,
        )

        self.logger.debug(
            "_follow_path_maybe_override",
            f"Goal visible ahead; overriding planned {command.name} with MOVE_FORWARD",
        )

        self.path = [self.grid_map.robot_direction]
        self.target_position = forward_position
        return NavigationCommand.MOVE_FORWARD

    def _follow_path_forward_prepare(self) -> bool:
        """
        Check map and vision constraints before sending MOVE_FORWARD.

        Returns True when MOVE_FORWARD can be sent this tick.
        """
        next_position = move(
            self.grid_map.robot_position,
            self.grid_map.robot_direction,
        )

        # Run both checks together so their frame scores build in parallel.
        danger_ahead = self.perception.check_danger_ahead()
        goal_ahead = self.perception.check_goal_ahead()

        # Danger has priority over goal.
        if danger_ahead.detected:
            self.logger.debug(
                "_follow_path_forward_prepare",
                f"Vision detected danger ahead at {next_position}; "
                "marking DANGER and replanning",
            )

            self.grid_map.set_cell(next_position, Cell.DANGER)
            self.grid_map.discard_frontier(next_position)

            self.path = []
            self.current_command = None
            self.perception.reset_all()
            self._set_state(ExplorerState.PLANNING_PATH)
            return False

        if danger_ahead.uncertain:
            self.logger.debug(
                "_follow_path_forward_prepare",
                "Vision danger check uncertain; waiting before MOVE_FORWARD",
            )
            return False

        if goal_ahead.uncertain:
            self.logger.debug(
                "_follow_path_forward_prepare",
                "Vision goal check uncertain; waiting before MOVE_FORWARD",
            )
            return False

        if goal_ahead.detected:
            self.logger.debug(
                "_follow_path_forward_prepare",
                f"Vision detected goal ahead at {next_position}; marking GOAL",
            )

            self.grid_map.set_cell(next_position, Cell.GOAL)

            # Make the goal cell the immediate target.
            # After MOVE_FORWARD completes, the path becomes empty and Explorer scans.
            self.path = [self.grid_map.robot_direction]
            self.target_position = next_position

        return True

    def _proceed_recovery(self) -> None:
        """
        Wait until Navigation finishes recovery.

        Recovery does not update GridMap position.
        After recovery, scan again from the same grid cell.
        """
        if not self.navigation.is_idle():
            return

        self.logger.debug(
            "_proceed_recovery",
            f"Recovery complete at position={self.grid_map.robot_position}; re-scanning",
        )

        self.target_position = None
        self.path = []
        self.current_command = None

        self._set_state(ExplorerState.SCANNING)

    def _scan_neighbours(self) -> None:
        """
        Scan only when the robot is assumed to be centred in a grid cell.

        This pass updates sonar occupancy. Vision may later promote an
        enterable cell to DANGER or GOAL.
        """
        scan = self.sensors.scan_neighbours()

        neighbour_cells: dict[RelativeDirection, Cell] = {
            direction: Cell.FREE if is_free else Cell.WALL
            for direction, is_free in scan.items()
        }

        self.grid_map.update_neighbours(neighbour_cells)

        self.logger.debug(
            "_scan_neighbours",
            f"Scanned at position={self.grid_map.robot_position}, "
            f"direction={self.grid_map.robot_direction.name}, "
            f"neighbours={self._format_neighbour_cells(neighbour_cells)}",
        )

    def _scan_get_unvisited_neighbours(self) -> list[Direction]:
        """
        Return absolute directions for neighbouring cells that are:
        - not visited
        - enterable according to the current grid map

        These are the directions worth rotating towards for far-goal vision checks.
        """
        check_order = [
            RelativeDirection.FRONT,
            RelativeDirection.RIGHT,
            RelativeDirection.LEFT,
            RelativeDirection.BACK,
        ]

        directions: list[Direction] = []

        for relative_direction in check_order:
            absolute_direction = Direction(
                (self.grid_map.robot_direction.value + relative_direction.value) % 4
            )
            adjacent_position = move(
                self.grid_map.robot_position,
                absolute_direction,
            )

            if adjacent_position in self.grid_map.visited:
                continue

            if not self.grid_map.can_enter(adjacent_position):
                continue

            directions.append(absolute_direction)

        return directions

    def _follow_path_check_safety(self) -> None:
        """
        Interrupt an in-progress forward move if sonar sees a late obstacle.

        This only runs while Navigation is busy, before the map position has
        been advanced for the current tile.
        """
        if self.current_command != NavigationCommand.MOVE_FORWARD:
            return

        if self.navigation.active_phase != NavigationPhase.MOVE_FORWARD:
            return

        # Odometry decides whether the robot is already close enough to finishing
        # the one-tile move. If yes, let the command complete and scan normally.
        if self.navigation.odometry.forward_almost_complete():
            return

        if not self.sensors.is_direction_too_close(RelativeDirection.FRONT):
            return

        self.logger.debug(
            "_follow_path_check_safety",
            "Safety triggered: front is too close during MOVE_FORWARD; starting recovery",
        )

        accepted = self.navigation.send_command(NavigationCommand.RECOVER)

        if not accepted:
            self.logger.debug("_follow_path_check_safety", "Recovery command rejected")
            return

        self.path = []
        self.current_command = NavigationCommand.RECOVER
        self._set_state(ExplorerState.RECOVERY)

    def _follow_path_finish_command(self) -> None:
        """
        Navigation has become idle, so the previous command has completed.

        MOVE_FORWARD consumes one path step.
        Turns do not consume path steps because the robot remains in the same cell.
        """
        finished_command = self.current_command

        if finished_command == NavigationCommand.MOVE_FORWARD:
            if len(self.path) > 0:
                completed_step = self.path.pop(0)
                self.logger.debug(
                    "_follow_path_finish_command",
                    f"Forward complete; consumed path step={completed_step.name}, "
                    f"new_position={self.grid_map.robot_position}",
                )

        elif finished_command in {
            NavigationCommand.TURN_LEFT,
            NavigationCommand.TURN_RIGHT,
            NavigationCommand.TURN_AROUND,
        }:
            self.logger.debug(
                "_follow_path_finish_command",
                f"Turn complete; now facing {self.grid_map.robot_direction.name}",
            )

        self.current_command = None

    def _command_for_direction(self, target_direction: Direction) -> NavigationCommand:
        """
        Convert an absolute grid direction into one NavigationCommand.
        """
        current_direction = self.grid_map.robot_direction

        if target_direction == current_direction:
            return NavigationCommand.MOVE_FORWARD

        if target_direction == left_of(current_direction):
            return NavigationCommand.TURN_LEFT

        if target_direction == right_of(current_direction):
            return NavigationCommand.TURN_RIGHT

        if target_direction == opposite_of(current_direction):
            return NavigationCommand.TURN_AROUND

        raise ValueError(
            f"Cannot convert {current_direction} -> {target_direction} into command"
        )

    def _set_state(self, new_state: ExplorerState) -> None:
        """Store a new explorer state and log real transitions."""
        if self.state == new_state:
            return

        old_state = self.state
        self.state = new_state

        self.logger.debug(
            "_set_state",
            f"State changed: {old_state.name} -> {new_state.name}",
        )

    def _format_path(self, path: list[Direction]) -> str:
        """Return a compact debug string for a direction path."""
        return "[" + ", ".join(direction.name for direction in path) + "]"

    def _format_neighbour_cells(
        self,
        cells: dict[RelativeDirection, Cell],
    ) -> str:
        """Return a compact debug string for relative-neighbour cells."""
        return (
            "{ "
            + ", ".join(
                f"{direction.name}: {cell.name}" for direction, cell in cells.items()
            )
            + " }"
        )
