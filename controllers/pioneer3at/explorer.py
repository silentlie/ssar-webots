from dataclasses import dataclass
from enum import Enum

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
    SCANNING = 0
    PLANNING_PATH = 1
    FOLLOWING_PATH = 2
    RECOVERY = 3
    FINISHED = 4


@dataclass(frozen=True)
class ExplorerSnapshot:
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
    def __init__(
        self,
        sensors: Sensors,
        grid_map: GridMap,
        navigation: Navigation,
        perception: VisionPerception,
        debug: bool = False,
        debug_level: DebugLevel = DebugLevel.NONE,
    ) -> None:
        self.sensors = sensors
        self.grid_map = grid_map
        self.path_planner = PathPlanner(grid_map)
        self.navigation = navigation
        self.perception = perception
        effective_debug_level = DebugLevel.DEBUG if debug else debug_level
        self.logger = DebugLogger("Explorer", effective_debug_level)

        self.state = ExplorerState.SCANNING

        self.target_position: Position | None = None
        self.path: list[Direction] = []

        # Explorer keeps the public command because Navigation exposes internal
        # phases while commands are in progress.
        self.current_command: NavigationCommand | None = None
        self._scan_goal_directions: list[Direction] | None = None

        self.logger.debug("__init__", "Explorer initialised")

    def update(self) -> None:
        """
        Main Explorer FSM update.

        Call this once per simulation loop.

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
            return

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
            self._scan_goal_directions = self._scan_goal_visible()
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

        # If this cell is no longer worth checking, finish this direction.
        if scan_position in self.grid_map.visited or not self.grid_map.can_enter(
            scan_position
        ):
            self._scan_goal_directions.pop()
            self.perception.reset_goal_visible()
            return

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

        if goal_visible_ahead.uncertain():
            self.logger.debug(
                "_proceed_scanning", "Goal-visible scan uncertain; waiting"
            )
            return

        if goal_visible_ahead.detected():
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

        # goal_visible_ahead is False, so this direction is finished.
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
            self.logger.debug("_plan_path", "No frontier left; exploration finished")
            self._set_state(ExplorerState.FINISHED)
            return

        if target == self.grid_map.robot_position:
            self.logger.debug(
                "_plan_path",
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
                "_plan_path",
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
            "_plan_path",
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
            self.logger.debug("_follow_path", "Path complete; scanning current cell")
            self.target_position = None
            self.current_command = None
            self._set_state(ExplorerState.SCANNING)
            return

        target_direction = self.path[0]
        command = self._command_for_direction(target_direction)

        # Far goal visibility is only a soft override.
        # If uncertain or false, keep following the planned path.
        if command != NavigationCommand.MOVE_FORWARD:
            goal_visible_ahead = self.perception.check_goal_visible_ahead()

            if goal_visible_ahead.detected():
                forward_position = move(
                    self.grid_map.robot_position,
                    self.grid_map.robot_direction,
                )

                if self.grid_map.can_enter(forward_position):
                    self.logger.debug(
                        "_follow_path",
                        f"Goal visible ahead; overriding planned {command.name} "
                        "with MOVE_FORWARD",
                    )

                    self.path = [self.grid_map.robot_direction]
                    self.target_position = forward_position
                    command = NavigationCommand.MOVE_FORWARD

        if command == NavigationCommand.MOVE_FORWARD:
            next_position = move(
                self.grid_map.robot_position,
                self.grid_map.robot_direction,
            )

            # Explorer decides whether the map allows movement.
            if not self.grid_map.can_enter(next_position):
                self.logger.debug(
                    "_follow_path",
                    f"Map says next cell is not enterable: {next_position}; replanning",
                )
                self.path = []
                self.current_command = None
                self.perception.reset_all()
                self._set_state(ExplorerState.PLANNING_PATH)
                return

            # Run both checks together so their frame scores build in parallel.
            danger_ahead = self.perception.check_danger_ahead()
            goal_ahead = self.perception.check_goal_ahead()

            # Danger has priority over goal.
            if danger_ahead.detected():
                self.logger.debug(
                    "_follow_path",
                    f"Vision detected danger ahead at {next_position}; "
                    "marking DANGER and replanning",
                )

                self.grid_map.set_cell(next_position, Cell.DANGER)
                self.grid_map.discard_frontier(next_position)

                self.path = []
                self.current_command = None
                self.perception.reset_all()
                self._set_state(ExplorerState.PLANNING_PATH)
                return

            if danger_ahead.uncertain():
                self.logger.debug(
                    "_follow_path",
                    "Vision danger check uncertain; waiting before MOVE_FORWARD",
                )
                return

            if goal_ahead.uncertain():
                self.logger.debug(
                    "_follow_path",
                    "Vision goal check uncertain; waiting before MOVE_FORWARD",
                )
                return

            if goal_ahead.detected():
                self.logger.debug(
                    "_follow_path",
                    f"Vision detected goal ahead at {next_position}; marking GOAL",
                )

                self.grid_map.set_cell(next_position, Cell.GOAL)

                # Make the goal cell the immediate target.
                # After MOVE_FORWARD completes, the path becomes empty and Explorer scans.
                self.path = [self.grid_map.robot_direction]
                self.target_position = next_position

            # Both vision checks are decided, so MOVE_FORWARD can be sent.

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

    def _proceed_recovery(self) -> None:
        """
        Wait until Navigation finishes recovery.

        Recovery does not update GridMap position.
        After recovery, scan again from the same grid cell.
        """
        if not self.navigation.is_idle():
            return

        self.logger.debug(
            "_update_recovery",
            f"Recovery complete at position={self.grid_map.robot_position}; re-scanning",
        )

        self.target_position = None
        self.path = []
        self.current_command = None

        self._set_state(ExplorerState.SCANNING)

    def _start_scanning(self) -> None:
        pass

    def _finished_scanning(self) -> None:
        pass

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
            "_scan_neighbours_once",
            f"Scanned at position={self.grid_map.robot_position}, "
            f"direction={self.grid_map.robot_direction.name}, "
            f"neighbours={self._format_neighbour_cells(neighbour_cells)}",
        )

    def _scan_goal_visible(self) -> list[Direction]:
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
            "_check_safety_while_busy",
            "Safety triggered: front is too close during MOVE_FORWARD; starting recovery",
        )

        accepted = self.navigation.send_command(NavigationCommand.RECOVER)

        if not accepted:
            self.logger.debug("_check_safety_while_busy", "Recovery command rejected")
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
                    "_handle_finished_command",
                    f"Forward complete; consumed path step={completed_step.name}, "
                    f"new_position={self.grid_map.robot_position}",
                )

        elif finished_command in {
            NavigationCommand.TURN_LEFT,
            NavigationCommand.TURN_RIGHT,
            NavigationCommand.TURN_AROUND,
        }:
            self.logger.debug(
                "_handle_finished_command",
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
        if self.state == new_state:
            return

        old_state = self.state
        self.state = new_state

        self.logger.debug(
            "_set_state",
            f"State changed: {old_state.name} -> {new_state.name}",
        )

    def _format_path(self, path: list[Direction]) -> str:
        return "[" + ", ".join(direction.name for direction in path) + "]"

    def _format_neighbour_cells(
        self,
        cells: dict[RelativeDirection, Cell],
    ) -> str:
        return (
            "{ "
            + ", ".join(
                f"{direction.name}: {cell.name}" for direction, cell in cells.items()
            )
            + " }"
        )
