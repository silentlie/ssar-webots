from enum import Enum

from displayController import DisplayController
from gridMap import (
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
from navigation import Navigation, NavigationCommand
from pathPlanner import PathPlanner
from sensors import Sensors


class ExplorerState(Enum):
    SCANNING = 0
    PLANNING_PATH = 1
    FOLLOWING_PATH = 2
    RECOVERY = 3
    FINISHED = 4


class Explorer:
    """
    High-level exploration state machine.

    Explorer updates the map, chooses frontier targets, asks PathPlanner for a
    route, and feeds one NavigationCommand at a time to Navigation.
    """

    def __init__(
        self,
        sensors: Sensors,
        grid_map: GridMap,
        navigation: Navigation,
        display: DisplayController,
        debug: bool = False,
    ) -> None:
        self.sensors = sensors
        self.grid_map = grid_map
        self.path_planner = PathPlanner(grid_map)
        self.display = display
        self.navigation = navigation
        self.debug = debug

        self.state = ExplorerState.SCANNING

        self.target_position: Position | None = None
        self.path: list[Direction] = []

        # Explorer keeps this because Navigation clears active_command
        # when the command finishes.
        self.current_command: NavigationCommand | None = None

        self._debug("Explorer initialized")

    def update(self) -> None:
        """
        Main Explorer FSM update.

        Call this once per simulation loop.

        Important:
        Explorer calls navigation.update(), so do not also call
        navigation.update() separately in the main loop.
        """
        self.navigation.update()

        self.display.update(
            grid=self.grid_map.grid,
            visited=self.grid_map.visited,
            robot_position=self.grid_map.robot_position,
            robot_direction=self.grid_map.robot_direction,
            path=self.path,
        )

        if self.state == ExplorerState.SCANNING:
            self._scan()
            self._set_state(ExplorerState.PLANNING_PATH)
            return

        if self.state == ExplorerState.PLANNING_PATH:
            self._plan_path()
            return

        if self.state == ExplorerState.FOLLOWING_PATH:
            self._follow_path()
            return

        if self.state == ExplorerState.RECOVERY:
            self._update_recovery()
            return

        if self.state == ExplorerState.FINISHED:
            return

    def _scan(self) -> None:
        """
        Scan only when the robot is assumed to be centred in a grid cell.
        """
        scan = self.sensors.scan_neighbors()

        neighbor_cells: dict[RelativeDirection, Cell] = {
            direction: Cell.FREE if is_free else Cell.WALL
            for direction, is_free in scan.items()
        }

        self.grid_map.update_neighbors(neighbor_cells)

        self._debug(
            f"Scanned at position={self.grid_map.robot_position}, "
            f"direction={self.grid_map.robot_direction.name}, "
            f"neighbors={self._format_neighbor_cells(neighbor_cells)}"
        )

    def _plan_path(self) -> None:
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
            self._debug("No frontier left; exploration finished")
            self._set_state(ExplorerState.FINISHED)
            return

        if target == self.grid_map.robot_position:
            self._debug(f"Discarding current-position frontier: {target}")
            self.grid_map.discard_frontier(target)
            self._set_state(ExplorerState.SCANNING)
            return

        path = self.path_planner.find_path(
            self.grid_map.robot_position,
            target,
        )

        if path is None:
            self._debug(f"Frontier unreachable; discarding target={target}")
            self.grid_map.discard_frontier(target)
            self.target_position = None
            self.path = []
            self._set_state(ExplorerState.PLANNING_PATH)
            return

        self.target_position = target
        self.path = path
        self.current_command = None

        self._debug(
            f"Planned path from {self.grid_map.robot_position} "
            f"to {self.target_position}: {self._format_path(self.path)}"
        )

        self._set_state(ExplorerState.FOLLOWING_PATH)

    def _follow_path(self) -> None:
        """
        Follow the planned path one NavigationCommand at a time.
        """
        if not self.navigation.is_idle():
            self._check_safety_while_busy()
            return

        if self.current_command is not None:
            self._handle_finished_command()

        if len(self.path) == 0:
            self._debug("Path complete; scanning current cell")
            self.target_position = None
            self.current_command = None
            self._set_state(ExplorerState.SCANNING)
            return

        target_direction = self.path[0]
        command = self._command_for_direction(target_direction)

        if command == NavigationCommand.MOVE_FORWARD:
            next_position = move(
                self.grid_map.robot_position,
                self.grid_map.robot_direction,
            )

            # Explorer decides whether the map allows movement.
            if not self.grid_map.can_enter(next_position):
                self._debug(
                    f"Map says next cell is not enterable: {next_position}; replanning"
                )
                self.path = []
                self.current_command = None
                self._set_state(ExplorerState.PLANNING_PATH)
                return

        accepted = self.navigation.send_command(command)

        if not accepted:
            self._debug(f"Navigation rejected command={command.name}")
            return

        self.current_command = command

        self._debug(
            f"Sent command={command.name}, "
            f"position={self.grid_map.robot_position}, "
            f"direction={self.grid_map.robot_direction.name}"
        )

    def _check_safety_while_busy(self) -> None:
        """
        Safety check while Navigation is busy.

        This only triggers during MOVE_FORWARD and uses is_direction_too_close(),
        not is_direction_blocked().
        """
        if self.current_command != NavigationCommand.MOVE_FORWARD:
            return

        if self.navigation.active_command != NavigationCommand.MOVE_FORWARD:
            return

        # Odometry decides whether the robot is already close enough to finishing
        # the one-tile move. If yes, let the command complete and scan normally.
        if self.navigation.odometry.forward_almost_complete():
            return

        if not self.sensors.is_direction_too_close(RelativeDirection.FRONT):
            return

        self._debug(
            "Safety triggered: front is too close during MOVE_FORWARD; starting recovery"
        )

        accepted = self.navigation.send_command(NavigationCommand.RECOVER)

        if not accepted:
            self._debug("Recovery command rejected")
            return

        self.path = []
        self.current_command = NavigationCommand.RECOVER
        self._set_state(ExplorerState.RECOVERY)

    def _handle_finished_command(self) -> None:
        """
        Navigation has become idle, so the previous command has completed.

        MOVE_FORWARD consumes one path step.
        Turns do not consume path steps because the robot remains in the same cell.
        """
        finished_command = self.current_command

        if finished_command == NavigationCommand.MOVE_FORWARD:
            if len(self.path) > 0:
                completed_step = self.path.pop(0)
                self._debug(
                    f"Forward complete; consumed path step={completed_step.name}, "
                    f"new_position={self.grid_map.robot_position}"
                )

        elif finished_command in {
            NavigationCommand.TURN_LEFT,
            NavigationCommand.TURN_RIGHT,
            NavigationCommand.TURN_AROUND,
        }:
            self._debug(
                f"Turn complete; now facing {self.grid_map.robot_direction.name}"
            )

        self.current_command = None

    def _update_recovery(self) -> None:
        """
        Wait until Navigation finishes recovery.

        Recovery does not update GridMap position.
        After recovery, scan again from the same grid cell.
        """
        if not self.navigation.is_idle():
            return

        self._debug(
            f"Recovery complete at position={self.grid_map.robot_position}; re-scanning"
        )

        self.target_position = None
        self.path = []
        self.current_command = None

        self._set_state(ExplorerState.SCANNING)

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

        self._debug(f"State changed: {old_state.name} -> {new_state.name}")

    def _debug(self, message: str) -> None:
        if self.debug:
            print(f"[Explorer] {message}")

    def _format_path(self, path: list[Direction]) -> str:
        return "[" + ", ".join(direction.name for direction in path) + "]"

    def _format_neighbor_cells(
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
