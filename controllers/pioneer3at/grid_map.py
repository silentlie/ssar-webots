"""Grid-space data model for exploration and path planning."""

from domain import (
    Cell,
    Direction,
    Position,
    RelativeDirection,
    left_of,
    move,
    opposite_of,
    right_of,
)


class GridMap:
    """
    Stores the explored occupancy grid and the robot's grid-space pose.

    The map is intentionally discrete: odometry handles physical movement, while
    this class advances only after Navigation finishes a full tile or turn.
    """

    def __init__(self) -> None:
        self._grid: dict[Position, Cell] = {}
        self._visited: set[Position] = set()
        # The stack gives recent-first exploration. The set tracks live
        # frontier membership so discarded entries can be skipped lazily.
        self._frontier_stack: list[Position] = []
        self._frontier_set: set[Position] = set()
        self._robot_position: Position = (0, 0)
        self._robot_direction: Direction = Direction.UP
        self._grid[self._robot_position] = Cell.FREE
        self._visited.add(self._robot_position)

    def get_cell(self, position: Position) -> Cell:
        return self._grid.get(position, Cell.UNKNOWN)

    def set_cell(self, position: Position, cell_type: Cell) -> None:
        self._grid[position] = cell_type

    def is_visited(self, position: Position) -> bool:
        return position in self._visited

    @property
    def grid(self) -> dict[Position, Cell]:
        return self._grid

    @property
    def visited(self) -> set[Position]:
        return self._visited

    @property
    def robot_position(self) -> Position:
        return self._robot_position

    @property
    def robot_direction(self) -> Direction:
        return self._robot_direction

    def _add_frontier(self, position: Position) -> None:
        if not self.is_visited(position):
            self._frontier_stack.append(position)
            self._frontier_set.add(position)

    def peek_frontier(self) -> Position | None:
        """Return the newest still-valid frontier without removing it."""
        while self._frontier_stack:
            if self._frontier_stack[-1] in self._frontier_set:
                return self._frontier_stack[-1]
            self._frontier_stack.pop()
        return None

    def discard_frontier(self, position: Position) -> None:
        self._frontier_set.discard(position)

    def update_neighbours(self, sensors: dict[RelativeDirection, Cell]) -> None:
        # Frontiers are stack-backed. Adding in reverse priority makes FRONT the
        # next frontier considered, then RIGHT, then LEFT, then BACK.
        update_order = [
            RelativeDirection.BACK,
            RelativeDirection.LEFT,
            RelativeDirection.RIGHT,
            RelativeDirection.FRONT,
        ]

        for relative_direction in update_order:
            if relative_direction not in sensors:
                continue

            cell_type = sensors[relative_direction]
            absolute_direction = Direction(
                (self._robot_direction.value + relative_direction.value) % 4
            )
            adjacent_position = move(self._robot_position, absolute_direction)

            if adjacent_position in self._visited:
                # Don't update known cells based on noisy sensor data.
                continue
            current_cell = self.get_cell(adjacent_position)
            if current_cell in {Cell.DANGER, Cell.GOAL}:
                # Vision classifications are more specific than sonar
                # occupancy, so later scans must not downgrade them.
                continue
            self.set_cell(adjacent_position, cell_type)

            if self.can_enter(adjacent_position):
                self._add_frontier(adjacent_position)
            else:
                self.discard_frontier(adjacent_position)

    def turned_left(self) -> None:
        self._robot_direction = left_of(self._robot_direction)

    def turned_right(self) -> None:
        self._robot_direction = right_of(self._robot_direction)

    def turned_around(self) -> None:
        self._robot_direction = opposite_of(self._robot_direction)

    def can_enter(self, position: Position) -> bool:
        return self.get_cell(position) in {
            Cell.FREE,
            Cell.GOAL,
        }

    def forwarded(self) -> None:
        next_position = move(self._robot_position, self._robot_direction)
        if not self.can_enter(next_position):
            raise ValueError(
                f"Cannot move to {next_position}, cell type: {self.get_cell(next_position)}"
            )
        self._robot_position = next_position
        self._visited.add(self._robot_position)
        self.discard_frontier(self._robot_position)
