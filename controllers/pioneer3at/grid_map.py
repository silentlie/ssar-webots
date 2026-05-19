from domain import (
    Cell,
    Direction,
    Position,
    RelativeDirection,
    left_of,
    move,
    opposite_of,
    relative_to_absolute,
    right_of,
)
from exploration_queue import ExplorationQueue


class GridMap:
    """Discrete occupancy grid and robot pose used by exploration."""

    UPDATE_ORDER = (
        RelativeDirection.BACK,
        RelativeDirection.LEFT,
        RelativeDirection.RIGHT,
        RelativeDirection.FRONT,
    )

    PROTECTED_CELLS = frozenset(
        {
            Cell.DANGER,
            Cell.GOAL,
        }
    )

    ENTERABLE_CELLS = frozenset(
        {
            Cell.FREE,
            Cell.GOAL,
        }
    )

    def __init__(self) -> None:
        """Create a grid with the robot starting at the origin."""
        self._grid: dict[Position, Cell] = {}
        self._exploration_queue = ExplorationQueue()
        self._robot_position: Position = (0, 0)
        self._robot_direction: Direction = Direction.UP
        self._grid[self._robot_position] = Cell.FREE
        self._mark_visited(self._robot_position)

    @property
    def grid(self) -> dict[Position, Cell]:
        """Return a copy of known grid cells."""
        return dict(self._grid)

    @property
    def visited(self) -> set[Position]:
        """Return a copy of visited positions."""
        return self._exploration_queue.visited

    @property
    def robot_position(self) -> Position:
        """Return the robot's current grid position."""
        return self._robot_position

    @property
    def robot_direction(self) -> Direction:
        """Return the robot's current grid direction."""
        return self._robot_direction

    def get_cell(self, position: Position) -> Cell:
        """Return the cell type at position, defaulting to UNKNOWN."""
        return self._grid.get(position, Cell.UNKNOWN)

    def set_cell(self, position: Position, cell_type: Cell) -> None:
        """Store a cell type and remove blocked cells from the frontier."""
        self._grid[position] = cell_type

        if not self.can_enter(position):
            self.discard_frontier(position)

    def is_visited(self, position: Position) -> bool:
        """Return True when position has already been visited."""
        return position in self._exploration_queue.visited

    def can_enter(self, position: Position) -> bool:
        """Return True when the robot may move into position."""
        return self.get_cell(position) in self.ENTERABLE_CELLS

    def peek_frontier(self) -> Position | None:
        """Return the next frontier cell without removing it."""
        return self._exploration_queue.peek()

    def discard_frontier(self, position: Position) -> None:
        """Remove position from the frontier queue."""
        self._exploration_queue.discard(position)

    def update_neighbours(
        self,
        neighbour_cells: dict[RelativeDirection, Cell],
    ) -> None:
        """Update adjacent cells from a relative-direction sensor scan."""
        for relative_direction in self.UPDATE_ORDER:
            if relative_direction not in neighbour_cells:
                continue
            cell_type = neighbour_cells[relative_direction]
            adjacent_position = self.neighbour_position(relative_direction)
            if self.is_visited(adjacent_position):
                continue
            current_cell = self.get_cell(adjacent_position)
            if current_cell in self.PROTECTED_CELLS:
                continue
            self.set_cell(adjacent_position, cell_type)
            if self.can_enter(adjacent_position):
                self._exploration_queue.add(adjacent_position)

    def turned_left(self) -> None:
        """Update grid pose after a completed left turn."""
        self._robot_direction = left_of(self._robot_direction)

    def turned_right(self) -> None:
        """Update grid pose after a completed right turn."""
        self._robot_direction = right_of(self._robot_direction)

    def turned_around(self) -> None:
        """Update grid pose after a completed 180-degree turn."""
        self._robot_direction = opposite_of(self._robot_direction)

    def forward_position(self) -> Position:
        """Return the grid position directly in front of the robot."""
        return move(self._robot_position, self._robot_direction)

    def forwarded(self) -> None:
        """Advance the robot one grid cell after a completed forward move."""
        next_position = self.forward_position()
        if not self.can_enter(next_position):
            raise ValueError(
                f"Cannot move to {next_position}, "
                f"cell type: {self.get_cell(next_position)}"
            )
        self._robot_position = next_position
        self._mark_visited(next_position)

    def neighbour_position(self, relative_direction: RelativeDirection) -> Position:
        """Return the adjacent position for a robot-relative direction."""
        absolute_direction = relative_to_absolute(
            self._robot_direction,
            relative_direction,
        )
        return move(self._robot_position, absolute_direction)

    def _mark_visited(self, position: Position) -> None:
        """Record position as visited in the exploration queue."""
        self._exploration_queue.visit(position)

    def reload_frontier(self) -> None:
        """Move visited cells back to the frontier for another exploration pass."""
        self._exploration_queue.reload_frontier()
