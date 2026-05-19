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
        self._grid: dict[Position, Cell] = {}
        self._exploration_queue = ExplorationQueue()
        self._robot_position: Position = (0, 0)
        self._robot_direction: Direction = Direction.UP
        self._grid[self._robot_position] = Cell.FREE
        self._mark_visited(self._robot_position)

    @property
    def grid(self) -> dict[Position, Cell]:
        return dict(self._grid)

    @property
    def visited(self) -> set[Position]:
        return self._exploration_queue.visited

    @property
    def robot_position(self) -> Position:
        return self._robot_position

    @property
    def robot_direction(self) -> Direction:
        return self._robot_direction

    def get_cell(self, position: Position) -> Cell:
        return self._grid.get(position, Cell.UNKNOWN)

    def set_cell(self, position: Position, cell_type: Cell) -> None:
        self._grid[position] = cell_type

        if not self.can_enter(position):
            self.discard_frontier(position)

    def is_visited(self, position: Position) -> bool:
        return position in self._exploration_queue.visited

    def can_enter(self, position: Position) -> bool:
        return self.get_cell(position) in self.ENTERABLE_CELLS

    def peek_frontier(self) -> Position | None:
        return self._exploration_queue.peek()

    def discard_frontier(self, position: Position) -> None:
        self._exploration_queue.discard(position)

    def update_neighbours(
        self,
        neighbour_cells: dict[RelativeDirection, Cell],
    ) -> None:
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
        self._robot_direction = left_of(self._robot_direction)

    def turned_right(self) -> None:
        self._robot_direction = right_of(self._robot_direction)

    def turned_around(self) -> None:
        self._robot_direction = opposite_of(self._robot_direction)

    def forward_position(self) -> Position:
        return move(self._robot_position, self._robot_direction)

    def forwarded(self) -> None:
        next_position = self.forward_position()
        if not self.can_enter(next_position):
            raise ValueError(
                f"Cannot move to {next_position}, "
                f"cell type: {self.get_cell(next_position)}"
            )
        self._robot_position = next_position
        self._mark_visited(next_position)

    def neighbour_position(self, relative_direction: RelativeDirection) -> Position:
        absolute_direction = relative_to_absolute(
            self._robot_direction,
            relative_direction,
        )
        return move(self._robot_position, absolute_direction)

    def _mark_visited(self, position: Position) -> None:
        self._exploration_queue.visit(position)

    def reload_frontier(self) -> None:
        self._exploration_queue.reload_frontier()
