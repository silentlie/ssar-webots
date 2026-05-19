from dataclasses import dataclass
from typing import cast

from config import DisplayConfig
from controller import Display, Robot
from debug_logger import DebugLevel, DebugLogger
from domain import DIRECTION_DELTAS, Cell, Direction, Position, move


@dataclass
class DisplayState:
    grid: dict[Position, Cell]
    visited: set[Position]
    robot_position: Position
    robot_direction: Direction
    path: list[Direction]
    explorer_state: str
    target_position: Position | None = None
    navigation_command: str | None = None

    @property
    def next_step_name(self) -> str:
        if len(self.path) == 0:
            return "None"

        return self.path[0].name


@dataclass
class ViewTransform:
    cell_size: int = 1
    map_min_x: int = 0
    map_min_y: int = 0
    offset_x: int = 0
    offset_y: int = 0


class DisplayController:
    def __init__(
        self,
        robot: Robot,
        config: DisplayConfig = DisplayConfig(),
        debug_level: DebugLevel = DebugLevel.NONE,
        display_name: str = "display",
    ) -> None:
        self.display = cast(Display, robot.getDevice(display_name))
        self.width = self.display.getWidth()
        self.height = self.display.getHeight()
        self.config = config
        self.view = ViewTransform()
        self.logger = DebugLogger("DisplayController", debug_level)
        self.logger.debug(
            "__init__",
            f"width={self.width}, height={self.height}",
        )

    def update(self, state: DisplayState) -> None:
        path_positions = self._path_to_positions(
            state.robot_position,
            state.path,
        )
        self._update_view_bounds(
            grid=state.grid,
            robot_position=state.robot_position,
            path_positions=path_positions,
        )
        self._clear()
        self._draw_grid(state.grid, state.visited)
        self._draw_path(path_positions)
        self._draw_target(path_positions)
        self._draw_robot(state.robot_position, state.robot_direction)
        self._draw_text(state)

    def _update_view_bounds(
        self,
        grid: dict[Position, Cell],
        robot_position: Position,
        path_positions: list[Position],
    ) -> None:
        positions = set(grid.keys())
        positions.add(robot_position)
        positions.update(path_positions)
        if len(positions) == 0:
            positions.add((0, 0))
        min_x = min(position[0] for position in positions)
        max_x = max(position[0] for position in positions)
        min_y = min(position[1] for position in positions)
        max_y = max(position[1] for position in positions)
        min_x -= 1
        max_x += 1
        min_y -= 1
        max_y += 1
        columns = max_x - min_x + 1
        rows = max_y - min_y + 1
        available_width = max(1, self.width - self.config.padding * 2)
        available_height = max(
            1,
            self.height - self.config.top_margin - self.config.padding,
        )
        cell_from_width = available_width // columns
        cell_from_height = available_height // rows
        self.view.cell_size = max(1, min(cell_from_width, cell_from_height))
        map_width = columns * self.view.cell_size
        map_height = rows * self.view.cell_size
        self.view.map_min_x = min_x
        self.view.map_min_y = min_y
        self.view.offset_x = (self.width - map_width) // 2
        self.view.offset_y = (
            self.config.top_margin + (available_height - map_height) // 2
        )

    def _clear(self) -> None:
        self.display.setColor(self.config.background_colour)
        self.display.fillRectangle(0, 0, self.width, self.height)

    def _draw_grid(
        self,
        grid: dict[Position, Cell],
        visited: set[Position],
    ) -> None:
        for position, cell in grid.items():
            colour = self._colour_for_cell(position, cell, visited)
            self._draw_cell(position, colour, inset=1)

    def _draw_path(self, path_positions: list[Position]) -> None:
        for position in path_positions:
            self._draw_cell(
                position,
                self.config.path_colour,
                inset=max(2, self.view.cell_size // 4),
            )

    def _draw_target(self, path_positions: list[Position]) -> None:
        if len(path_positions) == 0:
            return
        self._draw_cell(
            path_positions[-1],
            self.config.target_colour,
            inset=max(2, self.view.cell_size // 5),
        )

    def _draw_robot(
        self,
        position: Position,
        direction: Direction,
    ) -> None:
        centre_x, centre_y = self._cell_centre(position)
        radius = max(4, self.view.cell_size // 4)
        self.display.setColor(self.config.robot_colour)
        self.display.fillOval(centre_x, centre_y, radius, radius)

        self._draw_robot_direction_triangle(position, direction)

    def _draw_robot_direction_triangle(
        self,
        position: Position,
        direction: Direction,
    ) -> None:
        centre_x, centre_y = self._cell_centre(position)
        arrow_length = max(5, self.view.cell_size // 3)
        arrow_width = max(3, self.view.cell_size // 6)
        dx, dy = DIRECTION_DELTAS[direction]
        px = -dy
        py = dx
        tip_x = centre_x + dx * arrow_length
        tip_y = centre_y + dy * arrow_length
        base_x = centre_x + dx * max(2, arrow_length // 3)
        base_y = centre_y + dy * max(2, arrow_length // 3)
        left_x = base_x + px * arrow_width
        left_y = base_y + py * arrow_width
        right_x = base_x - px * arrow_width
        right_y = base_y - py * arrow_width
        self.display.setColor(self.config.robot_arrow_colour)
        self.display.fillPolygon(
            [tip_x, left_x, right_x],
            [tip_y, left_y, right_y],
        )

    def _draw_text(self, state: DisplayState) -> None:
        self.display.setColor(self.config.text_colour)
        command = state.navigation_command
        if command is None:
            command = "None"
        target = state.target_position
        if target is None:
            target_text = "None"
        else:
            target_text = str(target)
        first_line = (
            f"State: {state.explorer_state}  "
            f"Pos: {state.robot_position}  "
            f"Dir: {state.robot_direction.name}"
        )
        second_line = (
            f"Next: {state.next_step_name}  Cmd: {command}  Target: {target_text}"
        )
        self.display.drawText(first_line, 4, 4)
        self.display.drawText(
            second_line,
            4,
            4 + self.config.text_line_height,
        )

    def _draw_cell(
        self,
        position: Position,
        colour: int,
        inset: int = 1,
    ) -> None:
        x, y = self._cell_top_left(position)
        safe_inset = min(
            max(0, inset),
            max(0, (self.view.cell_size - 1) // 2),
        )
        size = max(1, self.view.cell_size - safe_inset * 2)
        self.display.setColor(colour)
        self.display.fillRectangle(x + safe_inset, y + safe_inset, size, size)
        self.display.setColor(self.config.grid_line_colour)
        self.display.drawRectangle(x, y, self.view.cell_size, self.view.cell_size)

    def _cell_top_left(self, position: Position) -> tuple[int, int]:
        grid_x, grid_y = position
        screen_x = (
            self.view.offset_x + (grid_x - self.view.map_min_x) * self.view.cell_size
        )
        screen_y = (
            self.view.offset_y + (grid_y - self.view.map_min_y) * self.view.cell_size
        )
        return screen_x, screen_y

    def _cell_centre(self, position: Position) -> tuple[int, int]:
        x, y = self._cell_top_left(position)
        return x + self.view.cell_size // 2, y + self.view.cell_size // 2

    def _path_to_positions(
        self,
        start_position: Position,
        path: list[Direction],
    ) -> list[Position]:
        positions: list[Position] = []
        current = start_position
        for direction in path:
            current = move(current, direction)
            positions.append(current)
        return positions

    def _colour_for_cell(
        self,
        position: Position,
        cell: Cell,
        visited: set[Position],
    ) -> int:
        if cell == Cell.WALL:
            return self.config.wall_colour
        if cell == Cell.DANGER:
            return self.config.danger_colour
        if cell == Cell.GOAL:
            return self.config.goal_colour
        if cell == Cell.FREE:
            if position in visited:
                return self.config.visited_free_colour
            return self.config.unvisited_free_colour
        return self.config.unvisited_free_colour
