from typing import cast

from controller import Display, Robot
from gridMap import Cell, Direction, Position, move


class DisplayController:
    BACKGROUND_COLOR = 0x111111
    WALL_COLOR = 0x000000
    VISITED_FREE_COLOR = 0xFFFFFF
    UNVISITED_FREE_COLOR = 0x808080
    DANGER_COLOR = 0xFF3333
    GOAL_COLOR = 0x00CC33

    PATH_COLOR = 0x8000FF
    TARGET_COLOR = 0xFF66CC
    ROBOT_COLOR = 0xFFD700
    ROBOT_ARROW_COLOR = 0x000000
    GRID_LINE_COLOR = 0x666666
    TEXT_COLOR = 0xFFFFFF

    def __init__(
        self,
        robot: Robot,
        debug: bool = False,
        display_name: str = "display",
        padding: int = 12,
        top_margin: int = 30,
    ) -> None:
        self.display = cast(Display, robot.getDevice(display_name))
        self.debug = debug

        self.width = self.display.getWidth()
        self.height = self.display.getHeight()

        self.padding = padding
        self.top_margin = top_margin

        self.cell_size = 1
        self.map_min_x = 0
        self.map_min_y = 0
        self.offset_x = 0
        self.offset_y = 0

        self._debug(
            f"DisplayController initialized: width={self.width}, height={self.height}"
        )

    def update(
        self,
        grid: dict[Position, Cell],
        visited: set[Position],
        robot_position: Position,
        robot_direction: Direction,
        path: list[Direction],
    ) -> None:
        path_positions = self._path_to_positions(robot_position, path)

        self._update_view_bounds(
            grid=grid,
            robot_position=robot_position,
            path_positions=path_positions,
        )

        self._clear()
        self._draw_grid(grid, visited)
        self._draw_path(path_positions)
        self._draw_target(path_positions)
        self._draw_robot(robot_position, robot_direction)
        self._draw_text(robot_position, robot_direction, path)

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

        # Add a one-cell visual border.
        min_x -= 1
        max_x += 1
        min_y -= 1
        max_y += 1

        columns = max_x - min_x + 1
        rows = max_y - min_y + 1

        available_width = max(1, self.width - self.padding * 2)
        available_height = max(1, self.height - self.top_margin - self.padding)

        cell_from_width = available_width // columns
        cell_from_height = available_height // rows

        # Pure dynamic scale. No min/max clamp.
        self.cell_size = max(1, min(cell_from_width, cell_from_height))

        map_width = columns * self.cell_size
        map_height = rows * self.cell_size

        self.map_min_x = min_x
        self.map_min_y = min_y

        self.offset_x = (self.width - map_width) // 2
        self.offset_y = self.top_margin + (available_height - map_height) // 2

    def _clear(self) -> None:
        self.display.setColor(self.BACKGROUND_COLOR)
        self.display.fillRectangle(0, 0, self.width, self.height)

    def _draw_grid(
        self,
        grid: dict[Position, Cell],
        visited: set[Position],
    ) -> None:
        for position, cell in grid.items():
            color = self._color_for_cell(position, cell, visited)
            self._draw_cell(position, color, inset=1)

    def _draw_path(self, path_positions: list[Position]) -> None:
        for position in path_positions:
            self._draw_cell(
                position,
                self.PATH_COLOR,
                inset=max(2, self.cell_size // 4),
            )

    def _draw_target(self, path_positions: list[Position]) -> None:
        if len(path_positions) == 0:
            return

        self._draw_cell(
            path_positions[0],
            self.TARGET_COLOR,
            inset=max(2, self.cell_size // 5),
        )

    def _draw_robot(
        self,
        position: Position,
        direction: Direction,
    ) -> None:
        center_x, center_y = self._cell_center(position)

        radius = max(4, self.cell_size // 4)

        # Correct Webots usage:
        # fillOval(center_x, center_y, radius_x, radius_y)
        self.display.setColor(self.ROBOT_COLOR)
        self.display.fillOval(center_x, center_y, radius, radius)

        self._draw_robot_direction_triangle(position, direction)

    def _draw_robot_direction_triangle(
        self,
        position: Position,
        direction: Direction,
    ) -> None:
        center_x, center_y = self._cell_center(position)

        arrow_length = max(5, self.cell_size // 3)
        arrow_width = max(3, self.cell_size // 6)

        dx = 0
        dy = 0

        if direction == Direction.UP:
            dx, dy = 0, -1
        elif direction == Direction.RIGHT:
            dx, dy = 1, 0
        elif direction == Direction.DOWN:
            dx, dy = 0, 1
        elif direction == Direction.LEFT:
            dx, dy = -1, 0

        # Perpendicular vector.
        px = -dy
        py = dx

        tip_x = center_x + dx * arrow_length
        tip_y = center_y + dy * arrow_length

        base_x = center_x + dx * max(2, arrow_length // 3)
        base_y = center_y + dy * max(2, arrow_length // 3)

        left_x = base_x + px * arrow_width
        left_y = base_y + py * arrow_width

        right_x = base_x - px * arrow_width
        right_y = base_y - py * arrow_width

        self.display.setColor(self.ROBOT_ARROW_COLOR)
        self.display.fillPolygon(
            [tip_x, left_x, right_x],
            [tip_y, left_y, right_y],
        )

    def _draw_text(
        self,
        robot_position: Position,
        robot_direction: Direction,
        path: list[Direction],
    ) -> None:
        self.display.setColor(self.TEXT_COLOR)

        next_step = path[0].name if len(path) > 0 else "None"

        text = f"Pos: {robot_position}  Dir: {robot_direction.name}  Next: {next_step}"

        self.display.drawText(text, 4, 4)

    def _draw_cell(
        self,
        position: Position,
        color: int,
        inset: int = 1,
    ) -> None:
        x, y = self._cell_top_left(position)
        size = self.cell_size - inset * 2

        self.display.setColor(color)
        self.display.fillRectangle(x + inset, y + inset, size, size)

        self.display.setColor(self.GRID_LINE_COLOR)
        self.display.drawRectangle(x, y, self.cell_size, self.cell_size)

    def _cell_top_left(self, position: Position) -> tuple[int, int]:
        grid_x, grid_y = position

        screen_x = self.offset_x + (grid_x - self.map_min_x) * self.cell_size
        screen_y = self.offset_y + (grid_y - self.map_min_y) * self.cell_size

        return screen_x, screen_y

    def _cell_center(self, position: Position) -> tuple[int, int]:
        x, y = self._cell_top_left(position)
        return x + self.cell_size // 2, y + self.cell_size // 2

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

    def _color_for_cell(
        self,
        position: Position,
        cell: Cell,
        visited: set[Position],
    ) -> int:
        if cell == Cell.WALL:
            return self.WALL_COLOR

        if cell == Cell.DANGER:
            return self.DANGER_COLOR

        if cell == Cell.GOAL:
            return self.GOAL_COLOR

        if cell == Cell.FREE:
            if position in visited:
                return self.VISITED_FREE_COLOR
            return self.UNVISITED_FREE_COLOR

        return self.UNVISITED_FREE_COLOR

    def _debug(self, message: str) -> None:
        if self.debug:
            print(f"[DisplayController] {message}")
