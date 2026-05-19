from enum import IntEnum

# Grid coordinates are (x, y), with y increasing downward on the display.
Position = tuple[int, int]


class Direction(IntEnum):
    UP = 0
    RIGHT = 1
    DOWN = 2
    LEFT = 3


class RelativeDirection(IntEnum):
    FRONT = 0
    RIGHT = 1
    BACK = 2
    LEFT = 3


class Cell(IntEnum):
    UNKNOWN = 0
    FREE = 1
    WALL = 2
    DANGER = 3
    GOAL = 4


DIRECTION_DELTAS: dict[Direction, Position] = {
    Direction.UP: (0, -1),
    Direction.RIGHT: (1, 0),
    Direction.DOWN: (0, 1),
    Direction.LEFT: (-1, 0),
}


def move(position: Position, direction: Direction) -> Position:
    dx, dy = DIRECTION_DELTAS[direction]
    return (position[0] + dx, position[1] + dy)


def left_of(direction: Direction) -> Direction:
    return Direction((direction.value - 1) % 4)


def right_of(direction: Direction) -> Direction:
    return Direction((direction.value + 1) % 4)


def opposite_of(direction: Direction) -> Direction:
    return Direction((direction.value + 2) % 4)


def relative_to_absolute(
    robot_direction: Direction,
    relative_direction: RelativeDirection,
) -> Direction:
    return Direction((robot_direction.value + relative_direction.value) % 4)
