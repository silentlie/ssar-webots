import random

WALL = "#"
FREE = "."
START = "S"
GOAL = "G"
DANGER = "R"


def generate_prim_maze(
    width: int, height: int, seed: int | None = None
) -> list[list[str]]:
    if width % 2 == 0:
        width += 1
    if height % 2 == 0:
        height += 1

    rng = random.Random(seed)

    maze = [[WALL for _ in range(width)] for _ in range(height)]

    def in_bounds(x: int, y: int) -> bool:
        return 1 <= x < width - 1 and 1 <= y < height - 1

    def add_frontier(x: int, y: int, frontier: list[tuple[int, int]]) -> None:
        for dx, dy in [(2, 0), (-2, 0), (0, 2), (0, -2)]:
            nx, ny = x + dx, y + dy

            if in_bounds(nx, ny) and maze[ny][nx] == WALL:
                frontier.append((nx, ny))

    start_x = rng.randrange(1, width, 2)
    start_y = rng.randrange(1, height, 2)

    maze[start_y][start_x] = FREE

    frontier: list[tuple[int, int]] = []
    add_frontier(start_x, start_y, frontier)

    while frontier:
        x, y = frontier.pop(rng.randrange(len(frontier)))

        if maze[y][x] == FREE:
            continue

        connected = []

        for dx, dy in [(2, 0), (-2, 0), (0, 2), (0, -2)]:
            nx, ny = x + dx, y + dy

            if in_bounds(nx, ny) and maze[ny][nx] == FREE:
                connected.append((nx, ny))

        if not connected:
            continue

        nx, ny = rng.choice(connected)

        maze[y][x] = FREE
        maze[(y + ny) // 2][(x + nx) // 2] = FREE

        add_frontier(x, y, frontier)

    return maze


def carve_loops(
    maze: list[list[str]],
    loop_ratio: float = 0.15,
    seed: int | None = None,
) -> None:
    rng = random.Random(seed)

    height = len(maze)
    width = len(maze[0])

    candidates = []

    for y in range(1, height - 1):
        for x in range(1, width - 1):
            if maze[y][x] != WALL:
                continue

            left_right = maze[y][x - 1] == FREE and maze[y][x + 1] == FREE
            up_down = maze[y - 1][x] == FREE and maze[y + 1][x] == FREE

            if left_right or up_down:
                candidates.append((x, y))

    rng.shuffle(candidates)

    remove_count = int(len(candidates) * loop_ratio)

    for x, y in candidates[:remove_count]:
        maze[y][x] = FREE


def count_open_neighbors(
    maze: list[list[str]],
    x: int,
    y: int,
) -> int:
    count = 0

    for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
        if maze[y + dy][x + dx] == FREE:
            count += 1

    return count


def find_wall_tiles(maze: list[list[str]]) -> list[tuple[int, int]]:
    height = len(maze)
    width = len(maze[0])

    return [
        (x, y)
        for y in range(1, height - 1)
        for x in range(1, width - 1)
        if maze[y][x] == WALL
    ]


def find_dead_end_tiles(maze: list[list[str]]) -> list[tuple[int, int]]:
    height = len(maze)
    width = len(maze[0])

    dead_ends = []

    for y in range(1, height - 1):
        for x in range(1, width - 1):
            if maze[y][x] != FREE:
                continue

            if count_open_neighbors(maze, x, y) == 1:
                dead_ends.append((x, y))

    return dead_ends


def percentage_count(
    total: int,
    ratio: float,
    minimum: int = 0,
) -> int:
    if total <= 0 or ratio <= 0:
        return 0

    count = int(total * ratio)

    if minimum > 0:
        count = max(minimum, count)

    return min(count, total)


def place_start_near_outer_wall(
    maze: list[list[str]],
    seed: int | None = None,
) -> tuple[int, int]:
    """
    Place S on a random FREE tile that is closest to the outer wall.

    Unlike place_outer_wall_start(), this does not place S on the boundary
    and does not carve an entrance.
    """

    rng = random.Random(seed)

    height = len(maze)
    width = len(maze[0])

    candidates: list[tuple[int, int, int]] = []

    for y in range(1, height - 1):
        for x in range(1, width - 1):
            if maze[y][x] != FREE:
                continue

            distance_to_outer_wall = min(
                x,
                y,
                width - 1 - x,
                height - 1 - y,
            )

            candidates.append((distance_to_outer_wall, x, y))

    if not candidates:
        raise ValueError("No valid free tile found for start placement.")

    min_distance = min(distance for distance, _, _ in candidates)

    closest_tiles = [
        (x, y) for distance, x, y in candidates if distance == min_distance
    ]

    sx, sy = rng.choice(closest_tiles)

    maze[sy][sx] = START

    return sx, sy


def place_dangers_on_walls(
    maze: list[list[str]],
    danger_ratio: float = 0.10,
    seed: int | None = None,
) -> list[tuple[int, int]]:

    rng = random.Random(seed)

    wall_tiles = find_wall_tiles(maze)

    count = percentage_count(
        total=len(wall_tiles),
        ratio=danger_ratio,
        minimum=1,
    )

    danger_tiles = rng.sample(wall_tiles, count)

    for x, y in danger_tiles:
        maze[y][x] = DANGER

    return danger_tiles


def place_goals_on_dead_ends(
    maze: list[list[str]],
    goal_ratio: float = 0.01,
    seed: int | None = None,
) -> list[tuple[int, int]]:

    rng = random.Random(seed)

    dead_ends = find_dead_end_tiles(maze)

    count = percentage_count(
        total=len(dead_ends),
        ratio=goal_ratio,
        minimum=1,
    )

    goal_tiles = rng.sample(dead_ends, count)

    for x, y in goal_tiles:
        maze[y][x] = GOAL

    return goal_tiles


def generate_decision_heavy_maze(
    width: int = 51,
    height: int = 51,
    seed: int | None = None,
    loop_ratio: float = 0.10,
    danger_ratio: float = 0.10,
    goal_ratio: float = 0.05,
) -> list[list[str]]:

    maze = generate_prim_maze(
        width=width,
        height=height,
        seed=seed,
    )

    carve_loops(
        maze,
        loop_ratio=loop_ratio,
        seed=seed,
    )

    place_start_near_outer_wall(
        maze,
        seed=seed,
    )

    place_dangers_on_walls(
        maze,
        danger_ratio=danger_ratio,
        seed=seed,
    )

    place_goals_on_dead_ends(
        maze,
        goal_ratio=goal_ratio,
        seed=seed,
    )

    return maze


def print_maze(maze: list[list[str]]) -> None:
    for row in maze:
        print("".join(row))


if __name__ == "__main__":
    maze = generate_decision_heavy_maze()

    print_maze(maze)
