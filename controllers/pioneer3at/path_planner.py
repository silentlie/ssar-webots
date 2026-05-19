import heapq
from collections import deque
from enum import Enum, auto

from domain import Direction, Position, move
from grid_map import GridMap


class PathAlgorithm(Enum):
    BFS = auto()
    A_STAR = auto()


class PathPlanner:
    """Plan grid paths through cells that GridMap marks as enterable."""

    def __init__(
        self,
        grid_map: GridMap,
        algorithm: PathAlgorithm = PathAlgorithm.A_STAR,
    ) -> None:
        """Create a planner using the selected algorithm."""
        self.grid_map = grid_map
        self.algorithm = algorithm

    def set_algorithm(self, algorithm: PathAlgorithm) -> None:
        """Change the path-planning algorithm used by find_path."""
        self.algorithm = algorithm

    def find_path(
        self,
        start: Position,
        target: Position,
    ) -> list[Direction] | None:
        """Return a direction list from start to target, or None if unreachable."""
        if self.algorithm == PathAlgorithm.BFS:
            return self.bfs(start, target)
        if self.algorithm == PathAlgorithm.A_STAR:
            return self.a_star(start, target)
        raise ValueError(f"Unsupported path planning algorithm: {self.algorithm}")

    def bfs(
        self,
        start: Position,
        target: Position,
    ) -> list[Direction] | None:
        """Find the shortest path using breadth-first search."""
        if start == target:
            return []
        if not self.grid_map.can_enter(target):
            return None
        queue: deque[Position] = deque([start])
        visited: set[Position] = {start}
        previous: dict[Position, tuple[Position, Direction]] = {}
        while queue:
            current = queue.popleft()
            for direction in Direction:
                next_position = move(current, direction)
                if next_position in visited:
                    continue
                if not self.grid_map.can_enter(next_position):
                    continue
                visited.add(next_position)
                previous[next_position] = (current, direction)
                if next_position == target:
                    return self._reconstruct_path(previous, start, target)
                queue.append(next_position)
        return None

    def a_star(
        self,
        start: Position,
        target: Position,
    ) -> list[Direction] | None:
        """Find the shortest path using A* with Manhattan distance."""
        if start == target:
            return []
        if not self.grid_map.can_enter(target):
            return None
        open_heap: list[tuple[int, int, int, Position]] = []
        heapq.heappush(open_heap, (0, 0, 0, start))
        cost_so_far: dict[Position, int] = {start: 0}
        previous: dict[Position, tuple[Position, Direction]] = {}
        # The counter avoids comparing Position tuples when heap priorities tie.
        counter = 0
        while open_heap:
            _, current_cost, _, current = heapq.heappop(open_heap)
            if current_cost != cost_so_far[current]:
                continue
            if current == target:
                return self._reconstruct_path(previous, start, target)
            for direction in Direction:
                next_position = move(current, direction)
                if not self.grid_map.can_enter(next_position):
                    continue
                new_cost = cost_so_far[current] + 1
                if (
                    next_position not in cost_so_far
                    or new_cost < cost_so_far[next_position]
                ):
                    cost_so_far[next_position] = new_cost
                    priority = new_cost + self._manhattan_distance(
                        next_position, target
                    )
                    counter += 1
                    heapq.heappush(
                        open_heap,
                        (priority, new_cost, counter, next_position),
                    )
                    previous[next_position] = (current, direction)
        return None

    def _manhattan_distance(self, a: Position, b: Position) -> int:
        """Return Manhattan distance between two grid positions."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def _reconstruct_path(
        self,
        previous: dict[Position, tuple[Position, Direction]],
        start: Position,
        target: Position,
    ) -> list[Direction]:
        """Walk parent links backward and return directions from start to target."""
        path: list[Direction] = []
        current = target
        while current != start:
            parent, direction = previous[current]
            path.append(direction)
            current = parent
        path.reverse()
        return path
