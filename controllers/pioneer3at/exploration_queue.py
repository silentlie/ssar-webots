from count_bucket_queue import CountBucketQueue
from domain import Position


class ExplorationQueue:
    def __init__(self) -> None:
        self._frontier = CountBucketQueue()
        self._visited = CountBucketQueue()

    @property
    def visited(self) -> set[Position]:
        return self._visited.positions

    def add(self, position: Position) -> None:
        if self._visited.contains(position):
            return
        self._frontier.remove(position)
        self._frontier.add(position)

    def visit(self, position: Position) -> None:
        self._frontier.remove(position)
        if not self._visited.contains(position):
            self._visited.add(position, 1)
        else:
            self._visited.increment(position)

    def peek(self) -> Position | None:
        return self._frontier.peek()

    def discard(self, position: Position) -> None:
        self._frontier.remove(position)

    def is_visited(self, position: Position) -> bool:
        return position in self._visited.positions

    def reload_frontier(self) -> None:
        if len(self._visited) == 0:
            return
        self._frontier = self._visited
        self._visited = CountBucketQueue()
