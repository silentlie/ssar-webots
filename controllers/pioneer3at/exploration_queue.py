from count_bucket_queue import CountBucketQueue
from domain import Position


class ExplorationQueue:
    """Track frontier and visited cells for repeated exploration passes."""

    def __init__(self) -> None:
        """Create empty frontier and visited queues."""
        self._frontier = CountBucketQueue()
        self._visited = CountBucketQueue()

    @property
    def visited(self) -> set[Position]:
        """Return a copy of visited positions."""
        return self._visited.positions

    def add(self, position: Position) -> None:
        """Add an unvisited position to the frontier."""
        if self._visited.contains(position):
            return
        self._frontier.remove(position)
        self._frontier.add(position)

    def visit(self, position: Position) -> None:
        """Mark position visited and count repeat visits."""
        self._frontier.remove(position)
        if not self._visited.contains(position):
            self._visited.add(position, 1)
        else:
            self._visited.increment(position)

    def peek(self) -> Position | None:
        """Return the next frontier position without removing it."""
        return self._frontier.peek()

    def discard(self, position: Position) -> None:
        """Remove position from the frontier, preserving visited state."""
        self._frontier.remove(position)

    def is_visited(self, position: Position) -> bool:
        """Return True when position has been visited."""
        return position in self._visited.positions

    def reload_frontier(self) -> None:
        """Requeue visited positions as frontier entries for another pass."""
        if len(self._visited) == 0:
            return
        # Keep visit counts with the positions so low-count cells are retried first.
        self._frontier = self._visited
        self._visited = CountBucketQueue()
