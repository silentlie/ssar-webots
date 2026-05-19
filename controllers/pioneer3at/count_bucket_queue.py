from domain import Position

QueueRecord = tuple[Position, int]


class CountBucketQueue:
    """Queue positions by visit count while keeping O(1) membership checks."""

    def __init__(self) -> None:
        """Create an empty bucket queue."""
        self._buckets: dict[int, list[Position]] = {}
        self._active: set[Position] = set()
        self._counts: dict[Position, int] = {}
        self._min_count: int | None = None

    def __len__(self) -> int:
        """Return the number of active positions."""
        return len(self._active)

    def contains(self, position: Position) -> bool:
        """Return True when position is currently active in the queue."""
        return position in self._active

    @property
    def positions(self) -> set[Position]:
        """Return a copy of all active positions."""
        return set(self._active)

    def count_for(self, position: Position) -> int:
        """Return the stored count for position, or zero when absent."""
        return self._counts.get(position, 0)

    def add(self, position: Position, count: int = 0) -> None:
        """Add position to the bucket for count if it is not already active."""
        if position in self._active:
            return
        count = max(0, count)
        if count not in self._buckets:
            self._buckets[count] = []
        self._buckets[count].append(position)
        self._active.add(position)
        self._counts[position] = count
        if self._min_count is None or count < self._min_count:
            self._min_count = count

    def remove(self, position: Position) -> QueueRecord | None:
        """Remove position from active membership and return its record."""
        if position not in self._active:
            return None
        self._active.remove(position)
        count = self._counts.pop(position)
        return (position, count)

    def increment(self, position: Position) -> int:
        """Increase position's count and move it to the matching bucket."""
        count = self.count_for(position) + 1
        self.remove(position)
        self.add(position, count)
        return count

    def peek(self) -> Position | None:
        """Return the next position without removing it."""
        self._clean()
        if self._min_count is None:
            return None
        return self._buckets[self._min_count][-1]

    def pop(self) -> QueueRecord | None:
        """Remove and return the next position and its count."""
        self._clean()
        if self._min_count is None:
            return None
        bucket = self._buckets[self._min_count]
        position = bucket.pop()
        count = self._counts.pop(position)
        self._active.remove(position)
        return (position, count)

    def clear(self) -> None:
        """Remove all queued positions and count state."""
        self._buckets.clear()
        self._active.clear()
        self._counts.clear()
        self._min_count = None

    def _clean(self) -> None:
        """Drop stale bucket entries until _min_count points at a live item."""
        while self._buckets:
            if self._min_count is None or self._min_count not in self._buckets:
                self._min_count = min(self._buckets)
            bucket = self._buckets[self._min_count]
            while bucket:
                position = bucket[-1]
                if (
                    position in self._active
                    and self._counts.get(position) == self._min_count
                ):
                    return
                # Removed or re-bucketed positions are cleaned lazily from lists.
                bucket.pop()
            self._buckets.pop(self._min_count)
        self._min_count = None
