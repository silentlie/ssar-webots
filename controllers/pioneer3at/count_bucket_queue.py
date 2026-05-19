from domain import Position

QueueRecord = tuple[Position, int]


class CountBucketQueue:
    def __init__(self) -> None:
        self._buckets: dict[int, list[Position]] = {}
        self._active: set[Position] = set()
        self._counts: dict[Position, int] = {}
        self._min_count: int | None = None

    def __len__(self) -> int:
        return len(self._active)

    def contains(self, position: Position) -> bool:
        return position in self._active

    @property
    def positions(self) -> set[Position]:
        return set(self._active)

    def count_for(self, position: Position) -> int:
        return self._counts.get(position, 0)

    def add(self, position: Position, count: int = 0) -> None:
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
        if position not in self._active:
            return None
        self._active.remove(position)
        count = self._counts.pop(position)
        return (position, count)

    def increment(self, position: Position) -> int:
        count = self.count_for(position) + 1
        self.remove(position)
        self.add(position, count)
        return count

    def peek(self) -> Position | None:
        self._clean()
        if self._min_count is None:
            return None
        return self._buckets[self._min_count][-1]

    def pop(self) -> QueueRecord | None:
        self._clean()
        if self._min_count is None:
            return None
        bucket = self._buckets[self._min_count]
        position = bucket.pop()
        count = self._counts.pop(position)
        self._active.remove(position)
        return (position, count)

    def clear(self) -> None:
        self._buckets.clear()
        self._active.clear()
        self._counts.clear()
        self._min_count = None

    def _clean(self) -> None:
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
                bucket.pop()
            self._buckets.pop(self._min_count)
        self._min_count = None
