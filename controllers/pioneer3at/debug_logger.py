from enum import IntEnum


class DebugLevel(IntEnum):
    NONE = 0
    ERROR = 1
    WARN = 2
    INFO = 3
    DEBUG = 4
    TRACE = 5


class DebugLogger:
    def __init__(
        self,
        name: str,
        level: DebugLevel = DebugLevel.NONE,
        context_level: DebugLevel = DebugLevel.INFO,
    ) -> None:
        self.name = name
        self.level = level
        self.context_level = context_level

    def enabled_for(self, level: DebugLevel) -> bool:
        return self.level >= level

    def should_show_context(self) -> bool:
        return self.level >= self.context_level

    def log(
        self,
        level: DebugLevel,
        context: str,
        *values: object,
        sep: str = " ",
        end: str = "\n",
        flush: bool = False,
    ) -> None:
        if not self.enabled_for(level):
            return

        if self.should_show_context():
            prefix = f"[{level.name}][{self.name}.{context}]"
        else:
            prefix = f"[{level.name}][{self.name}]"

        print(prefix, *values, sep=sep, end=end, flush=flush)

    def error(self, context: str, *values: object) -> None:
        self.log(DebugLevel.ERROR, context, *values)

    def warn(self, context: str, *values: object) -> None:
        self.log(DebugLevel.WARN, context, *values)

    def info(self, context: str, *values: object) -> None:
        self.log(DebugLevel.INFO, context, *values)

    def debug(self, context: str, *values: object) -> None:
        self.log(DebugLevel.DEBUG, context, *values)

    def trace(self, context: str, *values: object) -> None:
        self.log(DebugLevel.TRACE, context, *values)
