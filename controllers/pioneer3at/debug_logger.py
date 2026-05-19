from enum import IntEnum


class DebugLevel(IntEnum):
    NONE = 0
    ERROR = 1
    WARN = 2
    INFO = 3
    DEBUG = 4
    TRACE = 5


class DebugLogger:
    """Small level-based logger for Webots controller diagnostics."""

    def __init__(
        self,
        name: str,
        level: DebugLevel = DebugLevel.NONE,
        context_level: DebugLevel = DebugLevel.INFO,
    ) -> None:
        """Create a logger with a component name and active level."""
        self.name = name
        self.level = level
        self.context_level = context_level

    def enabled_for(self, level: DebugLevel) -> bool:
        """Return True when messages at level should be emitted."""
        return self.level >= level

    def should_show_context(self) -> bool:
        """Return True when log prefixes should include call context."""
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
        """Print a log message if level is enabled."""
        if not self.enabled_for(level):
            return

        if self.should_show_context():
            prefix = f"[{level.name}][{self.name}.{context}]"
        else:
            prefix = f"[{level.name}][{self.name}]"

        print(prefix, *values, sep=sep, end=end, flush=flush)

    def error(self, context: str, *values: object) -> None:
        """Log an error-level message."""
        self.log(DebugLevel.ERROR, context, *values)

    def warn(self, context: str, *values: object) -> None:
        """Log a warning-level message."""
        self.log(DebugLevel.WARN, context, *values)

    def info(self, context: str, *values: object) -> None:
        """Log an info-level message."""
        self.log(DebugLevel.INFO, context, *values)

    def debug(self, context: str, *values: object) -> None:
        """Log a debug-level message."""
        self.log(DebugLevel.DEBUG, context, *values)

    def trace(self, context: str, *values: object) -> None:
        """Log a trace-level message."""
        self.log(DebugLevel.TRACE, context, *values)
