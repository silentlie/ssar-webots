"""Small shared helpers for the Pioneer 3AT controller modules."""


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(max_value, value))
