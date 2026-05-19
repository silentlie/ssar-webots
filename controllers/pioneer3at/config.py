import math
from dataclasses import dataclass


@dataclass(frozen=True)
class SensorConfig:
    # Webots sonar raw-value thresholds, not metres.
    blocked: float = 900.0
    too_close: float = 980.0

    # If both side-wall alignment readings strongly disagree, ignore correction.
    parallel_conflict: float = 15.0

    # Exponential smoothing factors.
    parallel_alpha: float = 0.7
    centre_alpha: float = 0.8


@dataclass(frozen=True)
class PerceptionConfig:
    # Close red danger marker before entering next cell.
    danger_ratio: float = 0.90
    danger_pixels: int = 80
    danger_confirm: int = 5
    danger_clear: int = 5

    # Close green goal marker before entering next cell.
    goal_ratio: float = 0.90
    goal_pixels: int = 80
    goal_confirm: int = 5
    goal_clear: int = 5

    # Far green goal visible ahead, used to prioritise planned path.
    goal_visible_ratio: float = 0.05
    goal_visible_pixels: int = 15
    goal_visible_confirm: int = 5
    goal_visible_clear: int = 5

    # Image sampling.
    sample_step: int = 2
    close_region: float = 1.0
    far_region: float = 1.0

    # Colour matching.
    red_min: int = 90
    red_gap: int = 45
    red_dominance: float = 1.5

    green_min: int = 90
    green_gap: int = 45
    green_dominance: float = 1.5


@dataclass(frozen=True)
class OdometryConfig:
    tile_size: float = 1.0
    wheel_radius: float = 0.11
    axle_length: float = 0.585
    forward_tolerance: float = 0.01
    turn_tolerance: float = math.radians(1)
    forward_end_margin: float = 0.20
