import math
from dataclasses import dataclass


@dataclass(frozen=True)
class SensorConfig:
    blocked: float = 900.0
    too_close: float = 980.0
    parallel_conflict: float = 15.0
    parallel_alpha: float = 0.7
    centre_alpha: float = 0.8


@dataclass(frozen=True)
class PerceptionConfig:
    danger_ratio: float = 0.90
    danger_pixels: int = 80
    danger_confirm: int = 5
    danger_clear: int = 5
    goal_ratio: float = 0.90
    goal_pixels: int = 80
    goal_confirm: int = 5
    goal_clear: int = 5
    goal_visible_ratio: float = 0.05
    goal_visible_pixels: int = 15
    goal_visible_confirm: int = 5
    goal_visible_clear: int = 5
    sample_step: int = 2
    close_region: float = 1.0
    far_region: float = 1.0
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


@dataclass(frozen=True)
class DisplayConfig:
    padding: int = 12
    top_margin: int = 32
    text_line_height: int = 12
    background_colour: int = 0x111111
    wall_colour: int = 0x000000
    visited_free_colour: int = 0xFFFFFF
    unvisited_free_colour: int = 0x808080
    danger_colour: int = 0xFF3333
    goal_colour: int = 0x00CC33
    path_colour: int = 0x8000FF
    target_colour: int = 0xFF66CC
    robot_colour: int = 0xFFD700
    robot_arrow_colour: int = 0x000000
    grid_line_colour: int = 0x666666
    text_colour: int = 0xFFFFFF


@dataclass(frozen=True)
class NavigationConfig:
    parallel_threshold: float = 10.0
    align_parallel_stable_steps: int = 10
    align_parallel_invalid_limit: int = 10
    parallel_forward_deadband: float = 0.0
    parallel_forward_kp: float = 0.015
    max_parallel_forward_correction: float = 0.5
    side_centre_threshold: float = 50.0
    centre_move_threshold: float = 3.0
    align_centre_invalid_limit: int = 10
