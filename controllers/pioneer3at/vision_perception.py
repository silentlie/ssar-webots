from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable, cast

from config import PerceptionConfig
from controller import Camera, Robot
from debug_logger import DebugLevel, DebugLogger

ColourMatcher = Callable[[int, int, int], bool]


class DetectionResult(Enum):
    DETECTED = auto()
    CLEAR = auto()
    UNCERTAIN = auto()

    def detected(self) -> bool:
        return self == DetectionResult.DETECTED

    def clear(self) -> bool:
        return self == DetectionResult.CLEAR

    def uncertain(self) -> bool:
        return self == DetectionResult.UNCERTAIN


@dataclass(frozen=True)
class ColourScan:
    ratio: float
    matched: int
    checked: int


class VisionPerception:
    CAMERA_NAME = "camera"

    def __init__(
        self,
        robot: Robot,
        config: PerceptionConfig = PerceptionConfig(),
        debug_level: DebugLevel = DebugLevel.NONE,
    ) -> None:
        self.config = config
        self.logger = DebugLogger("VisionPerception", debug_level)
        timestep = int(robot.getBasicTimeStep())
        self.camera = cast(Camera, robot.getDevice(self.CAMERA_NAME))
        self.camera.enable(timestep)
        self.danger_score = 0
        self.goal_score = 0
        self.goal_visible_score = 0
        self.logger.debug(
            "__init__",
            f"danger_ratio={self.config.danger_ratio:.2f}, "
            f"danger_pixels={self.config.danger_pixels}, "
            f"goal_ratio={self.config.goal_ratio:.2f}, "
            f"goal_pixels={self.config.goal_pixels}, "
            f"goal_visible_ratio={self.config.goal_visible_ratio:.2f}, "
            f"goal_visible_pixels={self.config.goal_visible_pixels}, "
            f"sample_step={self.config.sample_step}",
        )

    def check_danger_ahead(self) -> DetectionResult:
        result, self.danger_score = self._check_colour(
            context="check_danger_ahead",
            score=self.danger_score,
            region_ratio=self.config.close_region,
            matcher=self._is_red,
            min_pixels=self.config.danger_pixels,
            min_ratio=self.config.danger_ratio,
            confirm=self.config.danger_confirm,
            clear=self.config.danger_clear,
        )
        return result

    def check_goal_ahead(self) -> DetectionResult:
        result, self.goal_score = self._check_colour(
            context="check_goal_ahead",
            score=self.goal_score,
            region_ratio=self.config.close_region,
            matcher=self._is_green,
            min_pixels=self.config.goal_pixels,
            min_ratio=self.config.goal_ratio,
            confirm=self.config.goal_confirm,
            clear=self.config.goal_clear,
        )
        return result

    def check_goal_visible_ahead(self) -> DetectionResult:
        result, self.goal_visible_score = self._check_colour(
            context="check_goal_visible_ahead",
            score=self.goal_visible_score,
            region_ratio=self.config.far_region,
            matcher=self._is_green,
            min_pixels=self.config.goal_visible_pixels,
            min_ratio=self.config.goal_visible_ratio,
            confirm=self.config.goal_visible_confirm,
            clear=self.config.goal_visible_clear,
        )
        return result

    def reset_all(self) -> None:
        self.danger_score = 0
        self.goal_score = 0
        self.goal_visible_score = 0
        self.logger.trace("reset_all", "all vision scores reset")

    def reset_danger(self) -> None:
        self.danger_score = 0
        self.logger.trace("reset_danger", "danger score reset")

    def reset_goal(self) -> None:
        self.goal_score = 0
        self.logger.trace("reset_goal", "goal score reset")

    def reset_goal_visible(self) -> None:
        self.goal_visible_score = 0
        self.logger.trace("reset_goal_visible", "goal visible score reset")

    def _check_colour(
        self,
        context: str,
        score: int,
        region_ratio: float,
        matcher: ColourMatcher,
        min_pixels: int,
        min_ratio: float,
        confirm: int,
        clear: int,
    ) -> tuple[DetectionResult, int]:
        scan = self._scan_colour(
            region_ratio=region_ratio,
            matcher=matcher,
        )
        candidate = (
            scan.checked > 0 and scan.matched >= min_pixels and scan.ratio >= min_ratio
        )
        next_score = self._next_score(
            score=score,
            candidate=candidate,
            confirm=confirm,
            clear=clear,
        )
        result = self._score_state(
            score=next_score,
            confirm=confirm,
            clear=clear,
        )
        self.logger.trace(
            context,
            f"ratio={scan.ratio:.2f}, "
            f"matched={scan.matched}, "
            f"checked={scan.checked}, "
            f"candidate={candidate}, "
            f"score={next_score}, "
            f"result={result.name}",
        )
        return result, next_score

    def _scan_colour(
        self,
        region_ratio: float,
        matcher: ColourMatcher,
    ) -> ColourScan:
        image = self.camera.getImage()
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        if image is None or width <= 0 or height <= 0:
            self.logger.error("_scan_colour", "camera image unavailable")
            return ColourScan(ratio=0.0, matched=0, checked=0)
        x_start, x_end, y_start, y_end = self._region_bounds(
            width=width,
            height=height,
            ratio=region_ratio,
        )
        matched = 0
        checked = 0
        for y in range(y_start, y_end, self.config.sample_step):
            for x in range(x_start, x_end, self.config.sample_step):
                red = self.camera.imageGetRed(image, width, x, y)
                green = self.camera.imageGetGreen(image, width, x, y)
                blue = self.camera.imageGetBlue(image, width, x, y)
                checked += 1
                if matcher(red, green, blue):
                    matched += 1
        if checked == 0:
            return ColourScan(ratio=0.0, matched=0, checked=0)
        return ColourScan(
            ratio=matched / checked,
            matched=matched,
            checked=checked,
        )

    def _region_bounds(
        self,
        width: int,
        height: int,
        ratio: float,
    ) -> tuple[int, int, int, int]:
        ratio = max(0.1, min(1.0, ratio))
        region_width = max(1, int(width * ratio))
        region_height = max(1, int(height * ratio))
        x_start = max(0, (width - region_width) // 2)
        y_start = max(0, (height - region_height) // 2)
        x_end = min(width, x_start + region_width)
        y_end = min(height, y_start + region_height)
        return x_start, x_end, y_start, y_end

    def _is_red(self, red: int, green: int, blue: int) -> bool:
        strongest_non_red = max(green, blue)
        return (
            red >= self.config.red_min
            and red - strongest_non_red >= self.config.red_gap
            and red >= green * self.config.red_dominance
            and red >= blue * self.config.red_dominance
        )

    def _is_green(self, red: int, green: int, blue: int) -> bool:
        strongest_non_green = max(red, blue)
        return (
            green >= self.config.green_min
            and green - strongest_non_green >= self.config.green_gap
            and green >= red * self.config.green_dominance
            and green >= blue * self.config.green_dominance
        )

    def _next_score(
        self,
        score: int,
        candidate: bool,
        confirm: int,
        clear: int,
    ) -> int:
        if candidate:
            score += 1
        else:
            score -= 1
        return max(-clear, min(confirm, score))

    def _score_state(
        self,
        score: int,
        confirm: int,
        clear: int,
    ) -> DetectionResult:
        if score >= confirm:
            return DetectionResult.DETECTED
        if score <= -clear:
            return DetectionResult.CLEAR
        return DetectionResult.UNCERTAIN
