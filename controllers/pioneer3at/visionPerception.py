from typing import Callable, cast

from controller import Camera, Robot


class VisionPerception:
    CAMERA_NAME = "camera"

    # Close red danger marker before entering next cell.
    DANGER_RED_RATIO_THRESHOLD = 0.90
    MIN_DANGER_RED_PIXELS = 80
    DANGER_CONFIRM_FRAMES = 5
    DANGER_CLEAR_FRAMES = 5

    # Close green goal marker before entering next cell.
    GOAL_GREEN_RATIO_THRESHOLD = 0.90
    MIN_GOAL_GREEN_PIXELS = 80
    GOAL_CONFIRM_FRAMES = 5
    GOAL_CLEAR_FRAMES = 5

    # Far green goal visible ahead, used to override/prioritise planned path.
    GOAL_VISIBLE_GREEN_RATIO_THRESHOLD = 0.05
    MIN_GOAL_VISIBLE_GREEN_PIXELS = 15
    GOAL_VISIBLE_CONFIRM_FRAMES = 5
    GOAL_VISIBLE_CLEAR_FRAMES = 5

    SAMPLE_STEP = 2

    # Close checks use the full image because the object should be up close.
    REGION_RATIO = 1.0

    # Far goal check focuses on the centre ahead region.
    GOAL_VISIBLE_REGION_RATIO = 0.6

    RED_MIN = 90
    RED_GAP = 45
    RED_DOMINANCE_RATIO = 1.5

    GREEN_MIN = 90
    GREEN_GAP = 45
    GREEN_DOMINANCE_RATIO = 1.5

    def __init__(
        self,
        robot: Robot,
        debug: bool = False,
    ) -> None:
        self.timestep = int(robot.getBasicTimeStep())

        self.camera = cast(Camera, robot.getDevice(self.CAMERA_NAME))
        self.camera.enable(self.timestep)

        self.debug = debug

        self._danger_score = 0
        self._goal_score = 0
        self._goal_visible_score = 0

    def check_danger_ahead(self) -> bool | None:
        red_ratio, red_pixels, checked_pixels = self._colour_ratio(
            region_ratio=self.REGION_RATIO,
            matcher=self._is_danger_red,
        )

        danger_candidate = (
            checked_pixels > 0
            and red_pixels >= self.MIN_DANGER_RED_PIXELS
            and red_ratio >= self.DANGER_RED_RATIO_THRESHOLD
        )

        self._danger_score = self._update_score(
            score=self._danger_score,
            candidate=danger_candidate,
            positive_limit=self.DANGER_CONFIRM_FRAMES,
            negative_limit=self.DANGER_CLEAR_FRAMES,
        )

        self._debug(
            "danger check: "
            f"red_ratio={red_ratio:.2f}, "
            f"red_pixels={red_pixels}, "
            f"checked_pixels={checked_pixels}, "
            f"candidate={danger_candidate}, "
            f"score={self._danger_score}"
        )

        return self._score_result(
            score=self._danger_score,
            positive_limit=self.DANGER_CONFIRM_FRAMES,
            negative_limit=self.DANGER_CLEAR_FRAMES,
        )

    def check_goal_ahead(self) -> bool | None:
        green_ratio, green_pixels, checked_pixels = self._colour_ratio(
            region_ratio=self.REGION_RATIO,
            matcher=self._is_goal_green,
        )

        goal_candidate = (
            checked_pixels > 0
            and green_pixels >= self.MIN_GOAL_GREEN_PIXELS
            and green_ratio >= self.GOAL_GREEN_RATIO_THRESHOLD
        )

        self._goal_score = self._update_score(
            score=self._goal_score,
            candidate=goal_candidate,
            positive_limit=self.GOAL_CONFIRM_FRAMES,
            negative_limit=self.GOAL_CLEAR_FRAMES,
        )

        self._debug(
            "goal check: "
            f"green_ratio={green_ratio:.2f}, "
            f"green_pixels={green_pixels}, "
            f"checked_pixels={checked_pixels}, "
            f"candidate={goal_candidate}, "
            f"score={self._goal_score}"
        )

        return self._score_result(
            score=self._goal_score,
            positive_limit=self.GOAL_CONFIRM_FRAMES,
            negative_limit=self.GOAL_CLEAR_FRAMES,
        )

    def check_goal_visible_ahead(self) -> bool | None:
        green_ratio, green_pixels, checked_pixels = self._colour_ratio(
            region_ratio=self.GOAL_VISIBLE_REGION_RATIO,
            matcher=self._is_goal_green,
        )

        goal_visible_candidate = (
            checked_pixels > 0
            and green_pixels >= self.MIN_GOAL_VISIBLE_GREEN_PIXELS
            and green_ratio >= self.GOAL_VISIBLE_GREEN_RATIO_THRESHOLD
        )

        self._goal_visible_score = self._update_score(
            score=self._goal_visible_score,
            candidate=goal_visible_candidate,
            positive_limit=self.GOAL_VISIBLE_CONFIRM_FRAMES,
            negative_limit=self.GOAL_VISIBLE_CLEAR_FRAMES,
        )

        self._debug(
            "goal visible check: "
            f"green_ratio={green_ratio:.2f}, "
            f"green_pixels={green_pixels}, "
            f"checked_pixels={checked_pixels}, "
            f"candidate={goal_visible_candidate}, "
            f"score={self._goal_visible_score}"
        )

        return self._score_result(
            score=self._goal_visible_score,
            positive_limit=self.GOAL_VISIBLE_CONFIRM_FRAMES,
            negative_limit=self.GOAL_VISIBLE_CLEAR_FRAMES,
        )

    def reset_all(self) -> None:
        self._danger_score = 0
        self._goal_score = 0
        self._goal_visible_score = 0

    def reset_danger(self) -> None:
        self._danger_score = 0

    def reset_goal(self) -> None:
        self._goal_score = 0

    def reset_goal_visible(self) -> None:
        self._goal_visible_score = 0

    def _colour_ratio(
        self,
        region_ratio: float,
        matcher: Callable[[int, int, int], bool],
    ) -> tuple[float, int, int]:
        image = self.camera.getImage()
        width = self.camera.getWidth()
        height = self.camera.getHeight()

        if image is None or width <= 0 or height <= 0:
            self._debug("camera image unavailable")
            return 0.0, 0, 0

        x_start, x_end, y_start, y_end = self._region_bounds(
            width,
            height,
            region_ratio,
        )

        matched_pixels = 0
        checked_pixels = 0

        for y in range(y_start, y_end, self.SAMPLE_STEP):
            for x in range(x_start, x_end, self.SAMPLE_STEP):
                red = self.camera.imageGetRed(image, width, x, y)
                green = self.camera.imageGetGreen(image, width, x, y)
                blue = self.camera.imageGetBlue(image, width, x, y)

                checked_pixels += 1

                if matcher(red, green, blue):
                    matched_pixels += 1

        if checked_pixels == 0:
            return 0.0, 0, 0

        return matched_pixels / checked_pixels, matched_pixels, checked_pixels

    def _region_bounds(
        self,
        width: int,
        height: int,
        region_ratio: float,
    ) -> tuple[int, int, int, int]:
        region_ratio = max(0.1, min(1.0, region_ratio))

        region_width = max(1, int(width * region_ratio))
        region_height = max(1, int(height * region_ratio))

        x_start = max(0, (width - region_width) // 2)
        y_start = max(0, (height - region_height) // 2)

        x_end = min(width, x_start + region_width)
        y_end = min(height, y_start + region_height)

        return x_start, x_end, y_start, y_end

    def _is_danger_red(self, red: int, green: int, blue: int) -> bool:
        strongest_non_red = max(green, blue)

        return (
            red >= self.RED_MIN
            and red - strongest_non_red >= self.RED_GAP
            and red >= green * self.RED_DOMINANCE_RATIO
            and red >= blue * self.RED_DOMINANCE_RATIO
        )

    def _is_goal_green(self, red: int, green: int, blue: int) -> bool:
        strongest_non_green = max(red, blue)

        return (
            green >= self.GREEN_MIN
            and green - strongest_non_green >= self.GREEN_GAP
            and green >= red * self.GREEN_DOMINANCE_RATIO
            and green >= blue * self.GREEN_DOMINANCE_RATIO
        )

    def _update_score(
        self,
        score: int,
        candidate: bool,
        positive_limit: int,
        negative_limit: int,
    ) -> int:
        if candidate:
            score += 1
        else:
            score -= 1

        return max(-negative_limit, min(positive_limit, score))

    def _score_result(
        self,
        score: int,
        positive_limit: int,
        negative_limit: int,
    ) -> bool | None:
        if score >= positive_limit:
            return True

        if score <= -negative_limit:
            return False

        return None

    def _debug(self, message: str) -> None:
        if self.debug:
            print(f"[VisionPerception] {message}")
