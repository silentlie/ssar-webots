from typing import cast

from controller import Camera, Robot


class CameraController:
    """Enables the robot camera and provides simple color classifiers."""

    def __init__(self, robot: Robot) -> None:
        self.camera = cast(Camera, robot.getDevice("camera"))
        self.camera.enable(int(robot.getBasicTimeStep()))

    def is_red_pixel(self, r: int, g: int, b: int) -> bool:
        """Return True for saturated red pixels used by danger markers."""
        return r > 120 and r > g * 1.5 and r > b * 1.5

    def is_green_pixel(self, r: int, g: int, b: int) -> bool:
        """Return True for saturated green pixels used by goal markers."""
        return g > 120 and g > r * 1.5 and g > b * 1.5
