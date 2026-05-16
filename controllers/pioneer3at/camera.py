from typing import cast

from controller import Camera, Robot


class CameraController:
    def __init__(self, robot: Robot) -> None:
        self.camera = cast(Camera, robot.getDevice("camera"))
        self.camera.enable(int(robot.getBasicTimeStep()))

    def is_red_pixel(self, r: int, g: int, b: int) -> bool:
        return r > 120 and r > g * 1.5 and r > b * 1.5

    def is_green_pixel(self, r: int, g: int, b: int) -> bool:
        return g > 120 and g > r * 1.5 and g > b * 1.5
