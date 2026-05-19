"""Webots entry point for the autonomous Pioneer 3AT controller."""

from config import OdometryConfig
from controller import Robot
from debug_logger import DebugLevel
from display_controller import DisplayController
from explorer import Explorer
from grid_map import GridMap
from navigation import Navigation
from odometry import Odometry
from sensors import Sensors
from vision_perception import VisionPerception
from wheels import Wheels

TILE_SIZE = 1
DEBUG = False
DEBUG_LEVEL = DebugLevel.DEBUG if DEBUG else DebugLevel.NONE
START_DELAY_SECONDS = 1.0

robot = Robot()
timestep = int(robot.getBasicTimeStep())
sensors = Sensors(robot, debug_level=DEBUG_LEVEL)
vision = VisionPerception(robot, debug_level=DEBUG_LEVEL)
wheels = Wheels(robot, default_turn_speed=2, debug_level=DEBUG_LEVEL)
odometry = Odometry(
    robot,
    config=OdometryConfig(tile_size=TILE_SIZE),
    debug_level=DEBUG_LEVEL,
)
grid_map = GridMap()
navigation = Navigation(wheels, odometry, grid_map, sensors, debug=DEBUG)
display = DisplayController(robot, debug=DEBUG)
explorer = Explorer(sensors, grid_map, navigation, display, vision, debug=DEBUG)

start_time = robot.getTime()
if DEBUG:
    print(
        f"[Main] Waiting {START_DELAY_SECONDS} seconds before starting exploration..."
    )
while robot.step(timestep) != -1:
    elapsed_time = robot.getTime() - start_time
    if elapsed_time < START_DELAY_SECONDS:
        # Let enabled Webots sensors produce stable first readings before motion.
        wheels.stop()
        continue
    explorer.update()
