"""Webots entry point for the autonomous Pioneer 3AT controller."""

from config import OdometryConfig
from controller import Robot
from debug_logger import DebugLevel, DebugLogger
from display_controller import DisplayController, DisplayState
from explorer import Explorer
from grid_map import GridMap
from navigation import Navigation
from odometry import Odometry
from sensors import Sensors
from vision_perception import VisionPerception
from wheels import Wheels

TILE_SIZE = 1
DEBUG_LEVEL = DebugLevel.DEBUG
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
navigation = Navigation(wheels, odometry, grid_map, sensors, debug_level=DEBUG_LEVEL)
display = DisplayController(robot, debug_level=DEBUG_LEVEL)
explorer = Explorer(
    sensors,
    grid_map,
    navigation,
    vision,
    debug_level=DEBUG_LEVEL,
)
logger = DebugLogger("Pioneer3AT", DEBUG_LEVEL)
start_time = robot.getTime()
logger.debug(
    f"Robot initialised. Starting exploration in {START_DELAY_SECONDS} seconds..."
)
while robot.step(timestep) != -1:
    elapsed_time = robot.getTime() - start_time
    if elapsed_time < START_DELAY_SECONDS:
        wheels.stop()
        continue
    explorer.update()
    snapshot = explorer.snapshot()
    display.update(
        DisplayState(
            grid=snapshot.grid,
            visited=snapshot.visited,
            robot_position=snapshot.robot_position,
            robot_direction=snapshot.robot_direction,
            path=snapshot.path,
            explorer_state=snapshot.state.name,
            target_position=snapshot.target_position,
            navigation_command=(
                None
                if snapshot.navigation_phase is None
                else snapshot.navigation_phase.name
            ),
        )
    )
