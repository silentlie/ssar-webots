from camera import CameraController
from controller import Robot
from displayController import DisplayController
from explorer import Explorer
from gridMap import GridMap
from navigation import Navigation
from odometry import Odometry
from sensors import Sensors
from wheels import Wheels

TILE_SIZE = 1
DEBUG = False
START_DELAY_SECONDS = 1.0

robot = Robot()
timestep = int(robot.getBasicTimeStep())
sensors = Sensors(robot, debug=DEBUG)
camera = CameraController(robot)
wheels = Wheels(robot, default_speed=6, default_turn_speed=1)
odometry = Odometry(robot, TILE_SIZE, debug=DEBUG)
gridMap = GridMap()
navigation = Navigation(wheels, odometry, gridMap, sensors, debug=DEBUG)
display = DisplayController(robot, debug=DEBUG)
explorer = Explorer(sensors, gridMap, navigation, display, debug=DEBUG)

start_time = robot.getTime()
if DEBUG:
    print(f"[Main] Waiting {START_DELAY_SECONDS} seconds before starting exploration...")
while robot.step(timestep) != -1:
    elapsed_time = robot.getTime() - start_time
    if elapsed_time < START_DELAY_SECONDS:
        wheels.stop()
        continue
    explorer.update()
