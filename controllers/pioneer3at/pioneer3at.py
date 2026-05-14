from controller import Robot
from utils import KeyboardController, getCamera

robot = Robot()
timestep = int(robot.getBasicTimeStep())
camera = getCamera(robot)
keyboardController = KeyboardController(robot, drive_speed=3.0, turn_speed=2.0)

while robot.step(timestep) != -1:
    keyboardController.update()
