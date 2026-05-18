from controller import Robot

from algorithm import ExplorationStrategy
from utils import Wheels

robot = Robot()
timestep = int(robot.getBasicTimeStep())
wheels = Wheels(robot, max_speed=3.0)
explorer = ExplorationStrategy(robot, wheels, timestep)

while robot.step(timestep) != -1:
    explorer.update()
