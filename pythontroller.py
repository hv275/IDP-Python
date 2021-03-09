"""pythontroller controller."""
import time

from controller import Robot
from BotClass import Dez

# create the Robot instance.
robot = Dez()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
#get the map ready
robot.initialise_map()
#ensures all the sensors are enabled to avoid errors
robot.step(1000)
robot.init()







while robot.step(timestep) != -1:
    #use for repeated things
    #may need changing for the cooperation of the robots
    #self note:
    if robot.sweep() and robot.name == "Dez":
        robot.goto((6,10))
        robot.face(90)
        break
    elif robot.sweep() and robot.name == "Troy":
        robot.goto((18,9))
        robot.face(270)
        break




# Enter here exit cleanup code.




