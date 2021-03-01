"""pythontroller controller."""
import time

from controller import Robot
from BotClass import Dez

# create the Robot instance.
robot = Dez()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#slightly different for the other robot
#argument is simply the distance
robot.init()
robot.uturn()



while robot.step(timestep) != -1:
    #use for repeated things
    #may need changing for the cooperation of the robots








    pass

# Enter here exit cleanup code.
