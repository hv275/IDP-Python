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






while robot.step(timestep) != -1:
    #use for repeated things
    #may need changing for the cooperation of the robots

    #sweep will not be operational and will only be using dummy variables until I get the model








    pass

# Enter here exit cleanup code.
