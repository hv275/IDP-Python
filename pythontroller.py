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
    #sweep will not be operational and will only be using dummy variables until I get the model
    robot.sweep()




# Enter here exit cleanup code.




