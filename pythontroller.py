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
# print(robot.getGPS())
# robot.init()











while robot.step(timestep) != -1:

    #use for repeated things
    #may need changing for the cooperation of the robots
    #self note of home positions: Dez : (6,10) ish, Troy: (18,9)ish
    robot.detect_collision()
    robot.step(500)
    # if robot.getTime() > 270:
    #     if robot.name == "Dez":
    #         robot.goto((15,20))
    #         break
    #     elif robot.name ==  "Troy":
    #         robot.goto((30,22))
    #         break
    # elif robot.sweep() and robot.name == "Dez":
    #     robot.goto((12, 20), heuristic="acf")
    #     robot.face(90)
    #     break
    # elif robot.sweep() and robot.name == "Troy":
    #     robot.goto((31, 20), heuristic="acf")
    #     robot.face(270)
    #     break









# Enter here exit cleanup code.




