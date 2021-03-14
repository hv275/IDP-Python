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
robot.init()











while robot.step(timestep) != -1:

    #use for repeated things
    #may need changing for the cooperation of the robots
    #self note of home positions: Dez : (6,10) ish, Troy: (18,9)ish

    if robot.sweep() and robot.name == "Dez":
        robot.goto((12, 22), heuristic="acf")
        robot.face(90)
        break
    elif robot.sweep() and robot.name == "Troy":
        robot.goto((62, 24), heuristic="acf")
        robot.face(270)
        break
    elif robot.getTime() == 270:
        if robot.name == "Dez":
            robot.goto((14,20))
        elif robot.name ==  "Troy":
            robot.goto((30,24))








# Enter here exit cleanup code.




