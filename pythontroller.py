"""pythontroller controller."""
"""the controller is identical for both robot instances, the difference is dintinquised by the name in webots
    dez is red, troy is green"""

# import the custom class
from BotClass import Dez

# create the Robot object
robot = Dez()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
# get the map ready
robot.initialise_map()
# wait to ensure all the sensors are recording
robot.step(1000)
# get the robots into position in order to begin the sweep
robot.init()

while robot.step(timestep) != -1:

    # a chuck of code that sends robots back to base in case they are out for too long
    if robot.getTime() > 220:
        # depending on the name, return to the neccessary position
        if robot.name == "Dez":
            robot.goto((15, 24))
            break
        elif robot.name == "Troy":
            robot.goto((28, 22))
            break
    # when sweep finishes, it returns True
    # when that happens, the robots will return
    elif robot.sweep() and robot.name == "Dez":
        robot.goto((12, 19), heuristic="acf")
        robot.face(90)
        break
    elif robot.sweep() and robot.name == "Troy":
        robot.goto((32, 19), heuristic="acf")
        robot.face(270)
        break

# Enter here exit cleanup code.
