# import all the needed modules
# Robot allows interaction with Webots
# maths for mathematical operations
# map is a custom map class
# struct is used in communication
from controller import Robot
import math
from Map import gridmap
import struct


# create a class that inherits from the WeBots "Robot" class
class Dez(Robot):
    # constructor
    def __init__(self):
        # inherit from Robot
        super().__init__()

        # initialise robot constants
        # defualt time step for sensors
        self.stepInt = 16
        # robot name
        self.name = self.getName()
        # tracker for sweep
        self.direc = "n"
        # default speed
        self.defaultSpeed = 6
        # map assosiated with the robot
        self.gridMap = None
        # size of a single square on the map
        self.gridSquare = 1
        # last path taken via the GPS system
        self.lastPath = None
        # wheel radius for movement
        self.wheelRad = 0.05
        # queue for communications
        self.queue = []

        # code below initalises wheels based on proto names
        self.wheels = []
        wheelsNames = ['l_wheel', 'r_wheel']
        for i in range(len(wheelsNames)):
            self.wheels.append(self.getDevice(wheelsNames[i]))
            self.wheels[i].setPosition(float("inf"))
            self.wheels[i].setVelocity(0.0)

        # code below initialises the claw motor based on proto names
        self.claw = self.getDevice("arm")
        self.claw.setPosition(float("inf"))
        self.claw.setVelocity(0)

        # code below intitalises distance sensors. Three were used
        self.distsensors = []
        sensornames = ["distance_sensor_front", "distance_sensor_centre", "distance_sensor_back"]
        for i in range(len(sensornames)):
            self.distsensors.append(self.getDevice(sensornames[i]))
            self.distsensors[i].enable(self.stepInt)

        # initialise GPS
        self.gps = self.getDevice("gps")
        self.gps.enable(self.stepInt)
        # initialise compass
        self.comp = self.getDevice("compass")
        self.comp.enable(self.stepInt)

        # initialise light sensors
        # multiple light sensors used to mimic what what we would expect in real life
        # lists store names then a loop goes through the names and appends WeBots sensor objects to the new list
        self.greenlightsensors_names = ["light_sensor_green", "light_sensor_green(1)", "light_sensor_green(2)",
                                        "light_sensor_green(3)",
                                        "light_sensor_green(4)", "light_sensor_green(5)", "light_sensor_green(6)",
                                        "light_sensor_green(7)"]
        self.redlightsensors_names = ["light_sensor_red", "light_sensor_red(1)", "light_sensor_red(2)",
                                      "light_sensor_red(3)", "light_sensor_red(4)", "light_sensor_red(5)",
                                      "light_sensor_red(6)", "light_sensor_red(7)"]
        self.redlightsensors = []
        self.greenlightsensors = []
        for i in range(len(self.redlightsensors_names)):
            self.redlightsensors.append(self.getDevice(self.redlightsensors_names[i]))
            self.redlightsensors[i].enable(self.stepInt)
        for i in range(len(self.greenlightsensors_names)):
            self.greenlightsensors.append(self.getDevice(self.greenlightsensors_names[i]))
            self.greenlightsensors[i].enable(self.stepInt)

        # separate sensor to measure ambient light
        self.ambsensor = self.getDevice("ambient_sensor")
        self.ambsensor.enable(self.stepInt)

        # emitter and receiver initialisation
        self.emitter = self.getDevice("emitter")
        self.receiver = self.getDevice("receiver")
        self.receiver.enable(self.stepInt)

        # setting the communication channels
        if self.name == "Dez":
            self.emitter.setChannel(1)
            self.receiver.setChannel(2)
        elif self.name == "Troy":
            self.emitter.setChannel(2)
            self.receiver.setChannel(1)

        # arrays for storing recieved co-ordinates
        self.queue_dez = []
        self.queue_troy = []

        # counter for number of sweeps conducted
        self.sweepCounter = 0

    def getDist(self):
        """method to obtain readings from distance sensors
            returns an array containing front sensor data left to right"""
        out = []
        for i in self.distsensors:
            out.append(i.getValue())
        return out

    def getBearing(self):
        """method to obtain the bearing
            uses the values output by the compass
            and converts them to bearing using some maths"""
        north = self.comp.getValues()
        rad = math.atan2(north[0], north[2])
        bearing = (rad - 1.5708) / math.pi * 180
        if bearing < 0:
            bearing += 360

        return bearing

    def getGPS(self):
        """method that returns the x and y coordinates of the robot as a list"""
        coords = self.gps.getValues()[0::2]

        # rounds the coordinates to the nearest grid square
        for i in enumerate(coords):
            #
            coords[i[0]] = abs(i[1] * 20 - (i[1] * 20) % self.gridSquare)
        # webots was outputting coordinates weirdly so needed to reverse them
        coords.reverse()
        return coords

    def moveArmDown(self):
        """method that actuates the arm motor for 2s in order to lower the claw"""
        end_time = self.getTime() + 2
        while self.step(32) != 1:
            if self.getTime() < end_time:
                self.claw.setVelocity(0.2)

            else:
                self.claw.setVelocity(0)
                break

    def moveArmUp(self):
        """method that actuates the arm motor for 2 seconds in order to raise the claw"""
        end_time = self.getTime() + 2
        while self.step(32) != 1:
            if self.getTime() < end_time:
                self.claw.setVelocity(-0.2)
            else:
                self.claw.setVelocity(0)
                break

    def moveForward(self, dist, vel=None):
        """method to move the robot forward
            utitises the dimensions of the robot in order to
            determine for how long to actuate the
            takes in distance to travel and velocity as arguments"""
        if vel == None:
            # if no velocity is passed into the method, set it to default
            vel = self.defaultSpeed

        end_time = self.getTime() + (dist / self.wheelRad) / vel
        while self.step(32) != -1:
            # collision protection
            self.detect_collision()
            # until the finish time is reached, actuate the motors, then stop
            if self.getTime() < end_time:
                for i in self.wheels:
                    i.setVelocity(vel)

            else:
                for i in self.wheels:
                    i.setVelocity(0)
                break

    def moveBack(self, dist, vel=None):
        """this method is a carbon copy of the moveForward method
            difference is that the velocity set to negative
            in order to reverse"""
        if vel == None:
            vel = self.defaultSpeed
        end_time = self.getTime() + (dist / self.wheelRad) / vel
        while self.step(32) != -1:
            self.detect_collision()
            if self.getTime() < end_time:
                for i in self.wheels:
                    i.setVelocity(-vel)

            else:
                for i in self.wheels:
                    i.setVelocity(0)
                break

    def rightTurnCompass(self, angle=90, vel=None):
        """method to perform a right turn
            takes in the angle to turn through and velocity as arguments"""

        # take the starting bearing
        start = round(self.getBearing()) % 360
        # calculate the end bearing
        end = (start + angle) % 360

        if vel == None:
            # set the turn velocity to a low value to ensure accuracy
            # unless otherwise stated
            vel = 0.95

        # time is used to break robots from infinite turning loops
        start_time = self.getTime()
        while self.step(16) != 1:
            # take the bearing to the nearest degree
            bearing = round(self.getBearing()) % 360
            # if time is over 30 i.e. robot is stuck, raise the claw and move back and turn to face north
            if self.getTime() > start_time + 30:
                self.moveArmUp()
                self.moveBack(0.05)
                self.moveArmDown()
                self.face(0)
                break

            # turn until the final bearing is reached
            # be careful as at high speeds, the robot may miss the final bearing and keep spinning
            if bearing != end:
                self.wheels[0].setVelocity(vel)
                self.wheels[1].setVelocity(-vel)

            else:
                for i in self.wheels:
                    i.setVelocity(0)
                break

    def leftTurnCompass(self, angle=90, vel=None):
        """same method as rightTurnCompass, but velocities are equal and opposite
            so that the robot rotates anticlockwise"""
        start = round(self.getBearing()) % 360
        start_time = self.getTime()
        end = (start + 360 - angle) % 360

        if vel == None:
            # artificially
            vel = 0.95

        while self.step(1) != 1:
            bearing = round(self.getBearing()) % 360
            if self.getTime() > start_time + 30:
                self.moveArmUp()
                self.moveBack(0.05)
                self.moveArmDown()
                self.face(180)
                break

            if bearing != end:
                self.wheels[0].setVelocity(-vel)
                self.wheels[1].setVelocity(+vel)


            else:
                for i in self.wheels:
                    i.setVelocity(0)
                break

    def init(self):
        """method to initialise the robot at the start
            moves the robot to its starting position"""
        self.moveForward(0.1)
        self.leftTurnCompass(angle=45)
        self.moveForward(0.3)
        self.leftTurnCompass(angle=45)
        print("Initialisation complete")

    def uturn(self, dist=0.1535):
        """method to uturn when neccesary, i.e. at the wall"""

        # profilactic stop
        for i in self.wheels:
            i.setVelocity(0)

        # depending on current direction, turn either clockwise or anticlockwise
        if self.direc == "n":
            self.leftTurnCompass()
            self.moveForward(dist)
            self.leftTurnCompass()
            self.direc = "s"
        elif self.direc == "s":
            self.rightTurnCompass()
            self.moveForward(dist)
            self.rightTurnCompass()
            self.direc = "n"

    def isBlock(self):
        """method to determine if the robot is detecting a block
            returns True or False based on the the detected colour
            True = box of correct colour
            False = no colours detected or box of opposite colour"""

        # move toward the block, cover it with the arm and actuate
        # light movement is done to move the block close to the claw mounted sensors
        self.moveArmDown()
        self.moveForward(0.13)
        self.leftTurnCompass(30)
        # definition of the ambient light values based on fairly extensive preliminary measurements
        ambient_light = 333.8

        # loop throught the sensors
        for i in self.redlightsensors:
            for j in self.greenlightsensors:
                # if the colour is above both ambient light and opposite colour levels
                # and aligns with the robot colour then return true
                # otherwise return true
                # also print detected value and the block colour detected
                if i.getValue() > j.getValue() and i.getValue() > ambient_light and self.name == "Dez":
                    print(f"Red: {i.getValue()}")
                    print(f"Green: {j.getValue()}")
                    print(f"Ambient: {self.ambsensor.getValue()}")
                    self.rightTurnCompass(30)
                    self.moveBack(0.1)
                    self.moveArmUp()
                    print("Red Block Detected, attempting delivery")
                    return True

                if j.getValue() > i.getValue() and j.getValue() > ambient_light and self.name == "Troy":
                    print(f"Red: {i.getValue()}")
                    print(f"Green: {j.getValue()}")
                    print(f"Ambient: {self.ambsensor.getValue()}")
                    self.rightTurnCompass(30)
                    self.moveBack(0.1)
                    self.moveArmUp()
                    print("Green Block Detected, attempting delivery")
                    return True

                elif j.getValue() > i.getValue() and j.getValue() > ambient_light and self.name == "Dez":
                    print(f"Red: {i.getValue()}")
                    print(f"Green: {j.getValue()}")
                    print(f"Ambient: {self.ambsensor.getValue()}")
                    self.rightTurnCompass(30)
                    self.moveBack(0.1)
                    self.moveArmUp()
                    print("Green Block Detected, location info sent")
                    return False

                elif i.getValue() > j.getValue() and i.getValue() > ambient_light and self.name == "Troy":
                    print(f"Red: {i.getValue()}")
                    print(f"Green: {j.getValue()}")
                    print(f"Ambient: {self.ambsensor.getValue()}")
                    self.rightTurnCompass(30)
                    self.moveBack(0.1)
                    self.moveArmUp()
                    print("Red Block Detected, location info sent")
                    return False

                # if nothing, continue through the list
                else:
                    continue
        # if nothing detected, then move around just in case and proceed
        self.rightTurnCompass(30)
        self.moveBack(0.1)
        self.moveArmUp()
        return False

    '''left until a better time'''

    def transmit_data(self):
        """method to transmit the location data to the other robot"""
        # get the coordinates and get the individual values
        coords = self.getGPS()
        x = coords[0]
        y = coords[1]
        # get the bearing
        bearing = round(self.getBearing())
        # package the data into a C like struct
        data = struct.pack('ffi', x, y, bearing)
        # send the data and wait one timestep for it to send
        self.emitter.send(data)
        self.step(self.stepInt)

    def receive_data(self):
        """method to recieve any data that could have been sent"""
        # wait one timestep to see if any data has been sent down the channel
        self.step(self.stepInt)
        # blank list to store future values
        received_coords = []
        for i in range(3):
            # if data is present in the receiver queue then get the struct and upack it
            # move onto the next value
            if self.receiver.getQueueLength() > 0:
                rec_data = self.receiver.getData()
                received_data = struct.unpack("ffi", rec_data)
                received_coords.append(received_data)
                self.receiver.nextPacket()

        # append the data to the current data queue of the respective robot
        # and print an acknowledgement that the data was received
        if self.name == "Dez":
            self.queue_dez.append(received_coords)
            print("Current queue for ", self.name, " is: ", self.queue_dez)
        else:
            self.queue_troy.append(received_coords)
            print("Current queue for ", self.name, " is: ", self.queue_troy)

    def detect_collision(self):
        """method to check if a collision is possible
            it is intented to be called before any robot moves forward
            Note: Dez is master, Troy is slave in the current config"""
        if self.name == "Dez":
            # Dez to send the data of its current location
            self.transmit_data()
        if self.name == "Troy":
            # troy to wait a single timestep to wait for data to receive
            self.step(self.stepInt)
            # if any data was recieved then unpack it
            if self.receiver.getQueueLength() > 0:
                rec_data = self.receiver.getData()
                received_data = struct.unpack("ffi", rec_data)
                self.receiver.nextPacket()
            # if no data was received then return a list of zeros
            else:
                received_data = [0, 0, 0]
            # pull the coordinates of each robot
            dez_coords = received_data[:2]
            troy_coords = self.getGPS()[:2]

            # calculate the straight line distance between the two robots
            distance = ((dez_coords[0] - troy_coords[0]) ** 2 + (dez_coords[1] - troy_coords[1]) ** 2) ** (1 / 2)
            # return true if robots are nearby
            # this a primitive but a functional way to implement collision detection
            # the map class could in the future be expanded to remove edges or nodes from the graph representation of the map
            # this would not allow robots to plot paths through the same space
            if distance < 10 and self.name == "Troy":
                self.stop()
                return True
            else:
                return False

    def avoid_collision(self):
        """method that implements basic collision protection"""
        # stop troy if robots are near
        # primitive function but was surprisingly effective
        if self.name == "Troy":
            return 1
        else:
            pass

    def sweep(self):
        """ method that implements the main sweep routing, makes the robot move forward
            until distance sensors detect an obstacle
            this is one of the more complex functions
            see line comments for detailed breakdown"""

        # correct the bearing in case the robot has weired of the straigh path
        self.correctBearing()

        # start a loop to handle reading sensors in motion
        while self.step(16) != -1:
            # if any of the sensors detects an obtastacle fairly far begin the checks
            if self.getDist()[0] > 580 or self.getDist()[1] > 580 or self.getDist()[2] > 580:
                # if close to the top or the bottom of the arena then it is most likely a wall and it is safe to turn
                # around
                if self.getGPS()[1] > 40 or self.getGPS()[1] < 3:
                    # incerement the sweep path counter
                    self.sweepCounter += 1
                    # code below is based on the individual starting paths
                    # upon filling the sweep counter, the method returns 1
                    # that lets the controller know that the sweep finished and it possible to return to base
                    # an appropriate message is also printed into the console
                    # if sweep counter has not reached the require value, turn around and end the loop
                    # thus allowing it to be called again from the controller
                    if self.sweepCounter == 8 and self.name == "Troy":
                        print("Troy finished sweep")
                        return 1
                    if self.sweepCounter == 8 and self.name == "Dez":
                        self.uturn(0.1)
                        break
                    if self.sweepCounter == 9 and self.name == "Dez" or self.name == "Dez" and self.getGPS()[0] < 3 and \
                            self.getGPS()[1] > 42:
                        print("Dez finished sweep")
                        return 1

                    self.uturn()
                    break

                # if location is not near the top of bottom of the arena, it is most likely a block
                # if one of the side sensors picks it up, rotate a bit
                # the rotation ensure a smoother capture with the claw
                if self.getDist()[1] > 850 or self.getDist()[0] > 850 or self.getDist()[2] > 850:
                    if self.getDist()[0] > 850 and self.getDist()[1] < 100:
                        self.leftTurnCompass(25)
                    if self.getDist()[2] > 850 and self.getDist()[1] < 100:
                        self.rightTurnCompass(25)

                    # runs the above is block the method
                    # if the method returns 1 then the block will be covered and delivier to base
                    if self.isBlock():
                        # stop the wheel motors just in case
                        self.stop()
                        # lower the arm
                        self.moveArmDown()
                        # correct the bearing in case the angle is off
                        self.correctBearing()
                        # depending on the name of the robot deliver the block the respective block
                        if self.name == "Dez":
                            print("Dez returning")
                            self.goto((11, 23), heuristic="acf")
                            self.face(90)
                            # this moving about is to ensure that the blocks are less likely to be pushed out
                            self.moveForward(0.25)
                            self.moveArmUp()
                            self.moveBack(0.25)
                            # the robot retraces its steps to the point it picked up the block
                            self.returnToPoint()
                            # continue with the sweep loop
                            continue
                        elif self.name == "Troy":
                            self.goto((35, 22))
                            self.face(270)
                            self.moveForward(0.25)
                            self.moveArmUp()
                            self.moveBack(0.25)
                            self.returnToPoint()
                            continue
                    else:
                        # if the block is incorrect colour, drive around and continue
                        # with speed and mobility improvements block location can be sent to the other robot
                        self.correctBearing()
                        print("bypassing")
                        self.bypassBlock()
                        continue
                # break out of the loop
                break
            # may need some coordinate adjustment
            # check for collisions
            if self.detect_collision() and self.name == "Troy":
                self.stop()
            else:
                for i in self.wheels:
                    i.setVelocity(self.defaultSpeed)

    def initialise_map(self):
        """method that initialises the map"""
        # I initialised the map outside of constructor to ease debugging
        # map is a different class
        self.gridMap = gridmap(45, 45, self.gridSquare)

    def goto(self, dest, heuristic="acf"):
        """method to handle going to a certain map square
            takes in destination and the navigation heuristic as arguments
            default heuristic is so called 'as crow flies' as manhattan resulted in some weird behaivour """

        # convert coordinates to tuples from arrays as that is what the navigation method requires (in different class)
        # also coordinates are quantised to the nearest grid square
        start = tuple([math.floor(i) for i in self.getGPS()])
        end = tuple([round(i - i % self.gridSquare) for i in dest])

        # get the route using A* pathfinding
        route = self.gridMap.directions(start, end, heuristic)
        # save the path the robot took so that return is easy
        # do the same with the bearing so it can face the right way
        self.lastPath = route
        self.lastAngle = round(self.getBearing()) % 360
        # for each direction in the list, face that way and move one grid square
        for dir in route:
            self.face(dir)
            self.face(dir)
            # move is a bit larger than the square size to account for the tiny amount of wheel slip
            self.moveForward(0.06)

    def returnToPoint(self):
        """method to handle returning to the last visited point based on the saved data from 'goto' method
            essentially reverses the directions makes the robot take them in the opposite way"""
        route = self.lastPath
        for i in range(len(route)):
            # make positive directions negative and vice versa
            route[i] = list(route[i])
            route[i][0] -= route[i][0] * 2
            route[i][1] -= route[i][1] * 2
            route[i] = tuple(route[i])
        # reverse the list
        route.reverse()
        # follow the directions back
        for dir in route:
            self.face(dir)
            self.moveForward(0.06)
            # if the time is getting late, abandon the path and then return to base
            if self.getTime() > 245:
                self.moveBack(0.2)
                break
        #face the last angle faced
        self.face(self.lastAngle)

    def face(self, direc):
        """method to handle the robot facing the correct direction
            takes in a direction as an argument
            direction can either be a tuple from the pathfinding algorithm or a bearing you want to face"""

        # set default velocity for turns
        vel = 1

        # library to look up bearing based on the direction given by the pathfinding
        dirToAngle = {(0, 1): 0, (0, -1): 180, (1, 0): 90, (-1, 0): 270}

        # get the current bearing to the nearest degree
        bearing = round(self.getBearing()) % 360
        # if the input is a tuple, look it up in the array
        if type(direc) == tuple:
            targetAngle = dirToAngle[direc]
        # if the input is a bearing, round it to the nearest degree
        else:
            targetAngle = round(direc) % 360

        # a clever way to figure out the length of the turning arc to ensure that the robot never turns through more
        # than 180 degrees
        if (360 + (targetAngle - bearing)) % 360 >= 180:
            # rotate clockwise, same code as in the turn function
            # I wanted a clean rotation code here so that if the main turn function is changed, this one is intact
            while self.step(16) != 1:
                bearing = round(self.getBearing()) % 360

                if bearing != targetAngle:
                    self.wheels[0].setVelocity(-vel)
                    self.wheels[1].setVelocity(vel)

                else:
                    for i in self.wheels:
                        i.setVelocity(0)
                    break
        else:
            while self.step(16) != 1:
                bearing = round(self.getBearing()) % 360
                if bearing != targetAngle:
                    self.wheels[0].setVelocity(vel)
                    self.wheels[1].setVelocity(-vel)

                else:
                    for i in self.wheels:
                        i.setVelocity(0)
                    break

    def bypassBlock(self):
        # basic code to bypass the block by turning and moving around it
        # all the time checks are to ensure the robot does not get stuck in this subroutine for way too long
        # the bearing check is to help with wall
        if round(self.getBearing()) == 180:
            self.moveBack(0.05)
            if self.getTime() > 220:
                return
            self.leftTurnCompass(angle=90)
            if self.getTime() > 220:
                return
            self.moveForward(0.2)
            if self.getTime() > 220:
                return
            self.rightTurnCompass(angle=90)
            if self.getTime() > 220:
                return
            self.moveForward(0.15)
            if self.getTime() > 220:
                return
            self.rightTurnCompass(angle=90)
            if self.getTime() > 220:
                return
            self.moveForward(0.2)
            if self.getTime() > 220:
                return
            self.leftTurnCompass(60)
            if self.getTime() > 220:
                return
            self.correctBearing()
        else:
            self.moveBack(0.05)
            if self.getTime() > 220:
                return
            self.leftTurnCompass(angle=90)
            if self.getTime() > 220:
                return
            self.moveForward(0.2)
            if self.getTime() > 220:
                return
            self.rightTurnCompass(angle=90)
            if self.getTime() > 220:
                return
            self.moveForward(0.15)
            if self.getTime() > 220:
                return
            self.rightTurnCompass(angle=90)
            if self.getTime() > 220:
                return
            self.moveForward(0.2)
            if self.getTime() > 220:
                return
            self.leftTurnCompass(90)
            if self.getTime() > 220:
                return
            self.correctBearing()

    def stop(self):
        """just a quality of life method as I got tired from stopping everything manually"""
        for i in self.wheels:
            i.setVelocity(0)

    def correctBearing(self, base=90):
        """method to correct bearing
            takes the nearest factor as an argument (90 by default) and adjusts the bearing to that
            done mainly to counteract the random weir that the robot occasionally did"""
        current = round(self.getBearing())
        goal = base * round(current / base)
        self.face(goal)
