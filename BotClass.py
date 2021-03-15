from controller import Robot
import math
from Map import gridmap
import numpy as np
import random
import struct


class Dez(Robot):
    def __init__(self):
        # inherit from Robot
        super().__init__()

        # robot consts
        # not all may be needed
        self.stepInt = 32
        self.name = self.getName()
        self.direc = "n"
        self.defaultSpeed = 6
        self.gridMap = None
        self.gridSquare = 1
        self.lastPath = None
        print(self.name)
        self.wheelRad = 0.05
        self.queue = []

        # inititialise wheels
        self.wheels = []
        wheelsNames = ['l_wheel', 'r_wheel']
        for i in range(len(wheelsNames)):
            self.wheels.append(self.getDevice(wheelsNames[i]))
            self.wheels[i].setPosition(float("inf"))
            self.wheels[i].setVelocity(0.0)

        # initialise the claw
        self.claw = self.getDevice("arm")
        self.claw.setPosition(float("inf"))
        self.claw.setVelocity(0)

        self.distsensors = []
        sensornames = ["distance_sensor_front","distance_sensor_centre",  "distance_sensor_back"]
        for i in range(len(sensornames)):
            self.distsensors.append(self.getDevice(sensornames[i]))
            self.distsensors[i].enable(self.stepInt)

        self.gps = self.getDevice("gps")
        self.gps.enable(self.stepInt)
        self.frontDist = self.getDevice("distance_sensor_front")
        self.frontDist.enable(self.stepInt)
        self.backDist = self.getDevice("distance_sensor_back")
        self.backDist.enable(self.stepInt)
        self.comp = self.getDevice("compass")
        self.comp.enable(self.stepInt)

        # light sensors
        self.redlight = self.getDevice("light_sensor_red")
        self.redlight.enable(self.stepInt)

        self.greenlight = self.getDevice("light_sensor_green")
        self.greenlight.enable(self.stepInt)

        # emitter and receiver
        self.emitter = self.getDevice("emitter")
        self.receiver = self.getDevice("receiver")
        self.receiver.enable(self.stepInt)

        if self.name == "Dez":
            self.emitter.setChannel(1)
            self.receiver.setChannel(2)
        elif self.name == "Troy":
            self.emitter.setChannel(2)
            self.receiver.setChannel(1)

        # array for storing recieved co-ordinates
        self.queue_dez = []
        self.queue_troy = []

        self.sweepCounter = 0


    def getDist(self):
        out = []
        for i in self.distsensors:
            out.append(i.getValue())
        return out

    def getBearing(self):
        north = self.comp.getValues()
        rad = math.atan2(north[0], north[2])
        bearing = (rad - 1.5708) / math.pi * 180
        if bearing < 0:
            bearing += 360


        return bearing

    def getGPS(self):
        # returns the useful part of the array
        # also the coordinate system is a mess
        # it is very dumb but should work
        coords = self.gps.getValues()[0::2]

        for i in enumerate(coords):
            #
            coords[i[0]] = abs(i[1] * 20 - (i[1] * 20) % self.gridSquare)
        # in place modification
        # this is dumb but I do not have the time for a cleaner fix
        coords.reverse()
        return coords

    def moveArmDown(self):
        end_time = self.getTime() + 1.8
        while self.step(32) != 1:
            if self.getTime() < end_time:
                self.claw.setVelocity(0.2)

            else:
                self.claw.setVelocity(0)
                break

    def moveArmUp(self):
        end_time = self.getTime() + 1.8
        while self.step(32) != 1:
            if self.getTime() < end_time:
                self.claw.setVelocity(-0.2)
            else:
                self.claw.setVelocity(0)
                break

    def moveForward(self, dist, vel=None):
        if vel == None:
            vel = self.defaultSpeed
        end_time = self.getTime() + (dist / self.wheelRad) / vel
        while self.step(32) != -1:
            if self.getTime() < end_time:
                for i in self.wheels:
                    i.setVelocity(vel)

            else:
                for i in self.wheels:
                    i.setVelocity(0)
                break

    def moveBack(self, dist, vel=None):
        if vel == None:
            vel = self.defaultSpeed
        end_time = self.getTime() + (dist / self.wheelRad) / vel
        while self.step(32) != -1:
            if self.getTime() < end_time:
                for i in self.wheels:
                    i.setVelocity(-vel)

            else:
                for i in self.wheels:
                    i.setVelocity(0)
                break

    '''redundant fuctions, no longer need but left just in case'''

    # def rightTurn(self, vel=None, angle=90):
    #     # configured for 90 degrees only
    #     # issues with slip during simulation, hard to precisely obtain turning times
    #     # numbers have been simply tunes for 90 deg turn at 3rad/s
    #     # can make into other values once we get the compass on
    #     angle = math.radians(angle)
    #     arc = angle * 0.12 * 1.578818
    #
    #     if vel == None:
    #         vel = self.defaultSpeed
    #        #     end_time = self.getTime() + (arc / self.wheelRad) / vel
    #     while self.step(16) != -1:
    #         if self.getTime() < end_time:
    #             self.wheels[0].setVelocity(vel)
    #             self.wheels[1].setVelocity(-vel)
    #         else:
    #             for i in self.wheels:
    #                 i.setVelocity(0)
    #             break
    #
    # def leftTurn(self, vel=None, angle=90):
    #     # turn left
    #     angle = math.radians(angle)
    #     arc = angle * 0.12 * 1.578818
    #
    #     if vel == None:
    #         vel = self.defaultSpeed
    #
    #     end_time = self.getTime() + (arc / self.wheelRad) / vel
    #     while self.step(16) != -1:
    #         if self.getTime() < end_time:
    #             self.wheels[0].setVelocity(-vel)
    #             self.wheels[1].setVelocity(vel)
    #         else:
    #             for i in self.wheels:
    #                 i.setVelocity(0)
    #             break

    def rightTurnCompass(self, angle=90, vel=None):
        # it may go around more than once
        # that is fine, I do not have the time to properly fix it
        start = round(self.getBearing()) % 360

        end = (start + angle) % 360

        if vel == None:
            # am artifically slowing it down
            vel = 1

        while self.step(16) != 1:
            bearing = round(self.getBearing()) % 360

            if bearing != end:
                self.wheels[0].setVelocity(vel)
                self.wheels[1].setVelocity(-vel)

            else:
                for i in self.wheels:
                    i.setVelocity(0)
                break

    def leftTurnCompass(self, angle=90, vel=None):
        # it may go around more than once
        # that is fine, I do not have the time to properly fix it
        start = round(self.getBearing()) % 360

        end = (start + 360 - angle) % 360

        if vel == None:
            # artificially
            vel = 1

        while self.step(1) != 1:
            bearing = round(self.getBearing()) % 360

            if bearing != end:
                self.wheels[0].setVelocity(-vel)
                self.wheels[1].setVelocity(+vel)

            else:
                for i in self.wheels:
                    i.setVelocity(0)
                break

    # def moveForwardEncoded(self,dist):
    #     lpos = self.leftcoder.getValue()
    #     rpos = self.rightcoder.getValue()
    #     lposfin = lpos + (dist / self.wheelRad)
    #     rposfin = rpos + (dist / self.wheelRad)
    #     while self.step(16) != -1:
    #         if self.leftcoder.getValue() < lposfin and self.rightcoder.getValue() < rposfin:
    #                 self.wheels[0].setVelocity(self.defaultSpeed)
    #                 self.wheels[1].setVelocity(self.defaultSpeed*2)
    #         else:
    #             for i in self.wheels:
    #                 i.setVelocity(0)
    #             break

    def init(self):
        # initaliser function - only run once at the start to set up
        # initaliser function - only run once at the start to set up
        self.moveForward(0.1)
        self.leftTurnCompass(angle=45)
        self.moveForward(0.3)
        self.leftTurnCompass(angle=45)
        print("Initialisation complete")

    def uturn(self, dist = 0.1535):
        for i in self.wheels:
            i.setVelocity(0)
        # warning - this is valid for both robots (kinda)
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
        # code to check if it is a block
        # probably just check for led with camera
        self.moveArmDown()
        self.moveForward(0.03)
        self.leftTurnCompass(30)
        # get coloured components of light
        red_light = self.redlight.getValue()
        # blue or green?
        green_light = self.greenlight.getValue()
        ambient_light = 333.4

        if red_light > ambient_light and self.name == "Dez":
            self.rightTurnCompass(30)
            self.moveBack(0.03)
            self.moveArmUp()
            print("Red Block Detected, attempting delivery")
            return True

        if green_light > ambient_light and self.name == "Troy":
            self.rightTurnCompass(30)
            self.moveBack(0.03)
            self.moveArmUp()
            print("Green Block Detected, attempting delivery")
            return True

        else:
            self.rightTurnCompass(20)
            self.moveBack(0.03)
            self.moveArmUp()
            if self.name == "Dez":
                print("Green Colour detected, location information sent")
            elif self.name == "Troy":
                print("Red Colour detected, location information sent")
            return False

    '''left until a better time'''

    def transmit_data(self):
        # transmit data from gps using emitter
        coords = self.getGPS()
        x = coords[0]
        y = coords[1]
        bearing = round(self.getBearing())
        data = struct.pack('ffi', x, y, bearing)
        self.emitter.send(data)
        print("\n", "Data transmitted: ", x, y, bearing)
        self.step(self.stepInt)

    def receive_data(self):
        self.step(self.stepInt)
        received_coords = []
        for i in range(3):
            if self.receiver.getQueueLength() > 0:
                rec_data = self.receiver.getData()
                received_data = struct.unpack("ffi", rec_data)
                received_coords.append(received_data)
                self.receiver.nextPacket()
        print("Data Recieved, block at: ", received_coords)
        if self.name == "Dez":
            self.queue_dez.append(received_coords)
            print("Current queue for ", self.name, " is: ", self.queue_dez)
        else:
            self.queue_troy.append(received_coords)
            print("Current queue for ", self.name, " is: ", self.queue_troy)
        # return received_coords


    def detect_collision(self):
        if self.name == "Dez":
            self.transmit_data()
        if self.name == "Troy":
            self.step(self.stepInt)
            if self.receiver.getQueueLength() > 0:
                rec_data = self.receiver.getData()
                received_data = struct.unpack("ffi", rec_data)
                self.receiver.nextPacket()
            else:
                received_data = [0, 0, 0]
            print(received_data)
            dez_coords = received_data[:2]
            troy_coords = self.getGPS()[:2]
            # print(dez_coords)
            # print(troy_coords)
            # print(len(dez_coords))
            # print(len(troy_coords))

            distance = ((dez_coords[0] - troy_coords[0])**2 + (dez_coords[1] - troy_coords[1])**2 )**(1/2)
            print(distance)
            if distance < 10:
                return True
            else:
                return False

    def avoid_collision(self):
        if self.name == "Troy":
            return 1
        else:
            pass


    def isBlockDummy(self):
        # for now always return 1
        # for further testing can make 50/50
        return 1

    def sweep(self):
        # arbitrary dist - change based on sensor
        self.correctBearing()
        loc = self.getGPS()
        while self.step(16) != -1:
            if self.getDist()[0] > 580 or self.getDist()[1] > 580 or self.getDist()[2] > 580:
                if self.getGPS()[1] > 40 or self.getGPS()[1] < 3:
                    self.sweepCounter += 1
                    print(str(self.name) + str(self.sweepCounter))
                    if self.sweepCounter == 8 and self.name == "Troy":
                        print("Troy finished sweep")
                        return 1
                    if self.sweepCounter == 8 and self.name == "Dez":
                        self.uturn(0.1)
                        print("tiny u turn")
                        break
                    if self.sweepCounter == 9 and self.name == "Dez":
                        print("Dez finished sweep")
                        return 1


                    self.uturn()
                    break
                if self.getDist()[1] > 850 or self.getDist()[0] > 850 or self.getDist()[2] > 850:
                    if self.getDist()[0] > 850 and self.getDist()[1] < 100:
                        self.leftTurnCompass(20)
                    if self.getDist()[2] > 850 and self.getDist()[1] < 100:
                        self.rightTurnCompass(20)
                    if self.isBlock():
                        self.stop()
                        self.moveArmDown()
                        self.correctBearing()
                        if self.name == "Dez":
                            print("Dez returning")
                            self.goto((14, 24), heuristic="acf")
                            self.face(90)
                            self.moveArmUp()
                            self.face(270)
                            self.returnToPoint()
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
                        self.correctBearing()
                        print("bypassing")
                        self.bypassBlock()
                        continue
                break
            # may need some coordinate adjustment
            if self.detect_collision() and self.name == "Troy":
                self.stop()
            else:
                for i in self.wheels:
                    i.setVelocity(self.defaultSpeed)

    def initialise_map(self):
        self.gridMap = gridmap(45, 45, self.gridSquare)

    def goto(self, dest, heuristic="acf"):
        start = tuple([math.floor(i) for i in self.getGPS()])
        end = tuple([round(i - i % self.gridSquare) for i in dest])

        route = self.gridMap.directions(start, end, heuristic)
        self.lastPath = route
        self.lastAngle = round(self.getBearing()) % 360
        for dir in route:
            self.face(dir)
            self.face(dir)
            self.moveForward(0.06)

    def gotoBearing(self, dest):
        "Cool function but currently broken. Do not use!!!!"
        curr = np.array(self.getGPS())
        vec = (np.array(dest) - curr)
        vec = vec / np.linalg.norm(vec)
        north = np.array([0, 1])
        dot = np.dot(north, vec)
        targetAngle = (math.degrees(np.arccos(dot)) + 360) % 360
        bearing = self.getBearing()
        self.face(targetAngle)
        while self.step(self.stepInt) != 1:
            dist = self.getGPS()

    def returnToPoint(self):
        # use this to return to points, please do not use anything else as it will not return to where we expect it to be.
        route = self.lastPath
        for i in range(len(route)):
            route[i] = list(route[i])
            route[i][0] -= route[i][0] * 2
            route[i][1] -= route[i][1] * 2
            route[i] = tuple(route[i])
        route.reverse()
        for dir in route:
            self.face(dir)
            self.moveForward(0.06)
        self.face(self.lastAngle)

    def face(self, direc):
        """Argument has to be a tuple or a bearing"""
        vel = 1
        # dictionary look up to find angles (only works with 90 increments for now
        # I may look into using maths to find the angle but for now this works well

        dirToAngle = {(0, 1): 0, (0, -1): 180, (1, 0): 90, (-1, 0): 270}

        bearing = round(self.getBearing()) % 360
        if type(direc) == tuple:
            targetAngle = dirToAngle[direc]
        else:
            targetAngle = round(direc) % 360
        if (360 + (targetAngle - bearing)) % 360 >= 180:
            # rotate clockwise
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
        # todo check that there is not another block begins
        if round(self.getBearing()) == 180:
            self.moveBack(0.05)
            self.leftTurnCompass(angle=60)
            self.moveForward(0.2)
            self.rightTurnCompass(angle=60)
            self.moveForward(0.1)
            self.rightTurnCompass(angle=60)
            self.moveForward(0.2)
            self.leftTurnCompass(60)
            self.correctBearing()
        else:
            self.moveBack(0.05)
            self.leftTurnCompass(angle=60)
            self.moveForward(0.2)
            self.rightTurnCompass(angle=60)
            self.moveForward(0.2)
            self.rightTurnCompass(angle=60)
            self.moveForward(0.2)
            self.leftTurnCompass(60)
            self.correctBearing()

    def stop(self):
        for i in self.wheels:
            i.setVelocity(0)

    def correctBearing(self, base = 90):
        current = round(self.getBearing())
        goal = base * round(current/base)
        #note, it uses its own turning code
        self.face(goal)