from controller import Robot
import math
from Map import gridmap
import numpy as np


class Dez(Robot):
    def __init__(self):
        # inherit from Robot
        super().__init__()

        # robot consts
        # not all may be needed
        self.stepInt = 32
        self.name = self.getName()
        self.direc = "n"
        self.defaultSpeed = 4
        self.gridMap = None
        self.gridSquare = 1
        self.lastPath = None
        print(self.name)
        self.wheelRad = 0.05



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

        self.gps = self.getDevice("gps")
        self.gps.enable(self.stepInt)
        self.frontDist = self.getDevice("distance_sensor_front")
        self.frontDist.enable(self.stepInt)
        self.backDist = self.getDevice("distance_sensor_back")
        self.backDist.enable(self.stepInt)
        self.comp = self.getDevice("compass")
        self.comp.enable(self.stepInt)

        #Camera initilisation
        self.camera = self.getDevice("camera")
        self.camera.enable(self.stepInt)

    def getDist(self):
        return self.frontDist.getValue()

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
            coords[i[0]] = abs(i[1] * 10 - (i[1] * 10) % self.gridSquare)
        # in place modification
        # this is dumb but I do not have the time for a cleaner fix
        coords.reverse()
        return coords

    def moveArmDown(self):
        end_time = self.getTime() + 2
        while self.step(32) != 1:
            if self.getTime() < end_time:
                self.claw.setVelocity(0.2)

            else:
                self.claw.setVelocity(0)
                break

    def moveArmUp(self):
        end_time = self.getTime() + 2
        while self.step(32) != 1:
            if self.getTime() < end_time:
                self.claw.setVelocity(-0.2)
                # self.clawhead.setVelocity(-0.15)
            else:
                # self.clawhead.setVelocity(0)
                self.claw.setVelocity(0)
                # self.clawhead.setVelocity(0)
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

    def rightTurnCompass(self,angle=90,vel=None):
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

    def leftTurnCompass(self, angle=90,vel=None):
        # it may go around more than once
        # that is fine, I do not have the time to properly fix it
        start = round(self.getBearing()) % 360

        end = (start + 360 - angle) % 360

        if vel == None:
            # artificially
            vel = 1

        while self.step(16) != 1:
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
        self.moveForward(0.35)
        self.leftTurnCompass(angle=45)
        print("Initialisation complete")

    def uturn(self):
        for i in self.wheels:
            i.setVelocity(0)
        # warning - this is valid for both robots (kinda)
        # warning - this is valid for both robots (kinda)
        if self.direc == "n":
            self.leftTurnCompass()
            self.moveForward(0.15, 3)
            self.leftTurnCompass()
            self.direc = "s"
        elif self.direc == "s":
            self.rightTurnCompass()
            self.moveForward(0.15, 3)
            self.rightTurnCompass()
            self.direc = "n"

    def isblock(self):
        # code to check if it is a block
        # probably just check for led with camera
        self.moveArmDown()
        camera_image = self.camera.getImage()
        # get coloured components of pixels
        red = self.camera.imageGetRed(camera_image, self.camera.getWidth(), 0,0)
        print(red)
        green = self.camera.imageGetGreen(camera_image, self.camera.getWidth(), 0,0)
        print(green)
        grey = self.camera.imageGetGrey(camera_image, self.camera.getWidth(), 0,0)
        print(grey)
        print(self.camera.getImageArray())

        if (red > grey or green > grey):
            self.moveArmUp()
            return True

        else:
            self.moveArmUp()
            return False

    def sweep(self):
        # arbitrary dist - change based on sensor
        self.moveForward(0.05)
        loc = self.getGPS()
        while self.step(16) != -1:
            if self.getDist() > 650:
                self.uturn()
                break
            #break clause not fully functional yet
            if loc[0] <3 and loc[1]<3 and self.name == "Dez":
                return 1

            elif loc[0] > 20 and loc[1]<3 and self.name == "Troy":
                return 1

            else:
                for i in self.wheels:
                    i.setVelocity(3)

    def initialise_map(self):
        self.gridMap = gridmap(24, 24, self.gridSquare)

    def goto(self, dest):
        start = tuple([math.floor(i) for i in self.getGPS()])
        end = tuple([round(i - i % self.gridSquare) for i in dest])
        print(start)
        print(end)
        route = self.gridMap.directions(start, end)
        self.lastPath = route
        for dir in route:
            self.face(dir)
            self.moveForward(0.12)
            print(self.getGPS())

    def gotoBearing(self,dest):
        #determine whether to the left or to the right
        curr = np.array(self.getGPS())
        vec = (np.array(dest) - curr)
        vec = vec/np.linalg.norm(vec)
        north = np.array([0,1])
        dot = np.dot(north,vec)
        targetAngle = (math.degrees(np.arccos(dot))+360)%360
        bearing = self.getBearing()
        self.face(targetAngle)
        while self.step(self.stepInt) != 1:
            dist = self.getGPS()


    def returnToPoint(self):
        #use this to return to points, please do not use anything else as it will not return to where we expect it to be. Am investigating
        route = self.lastPath
        for i in range(len(route)):
            route[i] = list(route[i])
            route[i][0] -= route[i][0]*2
            route[i][1] -= route[i][1]*2
            route[i] = tuple(route[i])
        route.reverse()
        for dir in route:
            self.face(dir)
            self.moveForward(0.1)


    def face(self, direc):
        """Argument has to be a touple or a bearing"""
        vel = 1
        #dictionary look up to find angles (only works with 90 increments for now
        #I may look into using maths to find the angle but for now this works well

        dirToAngle = {(0, 1): 0, (0, -1): 180, (1, 0): 90, (-1, 0): 270}
        bearing = round(self.getBearing()) % 360
        if type(direc) == tuple:
            targetAngle = dirToAngle[direc]
        else:
            print(direc)
            targetAngle = round(direc)%360
        #code below uses some serious abuse of modulo operator
        if (360+(targetAngle-bearing))%360 >= 180:
            #rotate clockwise
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
        #todo check that there is not another block begind
        self.rightTurnCompass(angle = 45)
        self.moveForward(0.2)
        self.leftTurnCompass(angle = 45)
        self.moveForward(0.2)
        self.leftTurnCompass(angle = 45)
        self.moveForward(0.2)
        self.rightTurnCompass(45)


