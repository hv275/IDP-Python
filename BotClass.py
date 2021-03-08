from controller import Robot
import math
from Map import gridmap


class Dez(Robot):
    def __init__(self, colour="g"):
        # inherit from Robot
        super().__init__()

        # robot consts
        # not all may be needed
        self.stepInt = 32
        self.colour = colour
        self.direc = "n"
        self.defaultSpeed = 5
        self.gridMap = None
        self.gridSquare = 1
        self.lastPath = None

        self.wheelRad = 0.05

        # initialise the distance sensor (currently commented out)
        """
        self.distSense = self.getDevice("ds_left")
        self.distSense.enable(32)"""

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
        coords = self.gps.getValues()[0::2]

        for i in enumerate(coords):
            #
            coords[i[0]] = abs(i[1] * 10 - (i[1] * 10) % self.gridSquare)
        return coords

    def moveArmDown(self):
        end_time = self.getTime() + 1.5
        while self.step(32) != 1:
            if self.getTime() < end_time:
                self.claw.setVelocity(0.2)

            else:
                self.claw.setVelocity(0)
                break

    def moveArmUp(self):
        end_time = self.getTime() + 1.5
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

    def rightTurnCompass(self, vel=None, angle=90):
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

    def leftTurnCompass(self, vel=None, angle=90):
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
        self.moveForward(0.2)
        self.leftTurnCompass(angle=45)
        print("Initialisation complete")

    def uturn(self):
        for i in self.wheels:
            i.setVelocity(0)
        if self.colour == "g":
            # warning - this is only valid for the left handed robot
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
        red = camera_image.imageGetRed(camera_image, self.camera.getWidth(), 0,0)
        blue = camera_image.imageGetBlue(camera_image, self.camera.getWidth(), 0,0)
        grey = camera_image.imageGetGray(camera_image, self.camera.getWidth(), 0,0)

        if (red > grey or blue > grey):
            return True

        else:
            return False

    def sweep(self):
        # arbitrary dist - change based on sensor
        self.moveForward(0.05)
        while self.step(16) != -1:
            if self.getDist() > 10:
                self.uturn()
                break
            else:
                for i in self.wheels:
                    i.setVelocity(3)

    def initialise_map(self):
        self.gridMap = gridmap(22, 22, self.gridSquare)

    def goto(self, dest):
        start = tuple([math.floor(i) for i in self.getGPS()])
        end = tuple([round(i - i % self.gridSquare) for i in dest])
        print(start)
        print(end)
        route = self.gridMap.directions(start, end)
        self.lastPath = route
        for dir in route:
            self.face(dir)
            self.moveForward(0.1)

    def returnToPoint(self):
        route = self.lastPath
        print(route)
        for i in range(len(route)):
            route[i] = list(route[i])
            route[i][0] -= route[i][0]*2
            route[i][1] -= route[i][1]*2
            route[i] = tuple(route[i])
        route.reverse()
        print(route)
        for dir in route:
            self.face(dir)
            self.moveForward(0.1)


    def face(self, direc):
        """Argument has to be a touple"""
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



