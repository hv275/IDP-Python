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
        self.initflag = 0
        self.defaultSpeed = 2
        self.gridMap = None

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
        # self.clawhead = self.getDevice("claw_motor")
        # self.clawhead.setPosition(float("inf"))
        # self.clawhead.setVelocity(0)

        self.gps = self.getDevice("gps")
        self.gps.enable(self.stepInt)
        self.frontDist = self.getDevice("distance_sensor_front")
        self.frontDist.enable(self.stepInt)
        self.backDist = self.getDevice("distance_sensor_back")
        self.backDist.enable(self.stepInt)
        self.comp = self.getDevice("compass")
        self.comp.enable(self.stepInt)
        self.leftcoder = self.getDevice("l_wheel_encoder")
        self.leftcoder.enable(self.stepInt)
        self.rightcoder = self.getDevice("r_wheel_encoder")
        self.rightcoder.enable(self.stepInt)

    def getDist(self):
        return self.frontDist.getValue()

    def getBearing(self):
        north = self.comp.getValues()
        rad = math.atan2(north[0], north[2])
        bearing = (rad - 1.5708) / math.pi * 180
        if bearing < 0:
            bearing += 360

        return bearing

    def moveArmDown(self):
        end_time = self.getTime() + 1
        while self.step(32) != 1:
            if self.getTime() < end_time:
                self.claw.setVelocity(0.2)
                # self.clawhead.setVelocity(0.2)
            else:
                # self.clawhead.setVelocity(0)
                self.claw.setVelocity(0)
                break

    def moveArmUp(self):
        end_time = self.getTime() + 1
        print(self.getTime())
        print(end_time)
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
            vel = self.defaultSpeed + 2
        end_time = self.getTime() + (dist / self.wheelRad) / vel
        while self.step(32) != -1:
            if self.getTime() < end_time:
                for i in self.wheels:
                    i.setVelocity(vel)

            else:
                for i in self.wheels:
                    i.setVelocity(0)
                break

    def rightTurn(self, vel=None, angle=90):
        # configured for 90 degrees only
        # issues with slip during simulation, hard to precisely obtain turning times
        # numbers have been simply tunes for 90 deg turn at 3rad/s
        # can make into other values once we get the compass on
        angle = math.radians(angle)
        arc = angle * 0.12 * 1.578818

        if vel == None:
            vel = self.defaultSpeed
        # todo make a function of geometry
        end_time = self.getTime() + (arc / self.wheelRad) / vel
        while self.step(16) != -1:
            if self.getTime() < end_time:
                self.wheels[0].setVelocity(vel)
                self.wheels[1].setVelocity(-vel)
            else:
                for i in self.wheels:
                    i.setVelocity(0)
                break

    def leftTurn(self, vel=None, angle=90):
        # turn left
        angle = math.radians(angle)
        arc = angle * 0.12 * 1.578818

        if vel == None:
            vel = self.defaultSpeed

        end_time = self.getTime() + (arc / self.wheelRad) / vel
        while self.step(16) != -1:
            if self.getTime() < end_time:
                self.wheels[0].setVelocity(-vel)
                self.wheels[1].setVelocity(vel)
            else:
                for i in self.wheels:
                    i.setVelocity(0)
                break

    def rightTurnCompass(self, vel=None, angle=90):
        #it may go around more than once
        #that is fine, I do not have the time to properly fix it
        start = round(self.getBearing())%360
        print(f"Start {start}")
        end = (start + 90)%360

        if vel == None:
            vel = self.defaultSpeed

        while self.step(16) != 1:
            bearing = round(self.getBearing())%360
            print(bearing)
            print(end)
            if bearing  != end:
                self.wheels[0].setVelocity(vel)
                self.wheels[1].setVelocity(-vel)

            else:
                for i in self.wheels:
                    i.setVelocity(0)
                break

    def leftTurnCompass(self, vel = None, angle = 90):
        # it may go around more than once
        # that is fine, I do not have the time to properly fix it
        start = round(self.getBearing()) % 360
        print(f"Start {start}")
        end = (start +270) % 360

        if vel == None:
            vel = self.defaultSpeed

        while self.step(16) != 1:
            bearing = round(self.getBearing()) % 360
            print(bearing)
            print(end)
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
        self.moveForward(0.25)
        self.rightTurn(angle=90)
        self.moveForward(0.25)
        print("Initialisation complete")

    def uturn(self):
        for i in self.wheels:
            i.setVelocity(0)
        if self.colour == "g":
            # warning - this is only valid for the left handed robot
            if self.direc == "n":
                self.leftTurn()
                self.moveForward(0.24, 3)
                self.leftTurn()
                self.direc = "s"
                print("done")
            elif self.direc == "s":
                self.leftTurn()
                self.moveForward(0.24, 3)
                self.leftTurn()
                self.direc = "n"

    def isblock(self):
        # code to check if it is a block
        # probably just check for led with camera
        #########
        return False

    def sweep(self):
        # arbitrary dist - change based on sensor
        self.moveForward(0.05)
        print(self.getDist())
        while self.step(16) != -1:
            if self.getDist() > 10:
                self.uturn()
                break
            else:
                for i in self.wheels:
                    i.setVelocity(3)

    def initialise_map(self):
        self.gridMap = gridmap(2.4, 2.4, 0.24)

    def goto(self, dest):
        pass
        # turn to the correct direction depending on the direction you are facing
        # follow a set of instructions as instructed by the navigator - basically north south directions
        # use bearing for direction calculations
        # I really need the model
