from controller import Robot
import math
from Map import gridmap


class Dez(Robot):
    def __init__(self, colour="g"):
        # inherit from Robot
        super().__init__()

        # robot consts
        # not all may be needed
        stepInt = 32
        self.colour = colour
        self.direc = "north"
        self.initflag = 0
        self.defaultSpeed = 3
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

        # todo initialise GPS when sensor is mounted

    def getDist(self):
        return self.distSense.getValue()

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
                print("done")
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
                print("done")
                break

    def rightTurn(self, vel=None, angle=90):
        # configured for 90 degrees only
        # issues with slip during simulation, hard to precisely obtain turning times
        # numbers have been simply tunes for 90 deg turn at 3rad/s
        # can make into other values once we get the compass on
        angle = math.radians(angle)
        arc = angle*0.08*1.8715

        if vel == None:
            vel = self.defaultSpeed
        # todo make a function of geometry
        end_time = self.getTime() + (arc / self.wheelRad) / vel
        while self.step(16) != -1:
            print(self.getTime())
            if self.getTime() < end_time:
                self.wheels[0].setVelocity(vel)
                self.wheels[1].setVelocity(-vel)
            else:
                for i in self.wheels:
                    i.setVelocity(0)
                print("done")
                break

    def leftTurn(self, vel=None, angle=90):
        # turn left
        angle = math.radians(angle)
        arc = angle*0.08*1.8715

        if vel == None:
            vel = self.defaultSpeed
        # todo make a function of geometry
        end_time = self.getTime() + (arc / self.wheelRad) / vel
        while self.step(16) != -1:
            if self.getTime() < end_time:
                self.wheels[0].setVelocity(-vel)
                self.wheels[1].setVelocity(vel)
            else:
                for i in self.wheels:
                    i.setVelocity(0)
                print("done")
                break

    def init(self):
        # initaliser function - only run once at the start to set up
        self.moveForward(0.5)
        self.rightTurn(angle=90)
        self.moveForward(0.5)
        print("Initialisation complete")

    def uturn(self, dir="n"):
        if self.colour == "g":
            # warning - this is only valid for the left handed robot
            if dir == "n":
                self.leftTurn()
                self.moveForward(0.24,3)
                self.leftTurn()
            if dir == "s":
                self.rightTurn()
                self.moveForward(1,3)
                self.rightTurn()

    def isblock(self):
        # code to check if it is a block
        # probably just check for led with camera
        #########
        return 0

    def sweep(self):
        # arbitrary dist - change based on sensor
        if self.getDist() < 200:
            if not self.isblock:
                self.uturn()
            else:
                pass
                # check for colour
                # insert code for claw here
                # and return to base
                # also work on bypassing the block
        else:
            self.moveForward(1)

    def initialise_map(self):
        self.gridMap = gridmap(2.4, 2.4, 0.24)

    def goto(self, dest):
        pass
        # turn to the correct direction depending on the direction you are facing
        # follow a set of instructions as instructed by the navigator - basically north south directions
        # use bearing for direction calculations
        # I really need the model
