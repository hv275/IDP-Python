from controller import Robot

class wheel:
    def __init__(self,obj):
        self.obj = obj
        self.pos = 0


class Dez(Robot):
    def __init__(self, colour="g"):
        #inherit from Robot
        super().__init__()


        #robot consts
        stepInt = 32
        self.colour = colour
        self.direc = "north"
        self.initflag = 0
        self.defaultSpeed = 3


        #todo change when I get the actual robot model
        self.wheelRad = 0.4

        #initialise the distance sensor
        self.distSense = self.getDevice("ds_left")
        self.distSense.enable(32)



        #inititialise wheels
        self.wheels = []
        wheelsNames = ['wheel1','wheel2']
        for i in range(len(wheelsNames)):
            self.wheels.append(wheel(self.getDevice(wheelsNames[i])))
            self.wheels[i].obj.setPosition(float("inf"))
            self.wheels[i].obj.setVelocity(0.0)


        #todo initialise GPS when sensor is mounted



    def getDist(self):
        return self.distSense.getValue()


    def moveForward(self,dist,vel=None):
        if vel == None:
            vel = self.defaultSpeed
        end_time = self.getTime() + (dist/self.wheelRad)/vel
        while self.step(32) != -1:
            if self.getTime() < end_time:
                for i in self.wheels:
                    i.obj.setVelocity(vel)
            else:
                for i in self.wheels:
                    i.obj.setVelocity(0)
                print("done")
                break

    def rightTurn(self, vel = None):
        if vel == None:
            vel = self.defaultSpeed
        # todo make a function of geometry
        dist = 1.1
        end_time = self.getTime() + (dist / self.wheelRad) / vel
        while self.step(32) != -1:
            if self.getTime() < end_time:
                self.wheels[0].obj.setVelocity(vel)
                self.wheels[1].obj.setVelocity(-vel)
            else:
                for i in self.wheels:
                    i.obj.setVelocity(0)
                print("done")
                break

    def leftTurn(self, vel = None):
        #turn left
        if vel == None:
            vel = self.defaultSpeed
        # todo make a function of geometry
        dist = 1.11
        end_time = self.getTime() + (dist / self.wheelRad) / vel
        while self.step(32) != -1:
            if self.getTime() < end_time:
                self.wheels[0].obj.setVelocity(-vel)
                self.wheels[1].obj.setVelocity(vel)
            else:
                for i in self.wheels:
                    i.obj.setVelocity(0)
                print("done")
                break

    def init(self):
        #initaliser function - only run once at the start to set up
        self.moveForward(3)
        self.leftTurn()
        self.moveForward(3)
        print("Initialisation complete")

    def uturn(self, dir = "n"):
        #warning - this is only valid for the left handed robot
        if dir == "n":
            self.leftTurn()
            self.moveForward(1,3)
            self.leftTurn()
        if dir == "s":
            self.rightTurn()
            self.moveForward()
            self.rightTurn()

    def isblock(self):
        #code to check if it is a block
        #probably just check for led with camera
        #########
        return 0


    def sweep(self):
        #arbitrary dist - change based on sensor
        if self.getDist() < 200:
            if not self.isblock:
                self.uturn()
            else:
                pass
                #check for colour
                #insert code for claw here
                #and return to base
                #also work on bypassing the block
        else:
            self.moveForward(1)

    def goto(self,dest):
        pass
        #turn to the correct direction depending on the direction you are facing
        #follow a set of instructions as instructed by the navigator - basically north south directions
        #use bearing for direction calculations
        #I really need the model






















