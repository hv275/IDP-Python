from controller import Robot


class Dez(Robot):
    def __init__(self):
        #inherit from Robot
        super().__init__()


        #robot consts
        stepInt = 32

        #todo change when I get the actual robot model
        self.wheelRad = 0.4

        #initialise the distance sensor
        self.distSense = self.getDevice("ds_left")
        self.distSense.enable(32)



        #inititialise wheels
        self.wheels = []
        wheelsNames = ['wheel1','wheel2']
        for i in range(len(wheelsNames)):
            self.wheels.append(self.getDevice(wheelsNames[i]))
            self.wheels[i].setPosition(float('inf'))
            self.wheels[i].setVelocity(0.0)


        #todo initialise GPS when sensor is mounted






    def readDist(self):
        return self.distSense.getValue()

    def setSpeed(self, vel):
        for i in self.wheels:
            i.setVelocity(vel)

    def moveForward(self,dist):
        #target is in radians
        target = dist/self.wheelRad

        for i in self.wheels:
            i.setPosition(target)
            i.setVelocity(2.5)






