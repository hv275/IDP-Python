from controller import Robot


class Dez(Robot):
    def __init__(self):
        super().__init__()
        stepInt = 32

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


        #initialise GPS





    def readDist(self):
        return self.distSense.getValue()

    def setSpeed(self, vel):
        for i in self.wheels:
            i.setVelocity(vel)



