#!/usr/bin/env python

class PIDController():
    def __init__(self):
        self.kp = 0.5
        self.ki = 0.0
        self.kd = 0.0
        self.tau = 0.5

        self.limMin = -1.0
        self.limMax = 1.0
        self.limMinInt = -1
        self.limMaxInt = 1

        self.errorNormalize = (1.0/400.0)
        self.T = 0.05
        self.integrator = 0.0
        self.prevError = 0.5
        self.differentiator = 0.0
        self.prevCentroid = 0.5
        self.out = 0.5

    # def get_input(self):
    #     ''''''

    def tickPID(self, setpoint, currentPos, msg, throttle_float):

        if msg.data == 0:
            error_x = 0.0
        else:
            error_x = float(currentPos - setpoint)
            error_x = error_x * self.errorNormalize

        self.proportional = self.kp * error_x
        if self.proportional < self.limMin:
           self.proportional = self.limMin
        elif self.proportional > self.limMax:
            self.proportional = self.limMax
        else:
            pass

        self.integrator = self.integrator + 0.5 * self.ki * self.T * (error_x + self.prevError)
        if self.integrator < self.limMin:
           self.integrator = self.limMin
        elif self.integrator > self.limMax:
            self.integrator = self.limMax
        else:
            pass

        # Derivative (band-limited differentiator)
        self.differentiator = -(2.0 * self.kd * (currentPos - self.prevCentroid) + (2.0 * self.tau - self.T) * self.differentiator) / (2.0 * self.tau + self.T)

        #Compute output and apply limits
        self.out = self.proportional + self.integrator + self.differentiator
        if (self.out > self.limMax):
            self.out = self.limMax
        elif (self.out < self.limMin):
            self.out = self.limMin

        print("--------------------------------------------------------------")
        print("Centroid   [{}]".format(setpoint))
        print("currentPos [{}]".format(currentPos))
        print("error      [{}]".format(error_x))
        print("prop.      [{}]".format(self.proportional))
        print("integ.     [{}]".format(self.integrator))
        print("differ.    [{}]".format(self.differentiator))
        print("steering   [{}]".format(self.out))
        print("throttle   [{}]".format(throttle_float))
        print("--------------------------------------------------------------")

        #Store error and measurement for later use
        self.prevError = error_x
        self.prevCentroid = currentPos
