#!/usr/bin/env python
import serial
import re


class PIDController():
    def __init__(self):
        self.kp = 0.5
        self.ki = 0.5
        self.kd = 0.5
        self.tau = 0.5

        self.limMin = -1.0
        self.limMax = 1.0
        self.limMinInt = -1
        self.limMaxInt = 1

        self.T = 0.5
        self.integrator = 0.5
        self.prevError = 0.5
        self.differentiator = 0.5
        self.prevCentroid = 0.5
        self.out = 0.5

    def get_input(self):
        ''''''