#!/usr/bin/env python
import serial
import re


class ESP32_Mjolnir():
    def __init__(self):
        self.ser = serial.Serial(port='/dev/ttyACM0', baudrate='115200')
        self.throttle = 0.0
        self.steering = 0.0
        self.speed = 0.0

    def get_input(self):
        """Get input values from ESP32"""
        while self.ser.in_waiting:
            mcu_message = self.ser.readline().decode().lower()  # The message coming in
            number_in_message = re.findall(r'\d+\.?\d*', mcu_message)  # Find number in message

            print("message from ESP32 : " + str(mcu_message))

            # if 'responseSpeed_' in mcu_message:
            #     self.speed = number_in_message[0]

    def emergency_stop(self):
        msg = f'STOP\n'
        self.send(msg)

    def request_speed(self):
        msg = f'pollSpeed\n'
        self.send(msg)

    def send_throttle(self, throttle):
        msg = f'commandThrottle_{throttle}\n'
        self.send(msg)

    def send_steering(self, steering):
        msg = f'commandSteering_{steering}\n'
        self.send(msg)

    def send(self, msg):
        self.ser.write(bytes(msg, 'utf-8'))