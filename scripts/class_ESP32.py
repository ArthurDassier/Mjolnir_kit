import serial
import re


class ESP32_Mjolnir():
    def __init__(self):
        self.ser = serial.Serial(port='/dev/ttyUSB0', baudrate='115200')
        self.throttle = 0.0
        self.steering = 0.0
        self.speed = 0.0

    def poll(self):
        """Get input values from ESP32 in manual mode"""
        # Go over all messages (one per line) in the serial buffer
        # Store the values on board (speed,throttle,steering)
        # Check next message
        # If none left, poll function is done
        while self.ser.in_waiting:
            mcu_message = self.ser.readline().decode().lower()  # The message coming in
            number_in_message = re.findall(r'\d+\.?\d*', mcu_message)  # Find number in message

            print("message from ESP32 :" + str(mcu_message))

            # self.watchdog_subthread.reset_countdown()  # Reset watchdog as soon as data is received
            if 'speed' in mcu_message:
                self.speed = number_in_message[0]
            elif 'throttle' in mcu_message:
                self.throttle = number_in_message[0]
            elif 'steering' in mcu_message:
                self.steering = number_in_message[0]

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