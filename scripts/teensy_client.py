#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

TEENSY_NODE_NAME = 'teensy_client'
STEERING_TOPIC_NAME = '/steering'
THROTTLE_TOPIC_NAME = '/throttle'


def callback(data):
    normalized_steering = data.data
    angle_delta = normalized_steering * 90  # difference in degrees from the center 90 degrees
    kit.servo[1].angle = 90 + angle_delta


def listener():
    rospy.init_node(TEENSY_NODE_NAME, anonymous=False)
    rospy.Subscriber(STEERING_TOPIC_NAME, Float32, callback)
    rospy.Subscriber(THROTTLE_TOPIC_NAME, Float32, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()

# class TeensyMC(TeensyMC_Test):
#     def __init__(self, mode=OperationMode.manual, port='/dev/ttyACM0', baudrate=115200, timeout=100, pollInterval=25, left_pulse=430, right_pulse=290,
#                  max_pulse=390, min_pulse=330, zero_pulse=370):
#         super().__init__(mode, port, baudrate, timeout, pollInterval, left_pulse, right_pulse, max_pulse, min_pulse, zero_pulse)

#     def __poll(self):
#         """Get input values from Teensy in manual mode"""
#         while not self.ser.in_waiting:
#             pass

#         mcu_message = self.ser.readline().decode().lower()  # The message coming in
#         sbc_message = 'poll'  # The message to be sent back. a single 'poll' means polling every information
#         number_in_message = re.findall(r'\d+\.*\d*', mcu_message)  # Find number in message

#         if number_in_message: # Correct reading is given, so teensy is alive.
#             self.watchdog_subthread.reset_countdown()  # Reset watchdog as soon as data is received

#             if 'speed' in mcu_message:
#                 self.speed = number_in_message[0]
#                 sbc_message += '_speed'
#             elif 'throttle' in mcu_message:
#                 self.throttle = number_in_message[0]
#                 sbc_message += '_throttle'
#             elif 'steering' in mcu_message:
#                 self.steering = number_in_message[0]
#                 sbc_message += '_steering'
#             elif 'mode' in mcu_message:
#                 self.mode = OperationMode.auto if 'auto' in mcu_message else OperationMode.manual
#                 sbc_message += '_mode'

#             self.ser.write(bytes(sbc_message + '\n', 'utf-8'))

#     def __command(self, throttle=None, speed=None, steering=0, shutdown=False):
#         """Send Instructions to Teensy in auto mode"""
#         # print (speed, steering)

#         #if throttle > 0:
#         #    self.throttle_pulse = dk.utils.map_range(throttle, 0, 1, self.zero_pulse, self.max_pulse)
#         #else:
#         #    self.throttle_pulse = dk.utils.map_range(throttle, -1, 0, self.min_pulse, self.zero_pulse)
#         #self.steering_pulse = dk.utils.map_range(steering, -1, 1, self.left_pulse, self.right_pulse)


#         msg = ''
#         if not shutdown:
#             if throttle is not None:
#                 msg = f'command throttle {throttle}\n'
#             else:
#                 msg = f'command speed {speed}\n'
#             msg += f'command steering {steering}\n'
#         else:
#             msg = 'command shutdown\n'
#         # print (msg)
#         self.ser.write(bytes(msg, 'utf-8'))

#     def run_threaded(self, *speed_and_steering_and_mode):
#         self.watchdog_mainthread.reset_countdown()
#         if True or self.mode == OperationMode.auto and len(speed_and_steering_and_mode) == 2:
#             self.speed = speed_and_steering_and_mode[0]
#             self.steering = speed_and_steering_and_mode[1]
#             self.__command(speed=self.speed, steering=self.steering)

#         # return self.speed, self.throttle, self.steering  # Be very careful with the order 
