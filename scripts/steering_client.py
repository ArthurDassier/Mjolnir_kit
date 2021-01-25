#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from adafruit_servokit import ServoKit

STEERING_NODE_NAME = 'steering_client'
STEERING_TOPIC_NAME = 'steering'

'''
    more documentation at https://learn.adafruit.com/16-channel-pwm-servo-driver/python-circuitpython
    throttle servo is on channel 1
'''

kit = ServoKit(channels=16)

def callback(data):
    normalized_steering = data.data  # this is a value between -1 and 1, with -1 being fully left and 1 being fully right
    angle_delta = normalized_steering * 90  # difference in degrees from the center 90 degrees
    kit.servo[1].angle = 90 + angle_delta


def listener():
    rospy.init_node(STEERING_NODE_NAME, anonymous=False)
    rospy.Subscriber(STEERING_TOPIC_NAME, Float32, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
