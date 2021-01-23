#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from adafruit_servokit import ServoKit

THROTTLE_NODE_NAME = 'throttle_client'
THROTTLE_TOPIC_NAME = 'throttle'

'''
    more documentation at https://learn.adafruit.com/16-channel-pwm-servo-driver/python-circuitpython
    throttle servo is on channel 0
'''

kit = ServoKit(channels=16)
throttle_scale = 0.2  # scale down sensitive throttle


def callback(data):
    rospy.loginfo(data.data)
    normalized_throttle = data.data
    kit.continuous_servo[0].throttle = normalized_throttle * throttle_scale


def listener():
    rospy.init_node(THROTTLE_NODE_NAME, anonymous=False)
    rospy.Subscriber(THROTTLE_TOPIC_NAME, Float32, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
