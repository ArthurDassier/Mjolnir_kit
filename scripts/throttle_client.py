#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from adafruit_servokit import ServoKit

THROTTLE_NODE_NAME = 'throttle_client'
THROTTLE_TOPIC_NAME = '/throttle'
kit = ServoKit(channels=16)
'''
    more documentation at https://learn.adafruit.com/16-channel-pwm-servo-driver/python-circuitpython
    throttle servo is on channel 0 
'''
throttle_scale = 0.03  # scale down sensitive throttle


def callback(data):
    # output_start= rospy.get_param("/Throttle_max_reverse")
    # output_end = rospy.get_param("/Throttle_max_forward")
    # Throttle_neutral = rospy.get_param("/Throttle_neutral")
    #
    # input_start = -1
    # input_end = 1
    #
    # input_throttle = data.data
    # normalized_throttle = output_start + (input_throttle - input_start) * ((output_end - output_start) / (input_end - input_start))
    normalized_throttle = data.data
    kit.continuous_servo[0].throttle = normalized_throttle * throttle_scale


def listener():
    rospy.init_node(THROTTLE_NODE_NAME, anonymous=False)
    rospy.Subscriber(THROTTLE_TOPIC_NAME, Float32, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
