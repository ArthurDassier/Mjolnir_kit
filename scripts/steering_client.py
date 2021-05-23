#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from adafruit_servokit import ServoKit

STEERING_NODE_NAME = 'steering_client'
STEERING_TOPIC_NAME = '/steering'
kit = ServoKit(channels=16)
'''
    more documentation at https://learn.adafruit.com/16-channel-pwm-servo-driver/python-circuitpython
    throttle servo is on channel 1 
'''


def callback(data):
    # output_start = rospy.get_param("/Steering_max_left")
    # output_end= rospy.get_param("/Steering_max_right")
    # Steering_straight = rospy.get_param("/Steering_straight")
    #
    # input_start = -1
    # input_end = 1
    #
    # input_steering = data.data  # this is a value between -1 and 1, with -1 being fully left and 1 being fully right
    # normalized_steering = output_start + (input_steering - input_start) * ((output_end - output_start) / (input_end - input_start))
    normalized_steering = data.data
    angle_delta = normalized_steering * 90  # difference in degrees from the center 90 degrees
    kit.servo[1].angle = 90 + angle_delta


def listener():
    rospy.init_node(STEERING_NODE_NAME, anonymous=False)
    rospy.Subscriber(STEERING_TOPIC_NAME, Float32, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
