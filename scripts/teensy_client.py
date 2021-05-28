#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import Float32
from class_Teensy import TeensyMjolnir

TEENSY_NODE_NAME = 'teensy_client'
STEERING_TOPIC_NAME = '/steering'
THROTTLE_TOPIC_NAME = '/throttle'

esp = TeensyMjolnir()

def steering_callback(data):
    normalized_steering = data.data
    angle_delta = normalized_steering * 90  # difference in degrees from the center 90 degrees
    esp.send_steering(angle_delta)
    esp.poll()
    #TeensyMjolnir().__send_throttle(data)


def steering_throttle(data):
    normalized_steering = data.data
    angle_delta = normalized_steering * 90  # difference in degrees from the center 90 degrees



def listener():
    rospy.init_node(TEENSY_NODE_NAME, anonymous=False)
    rospy.Subscriber(STEERING_TOPIC_NAME, Float32, steering_callback)
    rospy.Subscriber(THROTTLE_TOPIC_NAME, Float32, steering_throttle)
    rospy.spin()


if __name__ == '__main__':
    listener()