#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import Float32
from class_ESP32 import ESP32_Mjolnir

ESP32_NODE_NAME = 'esp32_client'
STEERING_TOPIC_NAME = '/steering'
THROTTLE_TOPIC_NAME = '/throttle'

esp = ESP32_Mjolnir()

throttle_scale = 1.0  # scale down sensitive throttle

def steering_callback(data):
    ''' Steering '''
    normalized_steering = data.data
    angle_delta = normalized_steering * 90  # difference in degrees from the center 90 degrees
    esp.send_steering(angle_delta)
    esp.get_input()


def throttle_callback(data):
    ''' Throttle '''
    normalized_throttle = data.data
    esp.send_throttle(normalized_throttle * throttle_scale)



def listener():
    rospy.init_node(ESP32_NODE_NAME, anonymous=False)
    rospy.Subscriber(STEERING_TOPIC_NAME, Float32, steering_callback)
    rospy.Subscriber(THROTTLE_TOPIC_NAME, Float32, throttle_callback)
    rospy.spin()


if __name__ == '__main__':
    listener()