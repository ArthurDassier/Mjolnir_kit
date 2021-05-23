#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Int32, Int32MultiArray


LANE_GUIDANCE_NODE_NAME = 'lane_guidance_node'
STEERING_TOPIC_NAME = '/steering'
THROTTLE_TOPIC_NAME = '/throttle'
CENTROID_TOPIC_NAME = '/centroid'

global steering_float, throttle_float
steering_float = Float32()
throttle_float = Float32()


def LineFollower(msg):
    kp = 0.195
    global steering_float, throttle_float
    steering_float = Float32()
    throttle_float = Float32()
    centroid = msg.data[0]
    width = msg.data[1]  # width of camera frame

    if msg.data == 0:
        error_x = 0
        throttle_float = 0.95
    else:
        error_x = float(centroid - (width / 2))
        throttle_float = 1.0

    steering_float = float(kp * (error_x / (width / 2)))
    if steering_float < -1:
        steering_float = -1
    elif steering_float > 1:
        steering_float = 1
    else:
        pass
    steering_pub.publish(steering_float)
    throttle_pub.publish(throttle_float)


if __name__ == '__main__':
    rospy.init_node(LANE_GUIDANCE_NODE_NAME, anonymous=False)
    centroid_subscriber = rospy.Subscriber(CENTROID_TOPIC_NAME, Int32MultiArray, LineFollower)
    steering_pub = rospy.Publisher(STEERING_TOPIC_NAME, Float32, queue_size=1)
    throttle_pub = rospy.Publisher(THROTTLE_TOPIC_NAME, Float32, queue_size=1)
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()