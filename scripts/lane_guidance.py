#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Int32, Int32MultiArray

global steering_float, throttle_float
steering_float = Float32()
throttle_float = Float32()


def LineFollower(msg):
    kp = 0.195
    global steering_float, throttle_float
    steering_float = Float32()
    throttle_float = Float32()
    centroid = msg[0]
    width = msg[1]  # width of camera frame

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
    rospy.init_node('lane_guidance_node', anonymous=True)
    centroid_subscriber = rospy.Subscriber('/centroid', Int32MultiArray, LineFollower)
    steering_pub = rospy.Publisher('steering', Float32, queue_size=1)
    throttle_pub = rospy.Publisher('throttle', Float32, queue_size=1)
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()
