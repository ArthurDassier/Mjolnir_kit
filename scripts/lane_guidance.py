#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Int32

global steering_float, throttle_float
steering_float = Float32()
throttle_float = Float32()


def LineFollower(msg):
    kp = 0.195
    global steering_float, throttle_float
    steering_float = Float32()
    throttle_float = Float32()
    width = 800  # width of camera frame

    if msg.data == 0:
        error_x = 0
        throttle_float = 0.135
    else:
        error_x = float(((msg.data) - width / 2))
        throttle_float = 0.14

    steering_float = float(kp * (error_x / (width / 2)))
    if steering_float < -1:
        steering_float = -1
    elif steering_float > 1:
        steering_float = 1
    else:
        pass


def main():
    rospy.init_node('lane_guidance_node', anonymous=True)
    centroid_subscriber = rospy.Subscriber('/centroid', Int32, LineFollower)
    steering_pub = rospy.Publisher('steering', Float32, queue_size=1)
    throttle_pub = rospy.Publisher('throttle', Float32, queue_size=1)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        
        steering_pub.publish(steering_float)
        throttle_pub.publish(throttle_float)
        rate.sleep()


if __name__ == '__main__':
    main()
