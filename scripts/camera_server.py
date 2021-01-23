#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

CAMERA_NODE_NAME = 'camera_server'
CAMERA_TOPIC_NAME = 'camera_rgb'

cv2_video_capture = cv2.VideoCapture(0)
CAMERA_FREQUENCY = 10  # Hz


def talker():
    pub = rospy.Publisher(CAMERA_TOPIC_NAME, Image, queue_size=10)
    rospy.init_node(CAMERA_NODE_NAME, anonymous=True)
    rate = rospy.Rate(CAMERA_FREQUENCY)

    while not rospy.is_shutdown():
        ret, frame = cv2_video_capture.read()

        # construct msg
        bridge = CvBridge()
        rgb = bridge.cv2_to_imgmsg(frame)
        pub.publish(rgb)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
