#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from std_msgs.msg import Int32, Int32MultiArray
from sensor_msgs.msg import Image
from decoder import decodeImage
import time


LANE_DETECTION_NODE_NAME = 'lane_detection_node'
CAMERA_TOPIC_NAME = 'camera_rgb'
CENTROID_TOPIC_NAME = '/centroid'

global mid_x, mid_y
mid_x = Int32()
mid_y = Int32()


def video_detection(data):
    frame = decodeImage(data.data, data.height, data.width)

    height, width, channels = frame.shape
    rows_to_watch = 50
    rows_offset = 150
    top_height = height - rows_offset
    bottom_height = top_height + rows_to_watch

    img = cv2.cvtColor(frame[top_height:bottom_height, 0:width], cv2.COLOR_RGB2BGR)

    # image post processing

    # experimentally found values from find_camera_values.py
    Hue_low = rospy.get_param("/Hue_low")
    Hue_high = rospy.get_param("/Hue_high")
    Saturation_low = rospy.get_param("/Saturation_low")
    Saturation_high = rospy.get_param("/Saturation_high")
    Value_low = rospy.get_param("/Value_low")
    Value_high = rospy.get_param("/Value_high")
    min_width = rospy.get_param("/Width_min")
    max_width = rospy.get_param("/Width_max")
    green_filter = rospy.get_param("/green_filter")

    # changing color space to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    # setting threshold limits for white color filter
    lower = np.array([Hue_low, Saturation_low, Value_low])
    upper = np.array([Hue_high, Saturation_high, Value_high])

    # creating mask
    mask = cv2.inRange(hsv, lower, upper))
    if green_filter:
        res_inv = cv2.bitwise_and(img, img, mask=cv2.bitwise_not(mask)) # comment when not using green filter
    else:
        res_inv = cv2.bitwise_and(img, img, mask=mask)

    # changing to gray color space
    gray = cv2.cvtColor(res_inv, cv2.COLOR_BGR2GRAY)

    # changing to black and white color space
    (dummy, blackAndWhiteImage) = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    # locating contours in image
    contours, dummy = cv2.findContours(blackAndWhiteImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    centers = []
    cx_list = []
    cy_list = []
    centroid_and_frame_width = []

    # plotting contours and their centroids
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if min_width < w < max_width:
            img = cv2.drawContours(img, contour, -1, (0, 255, 0), 3)
            m = cv2.moments(contour)
            try:
                cx = int(m['m10'] / m['m00'])
                cy = int(m['m01'] / m['m00'])
                centers.append([cx, cy])
                cx_list.append(int(m['m10'] / m['m00']))
                cy_list.append(int(m['m01'] / m['m00']))
                cv2.circle(img, (cx, cy), 7, (0, 255, 0), -1)
            except ZeroDivisionError:
                pass

    try:
        if len(cx_list) >= 2:
            mid_x = int(0.5 * (cx_list[0] + cx_list[1]))
            mid_y = int(0.5 * (cy_list[0] + cy_list[1]))
            cv2.circle(img, (mid_x, mid_y), 7, (255, 0, 0), -1)
            centroid_and_frame_width.append(mid_x)
            centroid_and_frame_width.append(width)
            pub.publish(data=centroid_and_frame_width)
        elif len(cx_list) == 1:
            mid_x = cx_list[0]
            mid_y = cy_list[0]
            cv2.circle(img, (mid_x, mid_y), 7, (0, 0, 255), -1)
            centroid_and_frame_width.append(mid_x)
            centroid_and_frame_width.append(width)
            pub.publish(data=centroid_and_frame_width)
        elif len(cx_list) == 0:
            pass
    except ValueError:
        pass

    # try:
    #     # plotting results
    #     cv2.imshow("original", orig)
    #     cv2.imshow("mask", mask)
    #     cv2.imshow("blackAndWhiteImage", blackAndWhiteImage)
    #     cv2.imshow("plotting_centroid", img)
    #     cv2.waitKey(1)
    # except KeyboardInterrupt:
    #     cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node(LANE_DETECTION_NODE_NAME, anonymous=False)
    camera_sub = rospy.Subscriber(CAMERA_TOPIC_NAME, Image, video_detection)
    pub = rospy.Publisher(CENTROID_TOPIC_NAME, Int32MultiArray, queue_size=2)
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()
