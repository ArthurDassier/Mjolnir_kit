#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from std_msgs.msg import Int32, Int32MultiArray
from sensor_msgs.msg import Image
from decoder import decodeImage
import time

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
    orig = img.copy()

    # image post processing

    # experimentally found values from find_camera_values.py
    Hue_low = 19
    Hue_high = 113
    Saturation_low = 0
    Saturation_high = 66
    Value_low = 132
    Value_high = 255
    blur_value = 25
    blur_kernal_value = 5
    dilation_value = 1

    # get rid of white noise
    kernel = np.ones((blur_kernal_value, blur_kernal_value), np.float32) / blur_value
    blurred = cv2.filter2D(img, -1, kernel)
    dilation = cv2.dilate(blurred, kernel, iterations=dilation_value)

    # changing color space to HSV
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # setting threshold limits for white color filter
    lower = np.array([Hue_low, Saturation_low, Value_low])
    upper = np.array([Hue_high, Saturation_high, Value_high])

    # creating mask
    mask = cv2.inRange(hsv, lower, upper)
    res = cv2.bitwise_and(dilation, dilation, mask=mask)
    res = cv2.bitwise_not(res)

    # changing to gray color space
    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

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
        area = cv2.contourArea(contour)
        # print(area) # uncomment for debug
        if 400 < area < 2000:
            # print(area) # uncomment for debug
            x, y, w, h = cv2.boundingRect(contour)
            img = cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
            m = cv2.moments(contour)
            cx = int(m['m10'] / m['m00'])
            cy = int(m['m01'] / m['m00'])
            centers.append([cx, cy])
            cx_list.append(int(m['m10'] / m['m00']))
            cy_list.append(int(m['m01'] / m['m00']))
            cv2.circle(img, (cx, cy), 7, (0, 255, 0), -1)

    try:
        if len(cx_list) >= 2:
            mid_x = int(0.5 * (cx_list[0] + cx_list[1]))
            mid_y = int(0.5 * (cy_list[0] + cy_list[1]))
            cv2.circle(img, (mid_x, mid_y), 7, (255, 0, 0), -1)
            centroid_and_frame_width.append(mid_x)
            centroid_and_frame_width.append(width)
            pub.publish(centroid_and_frame_width)
    except ValueError:
        pass

    if len(cx_list) == 1:
        mid_x = cx_list[0]
        mid_y = cy_list[0]
        cv2.circle(img, (mid_x, mid_y), 7, (255, 0, 0), -1)
        centroid_and_frame_width.append(mid_x)
        centroid_and_frame_width.append(width)
        pub.publish(centroid_and_frame_width)
    elif len(cx_list) == 0:
        pass
    
    # plotting results
    # try:
    #     cv2.imshow("original", orig)
    #     cv2.imshow("mask", mask)
    #     cv2.imshow("blurred", blurred)
    #     cv2.imshow("contours_img", img)
    #     cv2.waitKey(1)
    # except KeyboardInterrupt:
    #     cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('lane_detection_node', anonymous=True)
    camera_sub = rospy.Subscriber('camera_rgb', Image, video_detection)
    pub = rospy.Publisher('/centroid', Int32MultiArray, queue_size=2)
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()
