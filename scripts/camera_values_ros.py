#!/usr/bin/env python
import os.path
import rospy
import cv2
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from decoder import decodeImage

CAMERA_VALUES_NODE_NAME = 'camera_values_node'
CAMERA_TOPIC_NAME = 'camera_rgb'
STEERING_TOPIC_NAME = '/steering'
THROTTLE_TOPIC_NAME = '/throttle'

cv2.namedWindow('sliders')


def callback(x):
    pass

def slider_to_normalized(slider_input):
    input_start = 0
    input_end = 2000
    output_start = -1
    output_end = 1
    normalized_output = output_start + (slider_input - input_start) * (
            (output_end - output_start) / (input_end - input_start))
    return normalized_output

lowH = 0
highH = 179
lowS = 0
highS = 255
lowV = 0
highV = 255
area_min = 1
area_max = 50000

min_width = 10
max_width = 500

steer_left = 0
steer_straight = 1000
steer_right = 2000
throttle_reverse = 0
throttle_neutral = 1080
throttle_forward = 2000

cv2.createTrackbar('lowH', 'sliders', lowH, highH, callback)
cv2.createTrackbar('highH', 'sliders', lowH, highH, callback)

cv2.createTrackbar('lowS', 'sliders', lowS, highS, callback)
cv2.createTrackbar('highS', 'sliders', highS, highS, callback)

cv2.createTrackbar('lowV', 'sliders', lowV, highV, callback)
cv2.createTrackbar('highV', 'sliders', highV, highV, callback)

cv2.createTrackbar('min_width', 'sliders', min_width, max_width, callback)
cv2.createTrackbar('max_width', 'sliders', min_width, max_width, callback)

cv2.createTrackbar('Steering_value', 'sliders', steer_straight, steer_right, callback)
cv2.createTrackbar('Throttle_value', 'sliders', throttle_neutral, throttle_forward, callback)

global steering_float, throttle_float
steering_float = Float32()
throttle_float = Float32()

def camera_values(data):
    global steering_float, throttle_float
    steering_float = Float32()
    throttle_float = Float32()

    frame = decodeImage(data.data, data.height, data.width)
    height, width, channels = frame.shape
    start_height = int(height * 0.60)
    bottom_height = int(height * 0.80)

    left_width = int(width / 4)
    right_width = int(3 * width / 4)

    left_width = int(0)
    right_width = int(width)
    img = frame[start_height:bottom_height, left_width:right_width]

    # get trackbar positions
    lowH = cv2.getTrackbarPos('lowH', 'sliders')
    highH = cv2.getTrackbarPos('highH', 'sliders')
    lowS = cv2.getTrackbarPos('lowS', 'sliders')
    highS = cv2.getTrackbarPos('highS', 'sliders')
    lowV = cv2.getTrackbarPos('lowV', 'sliders')
    highV = cv2.getTrackbarPos('highV', 'sliders')
    min_area = cv2.getTrackbarPos('min_area', 'sliders')
    max_area = cv2.getTrackbarPos('max_area', 'sliders')
    min_width = cv2.getTrackbarPos('min_width', 'sliders')
    max_width = cv2.getTrackbarPos('max_width', 'sliders')

    steer_input = cv2.getTrackbarPos('Steering_value', 'sliders')
    throttle_input = cv2.getTrackbarPos('Throttle_value', 'sliders')


    # set ros parameters
    rospy.set_param('/Hue_low', lowH)
    rospy.set_param('/Hue_high', highH)
    rospy.set_param('/Saturation_low', lowS)
    rospy.set_param('/Saturation_high', highS)
    rospy.set_param('/Value_low', lowV)
    rospy.set_param('/Value_high', highV)
    rospy.set_param('/Area_min', min_area)
    rospy.set_param('/Area_max', max_area)
    rospy.set_param('/Width_min', min_width)
    rospy.set_param('/Width_max', max_width)

    # Write files to yaml file for storage
    f = open(os.path.dirname(__file__) + '/../config/color_filter_parameters/custom_filter.yaml', "w")
    f.write(f"Hue_low : {lowH} \n"
            f"Hue_high : {highH} \n"
            f"Saturation_low : {lowS} \n"
            f"Saturation_high : {highS} \n"
            f"Value_low : {lowV} \n"
            f"Value_high : {highV} \n"
            f"Area_min : {min_width} \n"
            f"Area_max : {max_width} \n"
            f"Width_min : {min_width} \n"
            f"Width_max : {max_width} \n"
            f"green_filter : {green_filter} \n")
    f.close()

    # changing color space to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    lower = np.array([lowH, lowS, lowV])
    higher = np.array([highH, highS, highV])
    mask = cv2.inRange(hsv, lower, higher)

    if green_filter:
        res_inv = cv2.bitwise_and(img, img, mask=cv2.bitwise_not(mask)) # comment when not using green filter
        rospy.set_param('/green_filter', True)
    else:
        res_inv = cv2.bitwise_and(img, img, mask=mask)
        rospy.set_param('/green_filter', False)

    # changing to gray color space
    gray = cv2.cvtColor(res_inv, cv2.COLOR_BGR2GRAY)

    # changing to black and white color space
    gray_lower = 127
    gray_upper = 255
    (dummy, blackAndWhiteImage) = cv2.threshold(gray, gray_lower, gray_upper, cv2.THRESH_BINARY)
    contours, dummy = cv2.findContours(blackAndWhiteImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    centers = []
    cx_list = []
    cy_list = []
    centroid_and_frame_width = []

    # plotting contours and their centroids
    for contour in contours:
        area = cv2.contourArea(contour)
        # print(area)  # uncomment for debug
        x, y, w, h = cv2.boundingRect(contour)
        # if min_area < area < max_area:
        if min_width < w < max_width:
            try:
                # print(area) # uncomment for debug
                x, y, w, h = cv2.boundingRect(contour)
                # img = cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
                img = cv2.drawContours(img, contour, -1, (0, 255, 0), 3)
                m = cv2.moments(contour)
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
        elif len(cx_list) == 1:
            mid_x = cx_list[0]
            mid_y = cy_list[0]
            cv2.circle(img, (mid_x, mid_y), 7, (255, 0, 0), -1)
    except ValueError:
        pass

    # plotting results
    try:
        cv2.imshow('img', img)
        cv2.imshow('mask', mask)
        cv2.imshow('res', res_inv)
        cv2.imshow('gray', gray)
        cv2.imshow('blackAndWhiteImage', blackAndWhiteImage)
        cv2.waitKey(1)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()

    steering_float = slider_to_normalized(steer_input)
    throttle_float = slider_to_normalized(throttle_input)

    steering_pub.publish(steering_float)
    throttle_pub.publish(throttle_float)


if __name__ == '__main__':
    green_filter_response = input("Create green filter? (y/n) ").upper()
    if green_filter_response == 'Y':
        green_filter = True
    else:
        green_filter = False
    rospy.init_node(CAMERA_VALUES_NODE_NAME, anonymous=False)
    camera_sub = rospy.Subscriber(CAMERA_TOPIC_NAME, Image, camera_values)
    steering_pub = rospy.Publisher(STEERING_TOPIC_NAME, Float32, queue_size=1)
    throttle_pub = rospy.Publisher(THROTTLE_TOPIC_NAME, Float32, queue_size=1)
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()
