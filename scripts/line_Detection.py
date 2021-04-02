#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from decoder import decodeImage

global mid_x, mid_y
mid_x = Int32()
mid_y = Int32()

pub = rospy.Publisher('/centroid', Int32, queue_size=1)


def video_detection(data):
    # decode image
    frame = decodeImage(data.data, data.height, data.width)

    # getting and setting image properties
    height, width, channels = frame.shape
    rows_to_watch = 50
    rows_offset = 150
    top_height = height - rows_offset
    bottom_height = top_height + rows_to_watch

    img = frame[top_height:bottom_height, 0:width]
    orig = img.copy()

    # experimentally found values from find_camera_values.py
    Hue_low = 20
    Hue_high = 50
    Saturation_low = 170
    Saturation_high = 255
    Value_low = 155
    Value_high = 255

    # changing color space to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # setting threshold limits for yellow color filter
    lower = np.array([Hue_low, Saturation_low, Value_low])
    upper = np.array([Hue_high, Saturation_high, Value_high])

    # creating mask
    mask = cv2.inRange(hsv, lower, upper)

    m = cv2.moments(mask, False)
    try:
        cx, cy = int(m['m10'] / m['m00']), int(m['m01'] / m['m00'])
    except ZeroDivisionError:
        cy, cx = int(height / 2), int(width / 2)

    # Publish centroid
    mid_x.data = cx
    mid_y.data = cy
    cv2.circle(img, (mid_x.data, mid_y.data), 7, (255, 0, 0), -1)
    pub.publish(mid_x)

    # plotting results
    try:
        cv2.imshow("original", orig)
        cv2.imshow("yellow mask", mask)
        cv2.imshow("plotting centroid", img)
        cv2.waitKey(1)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()


def main():
    rospy.init_node('lane_detection_node', anonymous=True)
    camera_sub = rospy.Subscriber('camera_rgb', Image, video_detection)
    rate = rospy.Rate(5)
    rospy.spin()



if __name__ == '__main__':
    main()
