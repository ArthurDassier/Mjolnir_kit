#!/usr/bin/env python
import cv2
import numpy as np


def callback(x):
    pass


cap = cv2.VideoCapture(0)

cv2.namedWindow('sliders')

lowH = 0
highH = 179
lowS = 0
highS = 255
lowV = 0
highV = 255


cv2.createTrackbar('lowH', 'sliders', lowH, highH, callback)
cv2.createTrackbar('highH', 'sliders', lowH, highH, callback)

cv2.createTrackbar('lowS', 'sliders', lowS, highS, callback)
cv2.createTrackbar('highS', 'sliders', highS, highS, callback)

cv2.createTrackbar('lowV', 'sliders', lowV, highV, callback)
cv2.createTrackbar('highV', 'sliders', highV, highV, callback)

cv2.createTrackbar('blur_value', 'sliders', 1, 255, callback)
cv2.createTrackbar('blur_kernal', 'sliders', 1, 255, callback)
cv2.createTrackbar('dilation_value', 'sliders', 1, 255, callback)

while True:
    ret, frame = cap.read()
    # frame = cv2.imread('image.png')
    blur_value = cv2.getTrackbarPos('blur_value', 'sliders')
    dilation_value = cv2.getTrackbarPos('dilation_value', 'sliders')
    blur_kernal_value = cv2.getTrackbarPos('blur_kernal', 'sliders')
    kernel = np.ones((blur_kernal_value, blur_kernal_value), np.float32) / blur_value
    blurred = cv2.filter2D(frame, -1, kernel)
    dilation = cv2.dilate(blurred, kernel, iterations=dilation_value)

    # changing colorspace to HSV


    # get trackbar positions
    lowH = cv2.getTrackbarPos('lowH', 'sliders')
    highH = cv2.getTrackbarPos('highH', 'sliders')
    lowS = cv2.getTrackbarPos('lowS', 'sliders')
    highS = cv2.getTrackbarPos('highS', 'sliders')
    lowV = cv2.getTrackbarPos('lowV', 'sliders')
    highV = cv2.getTrackbarPos('highV', 'sliders')

    # hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([lowH, lowS, lowV])
    higher = np.array([highH, highS, highV])
    mask = cv2.inRange(hsv, lower, higher)

    res = cv2.bitwise_and(dilation, dilation, mask=mask)

    # changing to gray colorspace
    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

    # changing to black and white color space
    gray_lower = 127
    gray_upper = 255
    (dummy, blackAndWhiteImage) = cv2.threshold(gray, gray_lower, gray_upper, cv2.THRESH_BINARY)

    cv2.imshow('frame', frame)
    cv2.imshow('mask', mask)
    cv2.imshow('dilation', dilation)
    cv2.imshow('blackAndWhiteImage', blackAndWhiteImage)

    key = cv2.waitKey(100) & 0xFF
    if key == ord('q'):
        break

cv2.destroyAllWindows()
# cap.release()