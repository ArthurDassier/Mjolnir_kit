#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from decoder import decodeImage
import time

global mid_x
mid_x = Int32()
global mid_y
mid_y = Int32()

pub = rospy.Publisher('/centroid', Int32, queue_size=1)


def video_detection(data):
    frame = decodeImage(data.data, data.height, data.width)
    
    height, width, channels = frame.shape
    #frame = frame[50:height, 0:width]
    #frame = frame[height/2:height, 0:width]

    #translate = 0
    #rows_to_watch = 100
    #img = frame[(height/2 + translate):(height/2 + translate+rows_to_watch), 0:width]
    #img = frame[200:300, 0:width]
    height_extension = 0
    width_extension = 25
    img = frame[250:300, 200:650]
    #img = cv2.resize(img, (300, 100))
    orig = img.copy()
    # get rid of white noise from grass

    kernel = np.ones((5, 5), np.float32)/15
    blurred = cv2.filter2D(img, -1, kernel)
    dilation = cv2.dilate(blurred, kernel, iterations=5)

    # changing colorspace to HSV
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # setting threshold limits for color filter
    # lower = np.array([36, 25, 25])
    # upper = np.array([70, 255,255])
    lower = np.array([35, 0, 0])
    upper = np.array([85, 255, 255])
    #lower = np.array([60, 0, 0])
    #upper = np.array([100, 255, 255])

    # creating mask
    mask = cv2.inRange(hsv, lower, upper)
    res = cv2.bitwise_and(dilation, dilation, mask=mask)
    res = cv2.bitwise_not(res)

    # changing to gray colorspace
    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

    # changing to black and white color space
    (dummy, blackAndWhiteImage) = cv2.threshold(
        gray, 127, 255, cv2.THRESH_BINARY)

    med_pix = np.median(blackAndWhiteImage)
    canny_low = int(max(0,0.7*med_pix))
    canny_high = int(min(255,1.3*med_pix))

    edges = cv2.Canny(blackAndWhiteImage,canny_low,canny_high,apertureSize = 7)
    #edges = cv2.Canny(blackAndWhiteImage,30,200,apertureSize = 7)
    blurred = cv2.filter2D(edges, -1, kernel)
    # locating contours in image
    #ret, thresh = cv2.threshold(blurred, 240, 255, 0)
    contours, dummy = cv2.findContours(
        blackAndWhiteImage, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    #cv2.drawContours(thresh, contours, -1, (0,255,0),3)

    centers = []
    cx_list = []
    cy_list = []
    # plotting contours and their centroids
    for contour in contours:
        area = cv2.contourArea(contour)
        #print(area)
        if(area > 400 and area < 2000):
            #print(area)
            x, y, w, h = cv2.boundingRect(contour)
            img = cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
            m = cv2.moments(contour)
            cx = int(m['m10'] / m['m00'])
            cy = int(m['m01'] / m['m00'])
            centers.append([cx, cy])
            cx_list.append(int(m['m10'] / m['m00']))
            cy_list.append(int(m['m01'] / m['m00']))
            cv2.circle(img, (cx, cy), 7, (0, 255, 0), -1)
    # print(centers)
    
    try:
        mid_x.data = int(0.5 * (cx_list[0] + cx_list[1]))
        mid_y.data = int(0.5 * (cy_list[0] + cy_list[1]))
        cv2.circle(img, (mid_x.data, mid_y.data), 7, (255, 0, 0), -1)
        # print(mid_x)
    except:
        pass

    if (len(cx_list) == 0):
        mid_x.data = 0

    elif (len(cx_list) == 1 or len(cx_list) == 2):
        mid_x.data = (cx_list[0])
        mid_y.data = cy_list[0]
        cv2.circle(img, (mid_x.data, mid_y.data), 7, (255, 0, 0), -1)
        pub.publish(mid_x)

    
    time.sleep(0.08)
    try:

        # plotting results
        #cv2.imshow("original", orig)
        #cv2.imshow("dilation", dilation)
        cv2.imshow("blackAndWhiteImage", blackAndWhiteImage)
        cv2.imshow("contours_img", img)
        cv2.waitKey(1)
        #vid_write.write(img)
	


    except KeyboardInterrupt:
        cv2.destroyAllWindows()

#fourcc = cv2.VideoWriter_fourcc(*'avc1')
#vid_write = cv2.VideoWriter("/home/jetson/catkin_ws/src/potatoinside/movie/great_vid.mp4", #fourcc, 20.0, (400,50))

    



def main():

    rospy.init_node('lane_detection_node', anonymous=True)
    camera_sub = rospy.Subscriber('camera_rgb', Image, video_detection)
    rate = rospy.Rate(5)
    rospy.spin()

if __name__ == '__main__':
    main()
