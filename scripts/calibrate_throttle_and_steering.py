#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import Float32

MOTOR_VALUES_NODE_NAME = 'motor_values_node'
STEERING_TOPIC_NAME = '/steering'
THROTTLE_TOPIC_NAME = '/throttle'

global steering_float, throttle_float
steering_float = Float32()
throttle_float = Float32()

cv2.namedWindow('motor values')


def callback(x):
    pass


steer_left = 0
steer_straight = 1000
steer_right = 2000
throttle_reverse = 0
throttle_neutral = 0
throttle_forward = 2000

cv2.createTrackbar('Steering_value', 'sliders', steer_straight, steer_right, callback)
cv2.createTrackbar('Throttle_value', 'sliders', throttle_neutral, throttle_forward, callback)


def motor_values():
    global steering_float, throttle_float
    steering_float = Float32()
    throttle_float = Float32()

    motor_dict = {
        "Steering_max_left": 0.0,
        "Steering_straight": 0.0,
        "Steering_max_right": 0.0,
        "Throttle_max_forward": 0.0,
        "Throttle_neutral": 0.0,
        "Throttle_max_reverse": 0.0
    }

    steer_input = cv2.getTrackbarPos('Steering_value', 'sliders')
    steering_float = slider_to_normalized(steer_input)

    throttle_input = cv2.getTrackbarPos('Throttle_value', 'sliders')
    throttle_float = slider_to_normalized(throttle_input)

    rospy.set_param('/Steering_max_left', motor_dict["Steering_max_left"])
    rospy.set_param('/Steering_straight', motor_dict["Steering_straight"])
    rospy.set_param('/Steering_max_right', motor_dict["Steering_max_right"])
    rospy.set_param('/Throttle_max_forward', motor_dict["Throttle_max_forward"])
    rospy.set_param('/Throttle_max_forward', motor_dict["Throttle_neutral"])
    rospy.set_param('/Throttle_max_reverse', motor_dict["Throttle_max_reverse"])

    # Write files to yaml file for storage
    color_config_path = "../config/motor_parameters/motor_values.yaml"
    f = open(color_config_path, "w")
    f.write(f"Steering_max_left : {motor_dict['Steering_max_left']} \n"
            f"Steering_straight : {motor_dict['Steering_straight']} \n"
            f"Steering_max_right : {motor_dict['Steering_max_right']} \n"
            f"Throttle_max_forward : {motor_dict['Throttle_max_forward']} \n"
            f"Throttle_neutral : {motor_dict['Throttle_neutral']} \n"
            f"Throttle_max_reverse : {motor_dict['Throttle_max_reverse']} \n")
    f.close()

    key = cv2.waitKey(0)
    if key == ord('l'):
        motor_dict["Steering_max_left"] = steering_float
    elif key == ord('s'):
        motor_dict["Steering_straight"] = steering_float
    elif key == ord('r'):
        motor_dict["Steering_max_right"] = steering_float
    elif key == ord('f'):
        motor_dict["Throttle_max_forward"] = throttle_float
    elif key == ord('n'):
        motor_dict["Throttle_neutral"] = throttle_float
    elif key == ord('b'):
        motor_dict["Throttle_max_reverse"] = throttle_float
    elif key == ord('e'):
        pass

    steering_pub.publish(steering_float)
    throttle_pub.publish(throttle_float)


def slider_to_normalized(slider_input):
    input_start = 0
    input_end = 2000
    output_start = -1
    output_end = 1
    normalized_output = output_start + (slider_input - input_start) * ((output_end - output_start) / (input_end - input_start))
    return normalized_output


if __name__ == '__main__':
    response = input("Is car on test stand (y/n) ").upper()
    if response == 'Y':
        while True:
            motor_values()
            rospy.init_node(MOTOR_VALUES_NODE_NAME, anonymous=False)
            steering_pub = rospy.Publisher(STEERING_TOPIC_NAME, Float32, queue_size=1)
            throttle_pub = rospy.Publisher(THROTTLE_TOPIC_NAME, Float32, queue_size=1)
            rate = rospy.Rate(15)
    else:
        print("Put car on test stand before calibrating and restart!")