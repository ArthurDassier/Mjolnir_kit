#!/usr/bin/env python
import rospy
import paho.mqtt.client as mqtt

from class_PIDController.py import PIDController
from std_msgs.msg import Float32, Int32, Int32MultiArray


LANE_GUIDANCE_NODE_NAME = 'lane_guidance_node'
STEERING_TOPIC_NAME = '/steering'
THROTTLE_TOPIC_NAME = '/throttle'
CENTROID_TOPIC_NAME = '/centroid'

KP_TOPIC_NAME = 'app/pid/kp'
KI_TOPIC_NAME = 'app/pid/ki'
KD_TOPIC_NAME = 'app/pid/kd'

steering_float = Float32()
throttle_float = Float32()

PIDController = PIDController()

#whatt's up arthur
#'sup fade
def LineFollower(msg):
    centroid = msg.data[0]
    width = msg.data[1]  # width of camera frame

    if msg.data == 0:
        error_x = 0
        throttle_float = 0.08 #previous value is 0.1
    else:
        error_x = float(centroid - (width / 2))
        throttle_float = 0.08 #previous values is 0.1

    PIDController.integrator = float(PIDController.kp * (error_x / (width / 2)))
    if PIDController.integrator < -1:
        PIDController.integrator = -1
    elif PIDController.integrator > 1:
        PIDController.integrator = 1
    else:
        pass

    # Derivative (band-limited differentiator)
    PIDController.differentiator = -(2.0 * PIDController.kd #* (measurement - PIDController.prevMeasurement)
    + (2.0 * PIDController.tau - PIDController.T) * PIDController.differentiator) / (2.0 * PIDController.tau + PIDController.T)

    #Compute output and apply limits
    PIDController.out = PIDController.kp * error_x + PIDController.integrator + PIDController.differentiator
    if (PIDController.out > PIDController.limMax):
        PIDController.out = PIDController.limMax
    elif (PIDController.out < PIDController.limMin):
        PIDController.out = PIDController.limMin

    steering_pub.publish(PIDController.out)
    throttle_pub.publish(throttle_float)


def on_connect(client, userdata, flags, rc):
    client.subscribe(KP_TOPIC_NAME)
    client.subscribe(KI_TOPIC_NAME)
    client.subscribe(KD_TOPIC_NAME)


def on_message(client, userdata, msg):
    ''''''
    topic = str(msg.topic)
    payload = msg.payload.decode('utf-8')
    print("Message received : " + topic + " " + payload)

    if topic == KP_TOPIC_NAME:
        print("Message KP_TOPIC_NAME DETECTED")
        kp = payload
    elif topic == KI_TOPIC_NAME:
        print("Message KI_TOPIC_NAME DETECTED")
        ki = payload
    elif topic == KD_TOPIC_NAME:
        print("Message KD_TOPIC_NAME DETECTED")
        kd = payload
    else:
        print("! ! ! UNKNOWN TOPIC NAME ! ! !")


if __name__ == '__main__':
    ''''''
    rospy.init_node(LANE_GUIDANCE_NODE_NAME, anonymous=False)

    mqtt_client = mqtt.Client("digi_mqtt_test")
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    mqtt_client.connect("ucsdrobocar-148-77", 1883)
    mqtt_client.loop_start()

    centroid_subscriber = rospy.Subscriber(CENTROID_TOPIC_NAME, Int32MultiArray, LineFollower)
    steering_pub = rospy.Publisher(STEERING_TOPIC_NAME, Float32, queue_size=1)
    throttle_pub = rospy.Publisher(THROTTLE_TOPIC_NAME, Float32, queue_size=1)
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()
