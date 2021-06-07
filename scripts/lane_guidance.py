#!/usr/bin/env python
import rospy
import paho.mqtt.client as mqtt

from class_PIDController import PIDController
from std_msgs.msg import Float32, Int32MultiArray


LANE_GUIDANCE_NODE_NAME = 'lane_guidance_node'
STEERING_TOPIC_NAME = '/steering'
THROTTLE_TOPIC_NAME = '/throttle'
CENTROID_TOPIC_NAME = '/centroid'

KP_TOPIC_NAME = 'app/pid/kp'
KI_TOPIC_NAME = 'app/pid/ki'
KD_TOPIC_NAME = 'app/pid/kd'

steering_float = Float32()
throttle_float = Float32()

steeringPID = PIDController()

#whatt's up arthur
#'sup fade
def LineFollower(msg):
    centroid = msg.data[0]
    width = msg.data[1]  # width of camera frame
    throttle_float = 0.08

    steeringPID.tickPID(centroid, width / 2,  msg, throttle_float)

    steering_pub.publish(steeringPID.out)
    throttle_pub.publish(throttle_float)



def on_connect(client, userdata, flags, rc):
    client.subscribe(KP_TOPIC_NAME)
    client.subscribe(KI_TOPIC_NAME)
    client.subscribe(KD_TOPIC_NAME)


def on_message(client, userdata, msg):
    ''''''
    topic = str(msg.topic)
    payload = float(msg.payload.decode('utf-8'))
    #print("Message received : " + topic + " " + str(payload))

    if topic == KP_TOPIC_NAME:
        steeringPID.kp = payload
    elif topic == KI_TOPIC_NAME:
        steeringPID.ki = payload
    elif topic == KD_TOPIC_NAME:
        steeringPID.kd = payload
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
