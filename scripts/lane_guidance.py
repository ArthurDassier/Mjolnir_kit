#!/usr/bin/env python
import rospy
import paho.mqtt.client as mqtt

from std_msgs.msg import Float32, Int32, Int32MultiArray


LANE_GUIDANCE_NODE_NAME = 'lane_guidance_node'
STEERING_TOPIC_NAME = '/steering'
THROTTLE_TOPIC_NAME = '/throttle'
CENTROID_TOPIC_NAME = '/centroid'

KP_TOPIC_NAME = 'app/pid/kp'
KI_TOPIC_NAME = 'app/pid/ki'
KD_TOPIC_NAME = 'app/pid/kd'

global steering_float, throttle_float
steering_float = Float32()
throttle_float = Float32()

#whatt's up arthur
#'sup fade
def LineFollower(msg):
    kp = 0.80
    global steering_float, throttle_float
    steering_float = Float32()
    throttle_float = Float32()
    centroid = msg.data[0]
    width = msg.data[1]  # width of camera frame

    if msg.data == 0:
        error_x = 0
        throttle_float = 0.08 #previous value is 0.1
    else:
        error_x = float(centroid - (width / 2))
        throttle_float = 0.08 #previous values is 0.1

    steering_float = float(kp * (error_x / (width / 2)))
    if steering_float < -1:
        steering_float = -1
    elif steering_float > 1:
        steering_float = 1
    else:
        pass
    steering_pub.publish(steering_float)
    throttle_pub.publish(throttle_float)


def on_connect(client, userdata, flags, rc):
    client.subscribe(KP_TOPIC_NAME)
    client.subscribe(KI_TOPIC_NAME)
    client.subscribe(KD_TOPIC_NAME)


def on_message(client, userdata, msg):
    ''''''
    print("Message received : " + str(msg.topic) + " " + str(msg.payload))


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
