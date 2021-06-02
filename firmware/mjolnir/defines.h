#ifndef MJOLNIR_DEFS_H
#define MJOLNIR_DEFS_H

// WiFi and MQTT libraries for ESP32
#include <WiFi.h>
#include <PubSubClient.h>

#define SSID "SD-DIYRoboCar"
#define PSWD "SDrobocars2017"
#define MQTT_SERV_IP "ucsdrobocar-148-77"
#define MQTT_PORT 1883

#define TOPIC_MJOLNIR_MODE  "mjolnir/mode"
#define TOPIC_APP_ESTOP     "app/estop"
#define TOPIC_APP_RESUME    "app/resume"

// TODO these are nonsense atm, fix soon
#define sens_pinA  10
#define sens_pinB  11
#define sens_pinC  12

#define pinThrottle  2
#define pinSteering  4

#endif
