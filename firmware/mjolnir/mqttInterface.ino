#include "defines.h"

void mqttReconnect() {

  const char* deviceID = "Mjolnir-ESP32";

  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection to ");
    Serial.print(MQTT_SERV_IP);
    Serial.print(" on port ");
    Serial.print(MQTT_PORT);
    Serial.print(" with client ID ");
    Serial.println(deviceID);

    // Attempt to connect
    if (mqttClient.connect(deviceID)) {
      Serial.println("MQTT Connected Successfully");

      mqttClient.subscribe(TOPIC_APP_ESTOP);
      mqttClient.subscribe(TOPIC_APP_RESUME);

    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/*
 *  MQTT callback function, called whenever a message is received on
 *  a subscribed topic
 *
 *  Parse the topic
 */
void mqttMsgRecvCallback(char* topic, byte* msgRaw, unsigned int length) {
  Serial.print("[");
  Serial.print(topic);
  Serial.print("]: ");
  // Convert to String for easier parsing
  String message;
  for (int i = 0; i < length; i++) {
    message += (char) msgRaw[i];
  }
  Serial.println(message);

  if      (String(topic) == TOPIC_APP_ESTOP)   dispatch_appEstop (msgRaw, length);
  else if (String(topic) == TOPIC_APP_RESUME)  dispatch_appResume(msgRaw, length);
}

void dispatch_appEstop(byte* msgRaw, unsigned int length) {
  Serial.println("MQTT Dispatch E-Stop!");
  mjolnir_state = MJOLNIR_ESTOP;
  dispatch_stop();

  mqttClient.publish(TOPIC_MJOLNIR_MODE, "ESTOP");
}

void dispatch_appResume(byte* msgRaw, unsigned int length) {
  Serial.println("MQTT Dispatch Resume");

  resetSerialBuffer();
  mjolnir_state = MJOLNIR_ACTIVE;
  mqttClient.publish(TOPIC_MJOLNIR_MODE, "NORMAL");
}

