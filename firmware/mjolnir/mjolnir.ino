/*
 |  mjolnir.ino
 |
 |  ECE/MAE 148 SP21 Team 1
 |  Author: George Troulis <gtroulis@ucsd.edu>
 |
 |  Based on the Teensy firmware from TritonAI, heavily modified and
 |  ported to ESP32.
 |  
 |
 |  Mjölnir is the name of the ESP32 that lies on our ECE/MAE 148
 |  final project. It is connected to the Jetson Nano SBC via a USB
 |  cable, and communicates with it via an emulated UART (Serial).
 |
 |  Mjölnir is responsible for the following tasks:
 |    - Hosts a basic text interface over serial to receive commands
 |      from a master device (Jetson Nano or other SBC)
 |    - Measures the speed of the main throttle BLDC via its 3 hall-effect encoder,
 |      and sends them to the Master SBC upon request
 |    - Receives PWM commands from the Master SBC, and forwards them
 |      to the Throttle ESC/Steering Servo
 |    - Communicates with PhoneApp via MQTT client to implement an emergency stop
 |    - Ensures that the Master SBC sends a periodic heartbeat to remain operational,
 |      otherwise performs emergency stop
 |    - Utilizes an internal Watchdog Timer to ensure it remains operational under
 |      unexpected software lockups
 |
 |  I/O Connections on ESP32:
 |    Red:       3V3
 |    Black:     GND
 |    Orange: Pin 17
 |    Yellow: Pin 16
 |    Green:  Pin  4
 |    Blue:   Pin  0 // DON'T USE THIS, Doesn't work
 |    Brown:  Pin  2
#00b100ff
 |
*/

#if defined(ESP32)
  // ESP32 has special Servo library port for some reason BUT API is identical
  // https://github.com/jkb-git/ESP32Servo
  #include <ESP32Servo.h>
#elif defined(ESP8266) or defined(ARDUINO_TEENSY40)
  #include <Servo.h>
#else
  #error "Code is low-level optimized for ESP8266/ESP32 or Teensy 4.0 only. Sorry :/"
#endif

#if not defined(ESP32)
  #warning "Code may not be fully compatible, it is optimized for ESP32 for final project"
#endif

#include "defines.h"

// ------------------------------------------------------------
// WiFi/MQTT Definitions
// ------------------------------------------------------------

// The MQTT Client objects
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// ------------------------------------------------------------
// I/O Pins
// ------------------------------------------------------------

Servo pwmThrottle;
Servo pwmSteering;

// ------------------------------------------------------------
// ISR Declarations/Shared Variables
// ------------------------------------------------------------

#if defined(ESP32) or defined(ESP8266)
  void IRAM_ATTR isr_APulse();
  void IRAM_ATTR isr_BPulse();
  void IRAM_ATTR isr_CPulse();
#else
  void isr_APulse();
  void isr_BPulse();
  void isr_CPulse();
#endif

volatile unsigned long ticA = 0;
volatile unsigned long ticB = 0;
volatile unsigned long ticC = 0;
volatile unsigned long tocA = 0;
volatile unsigned long tocB = 0;
volatile unsigned long tocC = 0;
volatile unsigned long omegaA = 0;
volatile unsigned long omegaB = 0;
volatile unsigned long omegaC = 0;

// ------------------------------------------------------------
// Serial Interface data
// ------------------------------------------------------------

char rxBuffer[256];
byte rxBufPtr = 0;
bool cmdReadyToParse = false;
bool commandParsed = false;

// ------------------------------------------------------------
// Global State Variables
// ------------------------------------------------------------

// TODO for testing, initialize to zero after testing done
uint32_t carSpeedRPM = 65535;

enum {
  MJOLNIR_ACTIVE,
  MJOLNIR_ESTOP,
  MJOLNIR_REMOTE_OVERRIDE
} mjolnir_state;

// ------------------------------------------------------------
// Main Functions
// ------------------------------------------------------------

void setup() {
  initIO();
  initMQTT();

  mjolnir_state = MJOLNIR_ACTIVE;

  //Serial.begin(115200);

  Serial.begin(500000);

}

void loop() {


  // Force reconnect if disconnected
  // TODO Make sure this does not interfere w/ Real-Time tasks
  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  mqttClient.loop();

  switch(mjolnir_state) {

    case MJOLNIR_ACTIVE:
      receiveSerialMessages();
      parseMessageAndDispatch();
      break;

    case MJOLNIR_ESTOP:
      break;

    case MJOLNIR_REMOTE_OVERRIDE:
      break;
  }
}

// ------------------------------------------------------------
// Init Functions
// ------------------------------------------------------------

void initIO() {
  // Attach the interrupts that will read the BLDC encoder to measure the speed
  //pinMode(sens_pinA, INPUT_PULLUP);
  //pinMode(sens_pinB, INPUT_PULLUP);
  //pinMode(sens_pinC, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(sens_pinA), isr_APulse, RISING);
  //attachInterrupt(digitalPinToInterrupt(sens_pinB), isr_BPulse, RISING);
  //attachInterrupt(digitalPinToInterrupt(sens_pinC), isr_CPulse, RISING);

  pinMode(pinThrottle, OUTPUT);
  pinMode(pinSteering, OUTPUT);
  pwmSteering.attach(pinSteering);
  pwmThrottle.attach(pinThrottle);
  pwmThrottle.writeMicroseconds(1500);
  pwmSteering.writeMicroseconds(1500);
}

void initMQTT() {
  Serial.print(F("Connecting to network "));
  Serial.println(SSID);

  WiFi.begin(SSID, PSWD);

  unsigned int retry = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");

    if (retry++ > 10) {
      Serial.println("");
      Serial.print("Network failed after several retries");
      ESP.restart();
    }
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("Mjolnir IP address: ");
  Serial.println(WiFi.localIP());

  mqttClient.setServer(MQTT_SERV_IP, MQTT_PORT);
  mqttClient.setCallback(mqttMsgRecvCallback);
}
