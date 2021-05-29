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
 |
 |    - Measures the speed of the main throttle BLDC via its 3 hall-effect encoder,
 |      and sends them to the Master SBC upon request
 |
 |    - Receives PWM commands from the Master SBC, and forwards them
 |      to the Throttle ESC/Steering Servo
 |
 |    - Communicates with PhoneApp via MQTT client to implement an emergency stop
 |
 |    - Ensures that the Master SBC sends a periodic heartbeat to remain operational,
 |      otherwise performs emergency stop
 |
 |    - Utilizes an internal Watchdog Timer to ensure it remains operational under
 |      unexpected software lockups
 |
*/

#if defined(ESP32)
  // ESP32 has special Servo library port for some reason BUT API is identical
  // https://github.com/jkb-git/ESP32Servo
  #include <ESP32Servo.h>
#elif defined(ESP8266)
  #include <Servo.h>
#else
  #error "Code is low-level optimized for ESP8266/ESP32 only. Sorry :/"
#endif

// ------------------------------------------------------------
// I/O Pins
// ------------------------------------------------------------

#define sens_pinA  2
#define sens_pinB  3
#define sens_pinC  4

// ------------------------------------------------------------
// ISR Declarations/Shared Variables
// ------------------------------------------------------------

void IRAM_ATTR isr_APulse();
void IRAM_ATTR isr_BPulse();
void IRAM_ATTR isr_CPulse();

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

// ------------------------------------------------------------
// Main Functions
// ------------------------------------------------------------

void setup() {
  initIO();

  Serial.begin(115200);

}

void loop() {

  //receiveSerialMessages();
  //parseMessageAndDispatch();

  int avgSpeed = (ticA + ticB + ticC) / 3;
  Serial.println(avgSpeed);
  
  delay(100);
}

// ------------------------------------------------------------
// Init Functions
// ------------------------------------------------------------

void initIO() {
  // Attach the interrupts that will read the BLDC encoder to measure the speed
  pinMode(sens_pinA, INPUT_PULLUP);
  pinMode(sens_pinB, INPUT_PULLUP);
  pinMode(sens_pinC, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sens_pinA), isr_APulse, RISING);
  attachInterrupt(digitalPinToInterrupt(sens_pinB), isr_BPulse, RISING);
  attachInterrupt(digitalPinToInterrupt(sens_pinC), isr_CPulse, RISING);
}


