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
#elif defined(ESP8266) or defined(ARDUINO_TEENSY40)
  #include <Servo.h>
#else
  #error "Code is low-level optimized for ESP8266/ESP32 or Teensy 4.0 only. Sorry :/"
#endif

#if not defined(ESP32)
  #warning "Code may not be fully compatible, it is optimized for ESP32 for final project"
#endif

// ------------------------------------------------------------
// I/O Pins
// ------------------------------------------------------------

#define sens_pinA  1
#define sens_pinB  2
#define sens_pinC  3

#define pinThrottle  5
#define pinSteering  7

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

// ------------------------------------------------------------
// Main Functions
// ------------------------------------------------------------

void setup() {
  initIO();

  //Serial.begin(115200);

  Serial.begin(460800);

}

void loop() {

  receiveSerialMessages();
  parseMessageAndDispatch();

  //int avgSpeed = (ticA + ticB + ticC) / 3;
  //Serial.println(avgSpeed);
  //Serial.print("ticA: ");
  //Serial.println(ticA);
  //Serial.print("ticB: ");
  //Serial.println(ticB);
  //Serial.print("ticC: ");
  //Serial.println(ticC);
  //Serial.println();

  //pwmThrottle.writeMicroseconds(1700);
  //pwmThrottle.writeMicroseconds(1500);

  //pwmSteering.writeMicroseconds(1500);
  //delay(500);
  //pwmSteering.writeMicroseconds(999);
  //delay(500);
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

  pinMode(pinThrottle, OUTPUT);
  pinMode(pinSteering, OUTPUT);
  pwmSteering.attach(pinSteering);

  pwmThrottle.writeMicroseconds(1500);
  pwmThrottle.writeMicroseconds(1500);
}


