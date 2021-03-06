# Mjölnir Firmware

## ECE/MAE 148 SP21 Team 1

### Summary

Mjölnir is the name of the ESP32 on our car. It is responsible for a variety of tasks:
- Hosts a basic text interface over serial to receive commands from a master device (Jetson Nano or other SBC)
- Measures the speed of the main throttle BLDC via its 3 hall-effect encoder, and sends them to the Master SBC upon request
- Receives PWM commands from the Master SBC, and forwards them to the Throttle ESC/Steering Servo
- Communicates with PhoneApp via MQTT client to implement an emergency stop
- Ensures that the Master SBC sends a periodic heartbeat to remain operational, otherwise performs emergency stop
- Utilizes an internal Watchdog Timer to ensure it remains operational under unexpected software lockups

### Serial Interface Specification

#### Commands from Jetson to Mjolnir:
  
`commandThrottle_XX`
* Commands Mjolnir to set Throttle PWM (RPM) to XX
* XX is float32, range [-1.0, 1.0]

`commandSteering_XX`
* Commands Mjolnir to set Steering PWM (angle) to XX
* XX is float32, range [-1.0, 1.0]

`commandStop`
* Commands Mjolnir to STOP all PWM outputs
* Set throttle to 0 RPM (brake)
* Set steering to angle 90 (straight)
* Note: to brake, ESC setting needs to be brake, not coast

`pollSpeed`
* Requests that Mjolnir send the motor RPM to the JTN using the `responseSpeed_XX` message

#### Commands/Messages from Jetson to Mjolnir:
`responseSpeed_XX`
* Sends the RPM of the motor back to the JTN
* XX is uint32 representing the RPM

### Authors
[Georges Troulis](https://github.com/ayilay)
