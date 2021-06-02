/*

    Mjolnir <-> Jetson
    Serial Interface Specification

    Jetson Nano (JTN) ALWAYS initiates communication

    Commands from JTN:
      
      t_XX
      > CommandThrottle
      > Commands Mjolnir to set Throttle PWM (RPM) to XX
      > XX is float32, range [-1.0, 1.0]

      s_XX
      > Command Steering
      > Commands Mjolnir to set Steering PWM (angle) to XX
      > XX is float32, range [-1.0, 1.0]

      h
      > Heartbeat
      > MUST be received periodically, otherwise assume master
        controller has disconnected, and performs E-Stop

      e
      > Emergency Stop
      > Commands Mjolnir to STOP all PWM outputs
        > Set throttle PWM pulse width to 1500 microseconds (neutral)
        > Set steering PWM pulse width to 1500 microseconds (0 degrees)
        > Note: to brake, ESC setting needs to be brake, not coast

      p
      > Poll Speed
      > Requests that Mjolnir send the motor RPM back to the JTN

    Commands to JTN from Mjolnir:
      r_XX
      > Sends the RPM of the motor back to the JTN
      > XX is uint32_t representing the RPM

    
    Note: Try not to contaminate the Serial interface for debug,
          to simplify parsing commands from the Jetson side


 */

static char rxChar;

void resetSerialBuffer() {
  rxBufPtr = 0;
  cmdReadyToParse = false;
  commandParsed   = false;

  // Flush the Arduino Serial buffer
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}

/*
 * Receives messages from the Serial interface and puts them
 * in a buffer, ready to be parsed
 */
void receiveSerialMessages() {
  // Do nothing if:
  // - There's no incoming serial data
  // - We have a command in the serial receive queue ready to be dispatched
  if (!Serial.available() || cmdReadyToParse)
    return;

  // char rxBuffer[256];
  // byte rxBufPtr = 0;
  // bool cmdReadyToParse = false;
  // bool commandParsed = false;

  while (Serial.available()) {
    rxChar = Serial.read();
    rxBuffer[rxBufPtr] = rxChar;

    if (rxChar == '\n') {
      // Stop receiving messages into the serial buffer, and indicate to the program
      // that we are ready to parse the current message in the buffer
      cmdReadyToParse = true;
      rxBuffer[rxBufPtr] = '\0';
      rxBufPtr++;
      return;
    }

    rxBufPtr++;
  }
}

void parseMessageAndDispatch() {
  if (!cmdReadyToParse)
    return;

  // Dispatch the received commands
  //Serial.print("Parsing ");
  //Serial.println(rxBuffer);
  //Serial.println(strcmp(rxBuffer, "requestSpeed"));

  switch (rxBuffer[0])
  {
    case 't': dispatch_setThrot();  break;
    case 's': dispatch_setSteer();  break;
    case 'p': dispatch_reqSpeed();  break;
    case 'h': dispatch_heartbeat(); break;
    case 'e': dispatch_stop();      break;

    //default:  Serial.println("Invalid command :(");
  }

  // Reset the parsing structures
  cmdReadyToParse = false;
  commandParsed = true;
  rxBufPtr = 0;

}

void dispatch_reqSpeed() {
  Serial.print("speedRPM_");
  Serial.println(carSpeedRPM);
}

void dispatch_setSteer() {
  float recvSteer = atof(&rxBuffer[2]);
  if (recvSteer < -1.0 || recvSteer > 1.0) return;

  uint32_t steerOut = (uint32_t) ((recvSteer * 500) + 1500);

  //Serial.println(rxBuffer);
  //Serial.println("Setting steering!");
  //Serial.println(recvSteer);
  //Serial.println(steerOut);
  pwmSteering.writeMicroseconds(steerOut);
}

void dispatch_setThrot() {
  float recvThrot = atof(&rxBuffer[2]);
  if (recvThrot < -1.0 || recvThrot > 1.0) return;

  uint32_t throttleOutput = (uint32_t) ((recvThrot * 500) + 1500);

  //Serial.println(rxBuffer);
  //Serial.println("Setting throttle!");
  //Serial.println(recvThrot);
  //Serial.println(throttleOutput);
  pwmThrottle.writeMicroseconds(throttleOutput);
}

void dispatch_heartbeat () {
  // TODO: Reset the heartbeat counter
}

void dispatch_stop() {
  //Serial.println("E-Stop!");
  //Serial.println(rxBuffer);

  // Setting throttle to reverse for a little bit actually forces the ESC
  // to brake
  pwmThrottle.writeMicroseconds(1300);
  pwmSteering.writeMicroseconds(1500);
  delay(300);
  pwmThrottle.writeMicroseconds(1500);
}
