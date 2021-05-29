/*

    Mjolnir <-> Jetson
    Serial Interface Specification

    Commands from JTN:
      
      commandThrottle_XX
      > Commands Mjolnir to set Throttle PWM (RPM) to XX
      > XX is float32, range [-1.0, 1.0]

      commandSteering_XX
      > Commands Mjolnir to set Steering PWM (angle) to XX
      > XX is float32, range [-1.0, 1.0]

      commandStop
      > Commands Mjolnir to STOP all PWM outputs
        > Set throttle to 0 RPM (brake)
        > Set steering to angle 90 (straight)
        > Note: to brake, ESC setting needs to be brake, not coast

      pollSpeed
      > Requests that Mjolnir send the motor RPM back to the JTN

    Commands to JTN from Mjolnir:
      responseSpeed_XX
      > Sends the RPM of the motor back to the JTN
      > XX is uint32_t representing the RPM


 */

static char rxChar;

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

       if (strncmp(rxBuffer, "pollSpeed",        9) == 0) dispatch_reqSpeed();
  else if (strncmp(rxBuffer, "commandSteering", 15) == 0) dispatch_setSteer();
  else if (strncmp(rxBuffer, "commandThrottle", 15) == 0) dispatch_setThrot(); 
  else if (strncmp(rxBuffer, "commandShutdown", 15) == 0) dispatch_stop();

  else Serial.println("Invalid command :(");

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
  Serial.println("Setting steering! TODO");
  Serial.println(rxBuffer);
}

void dispatch_setThrot() {
  Serial.println("Setting throttle! TODO");
  Serial.println(rxBuffer);
}

void dispatch_stop() {
  Serial.println("E-Stop! TODO");
  Serial.println(rxBuffer);
}
