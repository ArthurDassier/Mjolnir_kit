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

       if (strncmp(rxBuffer, "requestSpeed",    12) == 0) dispatch_reqSpeed();
  else if (strncmp(rxBuffer, "commandSteering", 15) == 0) dispatch_setSteer();
  else if (strncmp(rxBuffer, "commandThrottle", 15) == 0) dispatch_setThrot(); 
  else if (strncmp(rxBuffer, "commandShutdown", 15) == 0) dispatch_stop();
  else {
    Serial.println("Invalid command :( ");
  }

  // Reset the parsing structures
  cmdReadyToParse = false;
  commandParsed = true;
  rxBufPtr = 0;

}

void dispatch_reqSpeed() {
  Serial.println("Sending speed! TODO");
  Serial.println(rxBuffer);
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
