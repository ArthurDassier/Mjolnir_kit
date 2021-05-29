/*
   ISR Routines
 */

void IRAM_ATTR isr_APulse() {
  //if (digitalRead(sens_pinB) == HIGH) { //Confirm if we are moving forward
  //  inReverse = false;
  //}
  //else { //Confirm if we are in reverse
  //  inReverse = true;
  //}

  ticA = micros();
  omegaA = ticA - tocA;
  tocA = ticA;
}

void IRAM_ATTR isr_BPulse() {
  ticB = micros();
  omegaB = ticB - tocB;
  tocB = ticB;
}

void IRAM_ATTR isr_CPulse() {
  ticC = micros();
  omegaC = ticC - tocC;
  tocC = ticC;
}
