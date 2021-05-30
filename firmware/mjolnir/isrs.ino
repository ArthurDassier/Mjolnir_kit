/*
   ISR Routines
 */

#if defined(ARDUINO_TEENSY40)
void isr_APulse() {
#else
void IRAM_ATTR isr_APulse() {
#endif
  //if (digitalRead(sens_pinB) == HIGH) { //Confirm if we are moving forward
  //  inReverse = false;
  //}
  //else { //Confirm if we are in reverse
  //  inReverse = true;
  //}

  //ticA = micros();
  //omegaA = ticA - tocA;
  //tocA = ticA;

  ticA ++;
}

#if defined(ARDUINO_TEENSY40)
void isr_BPulse() {
#else
void IRAM_ATTR isr_BPulse() {
#endif
  //ticB = micros();
  //omegaB = ticB - tocB;
  //tocB = ticB;

  ticB ++;
}

#if defined(ARDUINO_TEENSY40)
void isr_CPulse() {
#else
void IRAM_ATTR isr_CPulse() {
#endif
  //ticC = micros();
  //omegaC = ticC - tocC;
  //tocC = ticC;

  ticC ++;
}
