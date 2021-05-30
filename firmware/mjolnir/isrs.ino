/*
   ISR Routines
 */

//void IRAM_ATTR isr_APulse() {
void isr_APulse() {
  //if (digitalRead(sens_pinB) == HIGH) { //Confirm if we are moving forward
  //  inReverse = false;
  //}
  //else { //Confirm if we are in reverse
  //  inReverse = true;
  //}

  ticA++;
  //ticA = micros();
  //omegaA = ticA - tocA;
  //tocA = ticA;
}

//void IRAM_ATTR isr_BPulse() {
void isr_BPulse() {
  ticB++;
  //ticB = micros();
  //omegaB = ticB - tocB;
  //tocB = ticB;
}

//void IRAM_ATTR isr_CPulse() {
void isr_CPulse() {
  ticC++;
  //ticC = micros();
  //omegaC = ticC - tocC;
  //tocC = ticC;
}
