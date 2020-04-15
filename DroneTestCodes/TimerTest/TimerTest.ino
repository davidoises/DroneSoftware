volatile uint8_t reached_timer;
int totalInterruptCounter;
 
hw_timer_t * timer = NULL;
 
void IRAM_ATTR onTimer() {
  reached_timer = 1;
 
}
 
void setup() {
 
  Serial.begin(115200);
 
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);
 
}
 
void loop() {
 
  if (reached_timer) {
 
    reached_timer = 0;
 
    totalInterruptCounter++;
 
    Serial.print("An interrupt as occurred. Total number: ");
    Serial.println(totalInterruptCounter);
 
  }
}
