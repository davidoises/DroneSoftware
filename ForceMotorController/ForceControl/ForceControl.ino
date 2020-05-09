#include "BLDCControl.h"

// HW Pins
#define PWMA 32
#define PWMB 33
#define PWMC 14
#define PWMD 10
#define SERVO1_X 13
#define SERVO1_Y 12
#define SERVO2_X 26
#define SERVO2_Y 25
#define ADCA 37
#define ADCB 38
#define ADCC 34
#define ADCD 35

// BLDC PID sampling time
#define BLDC_PID_SAMPLING 0.005f

// Create object with its corresponding interfaces
BLDCController bldc_a = BLDCController(ADCC, PWMC, 0, BLDC_PID_SAMPLING);

// Time measurement
hw_timer_t * pid_timer = NULL;

// Sampling ISR
void IRAM_ATTR pid_isr() {
  
  bldc_a.calculatePID();
  bldc_a.controlMotor();
  
  Serial.print(bldc_a.getSetpoint());
  Serial.print(" ");
  Serial.println(bldc_a.getForce());
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);

  // Initialize pwm
  bldc_a.init();

  // Wait for an "s" to start test
  while(!Serial.available());
  while(char(Serial.read()) != 's');
  
  pid_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(pid_timer, &pid_isr, true);
  timerAlarmWrite(pid_timer, BLDC_PID_SAMPLING*1000000.0, true);
  timerAlarmEnable(pid_timer);
}

void loop() {

  while(Serial.available())
  {
    double set_point = Serial.parseFloat();
    if(set_point != 0)
      bldc_a.setpoint(set_point);
  }
  
  //bldc_a.controlMotor();
}
