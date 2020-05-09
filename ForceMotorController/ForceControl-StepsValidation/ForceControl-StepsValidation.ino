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
hw_timer_t * testing_timer = NULL;

// Testing signal variables
double step_delay = 1500; // millisecods
unsigned long counter = 0;

// PID ISR
void IRAM_ATTR pid_isr() {
  
  bldc_a.calculatePID();
  bldc_a.controlMotor();
  
  Serial.print(bldc_a.getSetpoint());
  Serial.print(" ");
  Serial.println(bldc_a.getForce());
}

// Testing signal ISR
void IRAM_ATTR testing_isr() {
  switch(counter)
  {
    case 0:
      bldc_a.setpoint(2);
      break;
    case 1:
      bldc_a.setpoint(0);
      break;
    case 2:
      bldc_a.setpoint(3);
      break;
    case 3:
      bldc_a.setpoint(4);
      break;
    case 4:
      bldc_a.setpoint(2.5);
      break;
    case 5:
      bldc_a.setpoint(0);
      break;
    case 6:
      bldc_a.setpoint(1);
      break;
    case 7:
      bldc_a.setpoint(5);
      break;
    case 8:
      bldc_a.setpoint(4);
      break;
    case 9:
      bldc_a.setpoint(0);
      counter = -1;
      break;
  }
  counter++;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);

  // Initialize pwm
  bldc_a.init();

  // PID calculation update timer
  pid_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(pid_timer, &pid_isr, true);
  timerAlarmWrite(pid_timer, BLDC_PID_SAMPLING*1000000.0, true);

  // Testing signal calculation update timer
  testing_timer = timerBegin(1, 80, true);
  timerAttachInterrupt(testing_timer, &testing_isr, true);
  timerAlarmWrite(testing_timer, step_delay*1000.0, true);

  // Wait for an "s" to start test
  while(!Serial.available());
  while(char(Serial.read()) != 's');

  
  timerAlarmEnable(pid_timer);
  timerAlarmEnable(testing_timer);
}

void loop() {
  
  //bldc_a.controlMotor();
}
