#include "BLDCControl.h"

// HW Pins
#define PWMA 32
#define PWMB 33
#define PWMC 14
#define PWMD 10
#define ADC_PINA 35

// BLDC PID sampling time
#define BLDC_PID_SAMPLING 0.005f

// Create object with its corresponding interfaces
BLDCController bldc_a = BLDCController(ADC_PINA, PWMA, 0, BLDC_PID_SAMPLING);

// Time measurement
hw_timer_t * pid_timer = NULL;
hw_timer_t * testing_timer = NULL;

// Testing signal variables
double step_delay = 50; // millisecods
unsigned long counter = 0;

// PID ISR
void IRAM_ATTR pid_isr() {
  
  bldc_a.calculatePID();
  bldc_a.controlMotor();
  
  Serial.print(bldc_a.getSetpoint());
  Serial.print(" ");
  Serial.println(bldc_a.getVoltage());
}

// Testing signal ISR
void IRAM_ATTR testing_isr() {
  // sine from 1100 to 1300 pwm signal
  double set_point = 8.0*sin(counter/15.0) + 14.0;
  bldc_a.setpoint(set_point);
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
