#include "ble_conf.h"

double kp = 1;
double ki = 2;
double kd = 3;

void setup() {
  Serial.begin(115200);
  ble_kp = kp;
  ble_ki = ki;
  ble_kd = kd;
  ble_setup();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(ble_triggered)
  {
    kp = ble_kp;
    ki = ble_ki;
    kd = ble_kd;
    ble_triggered = 0;
  }
  Serial.print(kp);
  Serial.print(" ");
  Serial.print(ki);
  Serial.print(" ");
  Serial.println(kd);
}
