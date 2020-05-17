#include <Wire.h>
#include "MPU6050.h"

MPU6050 mpu;

double initial_acc_z = 0;

void setup()
{
  Serial.begin(500000);
  delay(100);
  Serial.println("\r\n");
  delay(100);

  Wire.begin(); // ESP32 default SDa=21, SCL=22
  Wire.setClock(400000);

  Serial.print("MPU6050 Initialization result: ");
  Serial.println(mpu.begin(&Wire));

  for(int i = 0; i < 100; i++)
  {
    mpu.get_acc_data();  
    initial_acc_z += mpu.sensed.acc_z;
    delay(4);
  }
  initial_acc_z /= 100.0;
  Serial.println(initial_acc_z);
}

void loop()
{
  mpu.get_acc_data();
  Serial.print(mpu.sensed.acc_x);
  Serial.print(" ");
  Serial.print(mpu.sensed.acc_y);
  Serial.print(" ");
  Serial.println(mpu.sensed.acc_z-initial_acc_z);
  /*
  mpu.get_gyr_data();
  Serial.print(mpu.sensed.gyr_x);
  Serial.print(" ");
  Serial.print(mpu.sensed.gyr_y);
  Serial.print(" ");
  Serial.println(mpu.sensed.gyr_z);
  */
  delay(6);
}
