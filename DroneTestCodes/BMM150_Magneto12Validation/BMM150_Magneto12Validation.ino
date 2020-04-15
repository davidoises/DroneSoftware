#include "BMX055.h"
#include "Math3D.h"
#include <math.h>

// BMX055 IMU addresses
#define AM_DEV 0x18
#define G_DEV 0x68
#define MAG_DEV 0x10

BMX055 imu = BMX055(AM_DEV, G_DEV, MAG_DEV, USE_MAG_CALIBRATION);

unsigned long prev_time = 0;
double attitude_ypr[3] = {0};

void setup() {
  Serial.begin(115200);
  //Serial.begin(1000000);
  
  delay(100);
  Serial.println("\r\n");
  delay(100);
  
  //Wire.begin(SDA,SCL)
  //Wire.begin(D4, D5); // ESP8266
  //Wire.begin(D1, D2); // ESP8266
  //Wire.begin(33,32); // ESP32
  Wire.begin(); // ESP32 default SDa=21, SCL=22
  Wire.setClock(400000);
  
  //imu.acc_init();
  //imu.gyr_init();
  imu.mag_init();
  delay(1000);

  prev_time = millis();
}

void loop() {
  while(Serial.available())
  {
    if(Serial.read() == 0x22)
    {
      unsigned long current_time = millis();
      double t_delta = ((double)current_time - (double)prev_time)/1000;
      prev_time = current_time;
    
      imu.get_mag_data();
      
      Serial.print(imu.magnetometer.x); 
      Serial.print(",");
      Serial.print(imu.magnetometer.y); 
      Serial.print(",");
      Serial.print(imu.magnetometer.z);
      Serial.println("");
    }
  }
}
