#include "BMX055.h"
#include "Math3D.h"
#include <math.h>
#include "SensorFusion.h"

// BMX055 IMU addresses
#define AM_DEV 0x18
#define G_DEV 0x68
#define MAG_DEV 0x10

BMX055 imu = BMX055(AM_DEV, G_DEV, MAG_DEV, USE_MAG_CALIBRATION);
NXPSensorFusion filter;

float roll = 0, pitch = 0, yaw = 0;

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
  
  imu.acc_init();
  imu.gyr_init();
  imu.mag_init();
  delay(1000);

  filter.begin(100);
}

void loop() {

  // Get all raw measurements
  imu.get_gyr_data();
  imu.get_acc_data();
  imu.get_mag_data();

  // All measurement variables
  float gx = imu.gyroscope.x*imu.gyroscope.res, gy = imu.gyroscope.y*imu.gyroscope.res, gz = imu.gyroscope.z*imu.gyroscope.res;
  float ax = imu.accelerometer.x*imu.accelerometer.res, ay = imu.accelerometer.y*imu.accelerometer.res, az = imu.accelerometer.z*imu.accelerometer.res;
  float mx = imu.magnetometer.y*imu.magnetometer.res, my = imu.magnetometer.x*imu.magnetometer.res, mz = -imu.magnetometer.z*imu.magnetometer.res;

  // Update the SensorFusion filter
  filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

  /*Serial.print("y");
  Serial.print(q0, 10);
  Serial.print(",");
  Serial.print(q1, 10);
  Serial.print(",");
  Serial.print(q2, 10);
  Serial.print(",");
  Serial.print(q3, 10);
  Serial.print("$\n");*/

  roll = filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw();
  
  Serial.print("y");
  Serial.print(pitch*(M_PI/180.0), 10);
  Serial.print(",");
  Serial.print(roll*(M_PI/180.0), 10);
  Serial.print(",");
  Serial.print(yaw*(M_PI/180.0), 10);
  Serial.print("$\n");

  delay(50);
}
