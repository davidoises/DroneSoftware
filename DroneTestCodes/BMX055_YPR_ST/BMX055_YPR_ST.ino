#include "BMX055.h"
#include "Math3D.h"
#include <math.h>

// BMX055 IMU addresses
#define AM_DEV 0x18
#define G_DEV 0x68
#define MAG_DEV 0x10

BMX055 imu = BMX055(AM_DEV, G_DEV, MAG_DEV, USE_MAG_CALIBRATION);

unsigned long prev_time = 0;
double pitch=0, roll=0, yaw=0;

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

  prev_time = millis();
}

void loop() {

  unsigned long current_time = millis();
  double t_delta = ((double)current_time - (double)prev_time)/1000;
  prev_time = current_time;

  imu.get_gyr_data();
  imu.get_acc_data();
  imu.get_mag_data();
  
  // x, y, z angles
  double gyr_roll = imu.gyroscope.x*imu.gyroscope.res*t_delta*(M_PI/180.0);
  double gyr_pitch = imu.gyroscope.y*imu.gyroscope.res*t_delta*(M_PI/180.0);
  double gyr_yaw = imu.gyroscope.z*imu.gyroscope.res*t_delta*(M_PI/180.0);

  double anorm = sqrt(pow(imu.accelerometer.x, 2) + pow(imu.accelerometer.y, 2) + pow(imu.accelerometer.z, 2));
  double ax = imu.accelerometer.x/anorm;
  double ay = imu.accelerometer.y/anorm;
  double az = imu.accelerometer.z/anorm;
    
  double acc_pitch = asin(-ax);
  double acc_roll = asin(ay/cos(acc_pitch));
  if(abs(acc_pitch) == PI/2.0)
  {
    acc_roll = 0;
  }

  // x, y complementary filter for angles
  pitch = 0.9*(pitch + gyr_pitch) + 0.1*acc_pitch;
  roll = 0.9*(roll + gyr_roll) + 0.1*acc_roll;

  double mag_pitch = -roll;
  double mag_roll = pitch;

  double mx = imu.magnetometer.x*cos(mag_pitch) + imu.magnetometer.z*sin(mag_pitch);
  double my = imu.magnetometer.x*sin(mag_roll)*sin(mag_pitch) + imu.magnetometer.y*cos(mag_roll) - imu.magnetometer.z*sin(mag_roll)*cos(mag_pitch);
  double mz = -imu.magnetometer.x*cos(mag_roll)*sin(mag_pitch) + imu.magnetometer.y*sin(mag_roll) + imu.magnetometer.z*cos(mag_roll)*cos(mag_pitch);
  double mag_yaw = atan(my/mx);
  if(mx > 0 && my >= 0)
  {
    mag_yaw = atan(my/mx);
  }
  else if(mx < 0)
  {
    mag_yaw = PI + atan(my/mx);
  }else if(mx > 0 && my < 0)
  {
    mag_yaw = 2*PI + atan(my/mx);
  }
  else if(mx == 0 && my < 0)
  {
    mag_yaw = PI/2.0;
  }
  else if(mx == 0 && my > 0)
  {
    mag_yaw = 3*PI/2.0;
  }

  /*Serial.print("y");
  Serial.print(attitude_ypr[0], 10);//* 180.0/3.141592653589793, 10);
  Serial.print(",");
  Serial.print(attitude_ypr[1], 10);//* 180.0/3.141592653589793, 10);
  Serial.print(",");
  Serial.print(attitude_ypr[2], 10);//* 180.0/3.141592653589793, 10);
  Serial.print("$\n");*/

  Serial.println(mag_yaw * 180.0/PI, 10);

  delay(50);
}
