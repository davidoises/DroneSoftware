#include "BMX055.h"
#include "Math3D.h"
#include <math.h>

// BMX055 IMU addresses
#define AM_DEV 0x18
#define G_DEV 0x68
#define MAG_DEV 0x10

BMX055 imu = BMX055(AM_DEV, G_DEV, MAG_DEV, USE_MAG_CALIBRATION);

unsigned long prev_time = 0;
double q0 = 1;
double q1 = 0, q2 = 0, q3 = 0;
double integralFBx, integralFBy, integralFBz;

#define kpx 1.0f
#define kix 0.25f
#define kpy 1.0f
#define kiy 0.25f
#define kpz 1.0f
#define kiz 1.5f

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

  double gx, gy, gz;
  double ax=0, ay=0, az=0;
  double mx, my, mz;
  double q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  double hx, hy, bx, bz;
  double halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
  double halfex, halfey, halfez;
  double qa, qb, qc;

  unsigned long current_time = millis();
  double t_delta = ((double)current_time - (double)prev_time)/1000;
  prev_time = current_time;

  // Get all raw measurements
  imu.get_gyr_data();
  imu.get_acc_data();
  imu.get_mag_data();

  // Get angular velocity in radians per second
  gx = imu.gyroscope.x*imu.gyroscope.res*(M_PI/180.0);
  gy = imu.gyroscope.y*imu.gyroscope.res*(M_PI/180.0);
  gz = imu.gyroscope.z*imu.gyroscope.res*(M_PI/180.0);

  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalized accelerometer measurements
    double anorm = sqrt(pow(imu.accelerometer.x, 2) + pow(imu.accelerometer.y, 2) + pow(imu.accelerometer.z, 2));
    ax = imu.accelerometer.x/anorm;
    ay = imu.accelerometer.y/anorm;
    az = imu.accelerometer.z/anorm;
  
    double mnorm = sqrt(pow(imu.magnetometer.x, 2) + pow(imu.magnetometer.y, 2) + pow(imu.magnetometer.z, 2));
    mx = imu.magnetometer.x/mnorm;
    my = imu.magnetometer.y/mnorm;
    mz = imu.magnetometer.z/mnorm;
  
    // Auxiliary variables to avoid repeated arithmetic
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
  
    // Reference direction of Earth's magnetic field
    hx = 2.0f *(mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f *(mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrt(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
  
    // Estimated direction of gravity and magnetic field
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
  
    // Error is sum of cross product between estimated direction
    // and measured direction of field vectors
    halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
    halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    integralFBx += kix * halfex * t_delta;
    integralFBy += kiy * halfey * t_delta;
    integralFBz += kiz * halfez * t_delta;
    gx += integralFBx; // apply integral feedback
    gy += integralFBy;
    gz += integralFBz;

    // Apply proportional feedback
    gx += kpx * halfex;
    gy += kpy * halfey;
    gz += kpz * halfez;

  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * t_delta); // pre-multiply common factors
  gy *= (0.5f * t_delta);
  gz *= (0.5f * t_delta);
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  double qnorm = sqrt(pow(q0, 2) + pow(q1, 2) + pow(q2, 2) + pow(q3, 2));
  q0 /= qnorm;
  q1 /= qnorm;
  q2 /= qnorm;
  q3 /= qnorm;

  Serial.print("y");
  Serial.print(q0, 10);
  Serial.print(",");
  Serial.print(q1, 10);
  Serial.print(",");
  Serial.print(q2, 10);
  Serial.print(",");
  Serial.print(q3, 10);
  Serial.print("$\n");

  /*double roll = atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
  double pitch = asin(-2.0f * (q1 * q3 - q0 * q2));
  double yaw = atan2(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);
  
  Serial.print("y");
  Serial.print(0, 10);
  Serial.print(",");
  Serial.print(0, 10);
  Serial.print(",");
  Serial.print(yaw, 10);
  Serial.print("$\n");*/

  delay(50);
}
