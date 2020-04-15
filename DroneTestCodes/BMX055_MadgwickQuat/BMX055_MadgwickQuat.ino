#include "BMX055.h"
#include "Math3D.h"
#include <math.h>

// BMX055 IMU addresses
#define AM_DEV 0x18
#define G_DEV 0x68
#define MAG_DEV 0x10

// 2 * Proportional Gain
#define beta1 0.1f
#define beta2 0.1f
#define beta3 0.1f
#define beta4 1.0f

BMX055 imu = BMX055(AM_DEV, G_DEV, MAG_DEV, USE_MAG_CALIBRATION);

unsigned long prev_time = 0;

float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

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

  // Variables for arithmetic calculations for quaternions
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1,_2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3,q2q2, q2q3, q3q3;

  // Elpased time for time related operation
  unsigned long current_time = millis();
  double t_delta = ((double)current_time - (double)prev_time)/1000;
  prev_time = current_time;

  // Get all raw measurements
  imu.get_gyr_data();
  imu.get_acc_data();
  imu.get_mag_data();

  // All measurement variables
  float gx = imu.gyroscope.x, gy = imu.gyroscope.y, gz = imu.gyroscope.z;
  float ax = imu.accelerometer.x, ay = imu.accelerometer.y, az = imu.accelerometer.z;
  float mx = imu.magnetometer.x, my = imu.magnetometer.y, mz = imu.magnetometer.z;

  // Get angular velocity in radians per second
  gx *= imu.gyroscope.res*(M_PI/180.0);
  gy *= imu.gyroscope.res*(M_PI/180.0);
  gz *= imu.gyroscope.res*(M_PI/180.0);

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalized accelerometer measurements
    float anorm = sqrtf(powf(ax, 2) + powf(ay, 2) + powf(az, 2));
    ax /= anorm;
    ay /= anorm;
    az /= anorm;
  
    float mnorm = sqrtf(powf(mx, 2) + powf(my, 2) + powf(mz, 2));
    mx /= mnorm;
    my /= mnorm;
    mz /= mnorm;
  
    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
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

    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;
  
    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) +
         _2q1 * (2.0f * q0q1 + _2q2q3 - ay) -
         _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (-_2bx * q3 + _2bz * q1) *
             (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) +
         _2q0 * (2.0f * q0q1 + _2q2q3 - ay) -
         4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
         _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (_2bx * q2 + _2bz * q0) *
             (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         (_2bx * q3 - _4bz * q1) *
             (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) +
         _2q3 * (2.0f * q0q1 + _2q2q3 - ay) -
         4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
         (-_4bx * q2 - _2bz * q0) *
             (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (_2bx * q1 + _2bz * q3) *
             (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         (_2bx * q0 - _4bz * q2) *
             (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) +
         _2q2 * (2.0f * q0q1 + _2q2q3 - ay) +
         (-_4bx * q3 + _2bz * q1) *
             (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (-_2bx * q0 + _2bz * q2) *
             (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

    // Normalized step
    float snorm = sqrtf(powf(s0, 2) + powf(s1, 2) + powf(s2, 2) + powf(s3, 2));
    s0 /= snorm;
    s1 /= snorm;
    s2 /= snorm;
    s3 /= snorm;

    // Apply feedback step
    qDot1 -= beta1 * s0;
    qDot2 -= beta2 * s1;
    qDot3 -= beta3 * s2;
    qDot4 -= beta4 * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * t_delta;
  q1 += qDot2 * t_delta;
  q2 += qDot3 * t_delta;
  q3 += qDot4 * t_delta;

  float qnorm = sqrtf(powf(q0, 2) + powf(q1, 2) + powf(q2, 2) + powf(q3, 2));
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
