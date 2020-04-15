#include "BMX055.h"
#include "SensorFusion.h"

// BMX055 IMU addresses
#define AM_DEV 0x18
#define G_DEV 0x68
#define MAG_DEV 0x10

// Class objects for data acquisition and sensor fusion
BMX055 imu = BMX055(AM_DEV, G_DEV, MAG_DEV, USE_MAG_CALIBRATION);
SensorFusion orientation;

// timer variables
volatile uint8_t update_orientation;
hw_timer_t * orientation_timer = NULL;

// Timer ISR
void IRAM_ATTR orientation_isr() {
  update_orientation = 1;
}

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

  orientation_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(orientation_timer, &orientation_isr, true);
  timerAlarmWrite(orientation_timer, 50000, true);
  
  orientation.set_yaw_offset(imu.magnetometer.yaw_offset);
  orientation.init(0, 0, MAG_TARGET);
  timerAlarmEnable(orientation_timer);
}

void loop() {
  if(update_orientation)
  {
    imu.get_gyr_data();
    imu.get_acc_data();
    imu.get_mag_data();
  
    orientation.fuse_sensors(imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z,
                            imu.gyroscope.x*imu.gyroscope.res, imu.gyroscope.y*imu.gyroscope.res, imu.gyroscope.z*imu.gyroscope.res,
                            imu.magnetometer.x, imu.magnetometer.y, imu.magnetometer.z);
  
    Serial.print("y");
    Serial.print(orientation.get_roll(), 10);//* 180.0/3.141592653589793, 10);
    Serial.print(",");
    Serial.print(orientation.get_pitch(), 10);//*180.0/3.141592653589793, 10);
    Serial.print(",");
    Serial.print(orientation.get_yaw(), 10);//* 180.0/3.141592653589793, 10);
    Serial.print("$\n");

    update_orientation = 0;
  }
}
