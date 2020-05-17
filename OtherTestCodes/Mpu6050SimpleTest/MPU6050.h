#ifndef MPU6050_LIB
#define MPU6050_LIB

#include <Arduino.h>
#include "Wire.h"

#define MPU6050_ADDRESS 0x68

#define ACC_RES 9.81f/4096.0f
#define GYR_RES 1.0f/65.5f

typedef struct
{
  int16_t acc_raw_x, acc_raw_y, acc_raw_z;
  int16_t gyr_raw_x, gyr_raw_y, gyr_raw_z;
  double acc_x, acc_y, acc_z;
  double gyr_x, gyr_y, gyr_z;
}SensorData;

class MPU6050
{
  public:
    SensorData sensed;
    uint8_t begin(TwoWire *aWire);
    uint8_t init();
    void get_acc_data();
    void get_gyr_data();
  private:
    TwoWire *selfaWire;
};

#endif
