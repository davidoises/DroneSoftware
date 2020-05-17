#include "MPU6050.h"

uint8_t MPU6050::begin(TwoWire *aWire)
{
  selfaWire = aWire;
  return init();
}

uint8_t MPU6050::init()
{
  selfaWire->beginTransmission(MPU6050_ADDRESS);                        //Start communication with the MPU-6050.
  uint8_t error = selfaWire->endTransmission();                              //End the transmission and register the exit status.
  if(error != 0)
    return error;
  
  selfaWire->beginTransmission(MPU6050_ADDRESS);                        //Start communication with the MPU-6050.
  selfaWire->write(0x6B);                                            //We want to write to the PWR_MGMT_1 register (6B hex).
  selfaWire->write(0x00);                                            //Set the register bits as 00000000 to activate the gyro.
  selfaWire->endTransmission();                                      //End the transmission with the gyro.

  selfaWire->beginTransmission(MPU6050_ADDRESS);                        //Start communication with the MPU-6050.
  selfaWire->write(0x1B);                                            //We want to write to the GYRO_CONFIG register (1B hex).
  selfaWire->write(0x08);                                            //Set the register bits as 00001000 (500dps full scale).
  selfaWire->endTransmission();                                      //End the transmission with the gyro.

  selfaWire->beginTransmission(MPU6050_ADDRESS);                        //Start communication with the MPU-6050.
  selfaWire->write(0x1C);                                            //We want to write to the ACCEL_CONFIG register (1A hex).
  selfaWire->write(0x10);                                            //Set the register bits as 00010000 (+/- 8g full scale range).
  selfaWire->endTransmission();                                      //End the transmission with the gyro.

  selfaWire->beginTransmission(MPU6050_ADDRESS);                        //Start communication with the MPU-6050.
  selfaWire->write(0x1A);                                            //We want to write to the CONFIG register (1A hex).
  selfaWire->write(0x03);                                            //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz).
  selfaWire->endTransmission();                                      //End the transmission with the gyro.
  
  return 0;
}

void MPU6050::get_acc_data()
{
  selfaWire->beginTransmission(MPU6050_ADDRESS);                       //Start communication with the gyro.
  selfaWire->write(0x3B);                                           //Start reading @ register 43h and auto increment with every read.
  selfaWire->endTransmission();                                     //End the transmission.
  selfaWire->requestFrom(MPU6050_ADDRESS, 6);                         //Request 14 bytes from the MPU 6050.
  sensed.acc_raw_x = selfaWire->read() << 8 | selfaWire->read();                   //Add the low and high byte to the acc_x variable.
  sensed.acc_raw_y = selfaWire->read() << 8 | selfaWire->read();                    //Add the low and high byte to the acc_y variable.
  sensed.acc_raw_z = selfaWire->read() << 8 | selfaWire->read(); // multiply by 9.81/4096                    //Add the low and high byte to the acc_z variable.

  sensed.acc_x = (double) sensed.acc_raw_x * ACC_RES;
  sensed.acc_y = (double) sensed.acc_raw_y * ACC_RES;
  sensed.acc_z = (double) sensed.acc_raw_z * ACC_RES;
}
void MPU6050::get_gyr_data()
{
  selfaWire->beginTransmission(MPU6050_ADDRESS);                       //Start communication with the gyro.
  selfaWire->write(0x43);                                           //Start reading @ register 43h and auto increment with every read.
  selfaWire->endTransmission();                                     //End the transmission.
  selfaWire->requestFrom(MPU6050_ADDRESS, 6);                         //Request 14 bytes from the MPU 6050.
  sensed.gyr_raw_x = selfaWire->read() << 8 | selfaWire->read();  //divide by 65.5 to get degrees              //Read high and low part of the angular data.
  sensed.gyr_raw_y = selfaWire->read() << 8 | selfaWire->read();               //Read high and low part of the angular data.
  sensed.gyr_raw_z = selfaWire->read() << 8 | selfaWire->read();                 //Read high and low part of the angular data.

  sensed.gyr_x = (double) sensed.gyr_raw_x * GYR_RES;
  sensed.gyr_y = (double) sensed.gyr_raw_y * GYR_RES;
  sensed.gyr_z = (double) sensed.gyr_raw_z * GYR_RES;
}
