#ifndef BMI160_LIB
#define BMI160_LIB

#include "i2c_wrapper.h"
#include "math.h"

// IMU address
#define BMI160 0x68

// Register addresses
#define CHIPID_REG 0x00
#define CMD_REG 0x7E
#define ACC_CONF_REG 0x40
#define ACC_RANGE_REG 0x41
#define GYR_CONF_REG 0x42
#define GYR_RANGE_REG 0x43
#define PMU_REG 0x03
#define GYR_X_REG 0x0C
#define GYR_Y_REG 0x0E
#define GYR_Z_REG 0x10
#define ACC_X_REG 0x12
#define ACC_Y_REG 0x14
#define ACC_Z_REG 0x16
#define TIME_REG 0x18
#define FOC_CONF_REG 0x69
#define OFFSET_6_REG 0x77
#define OFF_GYR_X_REG 0x74
#define OFF_GYR_Y_REG 0x75
#define OFF_GYR_Z_REG 0x76

// IMU constants
#define RST_CMD 0xB6
#define RST_DELAY 15000
#define ACC_NORMAL_CMD 0x11
#define ACC_CMD_DELAY 4000
#define GYR_NORMAL_CMD 0x15
#define GYR_CMD_DELAY 86000
#define START_FOC_CMD 0x03
#define FOC_CMD_DELAY 25000
#define TIME_RES .000039


uint8_t chipid;
uint16_t acc_settings;
uint16_t gyr_settings;
uint8_t pmu_status;
float acc_res;
float gyr_res;

uint8_t imu_read(uint8_t reg)
{
    uint8_t rx_data[1];
    i2cget(BMI160, reg, rx_data, 1);
    return rx_data[0];
}

uint8_t imu_write(uint8_t reg, uint8_t data)
{
    uint8_t tx_data[1];
    tx_data[0] = reg;
    tx_data[1] = data;
    return i2cset(BMI160, tx_data, 2, true);
}

void bmi160_init()
{
    chipid = imu_read(CHIPID_REG);

    imu_write(CMD_REG, RST_CMD);
    //os_delay_us(RST_DELAY);
    delayMicroseconds(RST_DELAY);
    imu_write(GYR_RANGE_REG, 0x03);

    acc_settings = imu_read(ACC_RANGE_REG) << 8;
    acc_settings |= imu_read(ACC_CONF_REG);
    gyr_settings = imu_read(GYR_RANGE_REG) << 8;
    gyr_settings |= imu_read(GYR_CONF_REG);

    imu_write(CMD_REG, ACC_NORMAL_CMD);
    //os_delay_us(ACC_CMD_DELAY);
    delayMicroseconds(ACC_CMD_DELAY);
    imu_write(CMD_REG, GYR_NORMAL_CMD);
    //os_delay_us(GYR_CMD_DELAY);
    delayMicroseconds(GYR_CMD_DELAY);

    pmu_status = imu_read(PMU_REG);

    uint8_t acc_range = (acc_settings >> 8) & 0x0F;
    switch(acc_range)
    {
        case 3:
            acc_range = 2;
            break;
        case 5:
            acc_range = 4;
            break;
        case 8:
            acc_range = 8;
            break;
        case 12:
            acc_range = 16;
            break;
        default:
            acc_range = 0;
            break;
    }
    acc_res = ((float) acc_range)*9.81/pow(2,15);

    uint16_t gyr_range = (gyr_settings >> 8) & 0x07;
    switch(gyr_range)
    {
        case 0:
            gyr_range = 2000;
            break;
        case 1:
            gyr_range = 1000;
            break;
        case 2:
            gyr_range = 500;
            break;
        case 3:
            gyr_range = 250;
            break;
        case 4:
            gyr_range = 125;
            break;
        default:
            gyr_range = 0;
            break;
    }
    //gyr_res = gyr_range/pow(2, 15);
    gyr_res = ((float) gyr_range)/pow(2,15);

    Serial.print("Chip id: ");
    Serial.println(chipid);

    Serial.print("Accelerometer settings: ");
    Serial.println(acc_settings);

    Serial.print("Gyroscope settings: ");
    Serial.println(acc_settings);

    Serial.print("PMU status: ");
    Serial.println(pmu_status);

    Serial.print("Accelerometer range ");
    Serial.print(acc_range);
    Serial.print(" g with resolution of  ");
    Serial.print(acc_res);
    Serial.print(" m/s2/LSB\r\n");

    Serial.print("Gyroscope range ");
    Serial.print(gyr_range);
    Serial.print(" deg/s with resolution of  ");
    Serial.print(gyr_res);
    Serial.print(" deg/s/LSB\r\n");
}

void set_foc()
{
    imu_write(FOC_CONF_REG, 0x40);
    imu_write(CMD_REG, START_FOC_CMD);
    //os_delay_us(FOC_CMD_DELAY);
    delayMicroseconds(FOC_CMD_DELAY);
    uint8_t enable_foc = imu_read(OFFSET_6_REG) | (3<<6);
    imu_write(OFFSET_6_REG, enable_foc);
}

uint16_t twos_comp(uint16_t val, uint8_t bits)
{
    if((val & (1 << (bits - 1))) != 0)
        val = val - (1 << bits);
    return val;
}

uint16_t get_foc(char axis)
{
    if(axis == 'x')
    {
        uint16_t off_h = (imu_read(OFFSET_6_REG) & 0x03) << 8;
        uint16_t offset = off_h | imu_read(OFF_GYR_X_REG);
        return twos_comp(offset, 10);
    }
    if(axis == 'y')
    {
        uint16_t off_h = (imu_read(OFFSET_6_REG) & 0x0C) << 8;
        uint16_t offset = off_h | imu_read(OFF_GYR_Y_REG);
        return twos_comp(offset, 10);

    }
    if(axis == 'z')
    {
        uint16_t off_h = (imu_read(OFFSET_6_REG) & 0x30) << 8;
        uint16_t offset = off_h | imu_read(OFF_GYR_Z_REG);
        return twos_comp(offset, 10);
    }
}

float get_time()
{
    uint16_t data = imu_read(TIME_REG+2) << 16;
    data |= imu_read(TIME_REG+1) << 8;
    data |= imu_read(TIME_REG);

    return ((float) data) * TIME_RES;
}

void get_sensor_data(char sensor, int16_t* data)
{
    uint8_t X_REG, Y_REG, Z_REG;
    if(sensor == 'A')
    {
        X_REG = ACC_X_REG;
        Y_REG = ACC_Y_REG;
        Z_REG = ACC_Z_REG;
    }
    else
    {
        X_REG = GYR_X_REG;
        Y_REG = GYR_Y_REG;
        Z_REG = GYR_Z_REG;
    }

    data[0] = imu_read(X_REG+1) << 8;
    data[0] |= imu_read(X_REG);
    data[0] = twos_comp(data[0], 16);

    data[1] = imu_read(Y_REG+1) << 8;
    data[1] |= imu_read(Y_REG);
    data[1] = twos_comp(data[1], 16);

    data[2] = imu_read(Z_REG+1) << 8;
    data[2] |= imu_read(Z_REG);
    data[2] = twos_comp(data[2], 16);

    /*data[0] = ((int16_t)imu_read(X_REG+1)) << 8;
    data[0] |= imu_read(X_REG);

    data[1] = ((int16_t)imu_read(Y_REG+1)) << 8;
    data[1] |= imu_read(Y_REG);

    data[2] = ((int16_t)imu_read(Z_REG+1)) << 8;
    data[2] |= imu_read(Z_REG);*/
}

#endif
