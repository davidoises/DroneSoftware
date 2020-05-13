#include <Wire.h>
#include "bme280.h"

#define SEALEVELPRESSURE_HPA (1013.25)

void delay_ms(uint32_t period_ms);
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void print_rslt(const char api_name[], int8_t rslt);

int8_t rslt;
struct bme280_dev bme280;
struct bme280_settings conf;
struct bme280_data comp_data;
double initial_altitude = 0;

void setup() {
  Serial.begin(500000);
  Serial.println("BME280 test");

  Wire.begin(); // ESP32 default SDa=21, SCL=22
  Wire.setClock(400000);

  bme280.dev_id = BME280_I2C_ADDR_PRIM;//BME280_I2C_ADDR_SEC;
  bme280.intf = BME280_I2C_INTF;
  bme280.read = i2c_reg_read;
  bme280.write = i2c_reg_write;
  bme280.delay_ms = delay_ms;

  rslt = bme280_init(&bme280);
  print_rslt(" bme280_init status", rslt);

  /* Recommended mode of operation: gaming */
  bme280.settings.osr_h = BME280_NO_OVERSAMPLING;
  bme280.settings.osr_p = BME280_OVERSAMPLING_4X;
  bme280.settings.osr_t = BME280_OVERSAMPLING_1X;
  bme280.settings.filter = BME280_FILTER_COEFF_16;
  bme280.settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

  rslt = bme280_set_sensor_settings(BME280_ALL_SETTINGS_SEL, &bme280);
  print_rslt(" bme280 settings change", rslt);

  rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &bme280);
  print_rslt(" bme280 normal mode", rslt);

  Serial.println(bme280_cal_meas_delay(&bme280.settings));

  delay(2000);
  for(int i = 0; i < 20; i++)
  {
    rslt = bme280_get_sensor_data(BME280_PRESS, &comp_data, &bme280); // This parses and compensates
    initial_altitude += 44330.0*(1.0-pow((comp_data.pressure/100.0)/SEALEVELPRESSURE_HPA, 1.0/5.255));
    delay(13);
  }
  initial_altitude /= 20.0;
}

void loop() {
  
  rslt = bme280_get_sensor_data(BME280_PRESS, &comp_data, &bme280); // This parses and compensates
  double altitude = 44330.0*(1.0-pow((comp_data.pressure/100.0)/SEALEVELPRESSURE_HPA, 1.0/5.255));// - initial_altitude;
  //Serial.println(altitude);
  Serial.println(comp_data.pressure);
  delay(13);
}

void delay_ms(uint32_t period_ms)
{
    delay(period_ms);
}

int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    Wire.beginTransmission(i2c_addr);
    Wire.write(reg_addr);
    for(int i = 0; i < length; i++) { Wire.write(reg_data[i]); }
    return Wire.endTransmission();
}

int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
  
   Wire.beginTransmission(i2c_addr);
   Wire.write(reg_addr);
   Wire.endTransmission(false);

    Wire.requestFrom(i2c_addr, length);
    int i = 0;
    while(Wire.available()){
      if(length <= i)
        break;
      reg_data[i] = Wire.read();
      i++;
    }
    if(length == i)
      return 0;
    else
      return -1;
    /* Implement the I2C read routine according to the target machine. */
}

void print_rslt(const char api_name[], int8_t rslt)
{
    if (rslt != BME280_OK)
    {
        //printf("%s\t", api_name);
        Serial.print(api_name);
        Serial.print("\t");
        if (rslt == BME280_E_NULL_PTR)
        {
            Serial.print("Error ");
            Serial.print(rslt);
            Serial.println(" : Null pointer error");
            //printf("Error [%d] : Null pointer error\r\n", rslt);
        }
        else if (rslt == BME280_E_COMM_FAIL)
        {
            Serial.print("Error ");
            Serial.print(rslt);
            Serial.println(" : Bus communication failed");
            //printf("Error [%d] : Bus communication failed\r\n", rslt);
        }
        else if (rslt == BME280_E_DEV_NOT_FOUND)
        {
            Serial.print("Error ");
            Serial.print(rslt);
            Serial.println(" : Device not found");
            //printf("Error [%d] : Device not found\r\n", rslt);
        }
        else
        {
            Serial.print("Error ");
            Serial.print(rslt);
            Serial.println(" : Unknown error code");
            /* For more error codes refer "*_defs.h" */
        }
    }
}
