#include <Wire.h>
#include "bmp280.h"
//#include "bmp280_defs.h"

void delay_ms(uint32_t period_ms);
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void print_rslt(const char api_name[], int8_t rslt);

int8_t rslt;
struct bmp280_dev bmp;
struct bmp280_config conf;
struct bmp280_uncomp_data ucomp_data;
double pres;
double initial_altitude = 0;

void setup() {
  Serial.begin(500000);
  Serial.println("BMP280 test");

  Wire.begin(); // ESP32 default SDa=21, SCL=22
  Wire.setClock(400000);
  
  /* Map the delay function pointer with the function responsible for implementing the delay */
  bmp.delay_ms = delay_ms;

  /* Assign device I2C address based on the status of SDO pin (GND for PRIMARY(0x76) & VDD for SECONDARY(0x77)) */
  bmp.dev_id = BMP280_I2C_ADDR_PRIM;

  /* Select the interface mode as I2C */
  bmp.intf = BMP280_I2C_INTF;

  /* Map the I2C read & write function pointer with the functions responsible for I2C bus transfer */
  bmp.read = i2c_reg_read;
  bmp.write = i2c_reg_write;

  rslt = bmp280_init(&bmp);
  print_rslt(" bmp280_init status", rslt);

  /* Always read the current settings before writing, especially when
   * all the configuration is not modified
   */
  rslt = bmp280_get_config(&conf, &bmp);
  print_rslt(" bmp280_get_config status", rslt);

  /* configuring the temperature oversampling, filter coefficient and output data rate */
  /* Overwrite the desired settings */
  conf.filter = BMP280_FILTER_COEFF_16;

  /* Pressure oversampling set at 16x */
  conf.os_pres = BMP280_OS_4X;

  /* temperature oversampling set at 2x */
  conf.os_temp = BMP280_OS_1X;

  /* Setting the output data rate as 1HZ(1000ms) */
  conf.odr = BMP280_ODR_0_5_MS;
  rslt = bmp280_set_config(&conf, &bmp);
  print_rslt(" bmp280_set_config status", rslt);

  /* Always set the power mode after setting the configuration */
  rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
  print_rslt(" bmp280_set_power_mode status", rslt);

  

  delay(5000);
  
  bmp280_get_uncomp_data(&ucomp_data, &bmp);
  bmp280_get_comp_pres_double(&pres, ucomp_data.uncomp_press, &bmp);
  initial_altitude = 44330.0*(1.0-pow((pres/100.0)/1013.23f, 1.0/5.255));

  delay(50);
}

void loop() {

    /* Reading the raw data from sensor */
    rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);
        
    /* Getting the compensated pressure as floating point value */
    rslt = bmp280_get_comp_pres_double(&pres, ucomp_data.uncomp_press, &bmp);
    uint8_t elapsed_time = bmp280_compute_meas_time(&bmp);

    double meter = 44330.0*(1.0-pow((pres/100.0)/1013.23f, 1.0/5.255)) - initial_altitude;

    //Serial.print(elapsed_time);
    //Serial.print(" ");
    //Serial.print(pres);
    //Serial.print(" ");
    Serial.println(meter);

    bmp.delay_ms(50); /* Sleep time between measurements = BMP280_ODR_1000_MS */
}

/*!
 *  @brief Function that creates a mandatory delay required in some of the APIs such as "bmg250_soft_reset",
 *      "bmg250_set_foc", "bmg250_perform_self_test"  and so on.
 *
 *  @param[in] period_ms  : the required wait time in milliseconds.
 *  @return void.
 *
 */
void delay_ms(uint32_t period_ms)
{
    delay(period_ms);
}

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose value is to be written.
 *  @param[in] length   : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    Wire.beginTransmission(i2c_addr);
    Wire.write(reg_addr);
    for(int i = 0; i < length; i++) { Wire.write(reg_data[i]); }
    return Wire.endTransmission();
}

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : Sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
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

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : name of the API whose execution status has to be printed.
 *  @param[in] rslt     : error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void print_rslt(const char api_name[], int8_t rslt)
{
    if (rslt != BMP280_OK)
    {
        //printf("%s\t", api_name);
        Serial.print(api_name);
        Serial.print("\t");
        if (rslt == BMP280_E_NULL_PTR)
        {
            Serial.print("Error ");
            Serial.print(rslt);
            Serial.println(" : Null pointer error");
            //printf("Error [%d] : Null pointer error\r\n", rslt);
        }
        else if (rslt == BMP280_E_COMM_FAIL)
        {
            Serial.print("Error ");
            Serial.print(rslt);
            Serial.println(" : Bus communication failed");
            //printf("Error [%d] : Bus communication failed\r\n", rslt);
        }
        else if (rslt == BMP280_E_IMPLAUS_TEMP)
        {
            Serial.print("Error ");
            Serial.print(rslt);
            Serial.println(" : Invalid Temperature");
            //printf("Error [%d] : Invalid Temperature\r\n", rslt);
        }
        else if (rslt == BMP280_E_DEV_NOT_FOUND)
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
