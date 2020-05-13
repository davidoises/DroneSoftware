#include <Wire.h>
#include "MS5611.h"
#include "bme280.h"

double prev_pressure = 0;

/***** MS5611 Variables and functions ***********/

MS5611 ms5611;

// rolling memory filter
double altitude_memory[20] = {0};
double altitude_sum = 0;
uint8_t altitude_index = 0;
double altitude_samples = 20.0; //20

//double referencePressure;
uint8_t conversion_time;
uint32_t rawTemp = 0;
double unfiltered_pressure = 0;
double filtered_pressure = 0;
double initial_altitude = 0;
uint8_t tp_counter = 0;

// Altitude timer variables
volatile uint8_t update_altitude;
hw_timer_t * altitude_timer = NULL;
void IRAM_ATTR altitude_isr() {
  tp_counter++;
  update_altitude = 1;
}

/***** BME280 Variables and functions ***********/

#define SEALEVELPRESSURE_HPA (1013.25)

void delay_ms(uint32_t period_ms);
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void print_rslt(const char api_name[], int8_t rslt);

int8_t rslt;
struct bme280_dev bme280;
struct bme280_settings conf;
struct bme280_data comp_data;
double bme280_initial_altitude = 0;

// Altitude timer variables
volatile uint8_t bme280_update_altitude;
hw_timer_t * bme280_altitude_timer = NULL;
void IRAM_ATTR bme280_altitude_isr() {
  bme280_update_altitude = 1;
}

/***** General Setup ***********/

void setup() 
{
  Serial.begin(500000);
  Wire.begin();
  Wire.setClock(400000);

  /***** MS5611 Setup start ***********/
  // Initialize MS5611 sensor
  Serial.println("");
  Serial.println("Initialize MS5611 Sensor");

  while(!ms5611.begin())
  {
    Serial.println("Could not find a valid MS5611 sensor, check wiring!");
    delay(500);
  }

  ms5611.setOversampling(MS5611_ULTRA_HIGH_RES);
  conversion_time = ms5611.getOversampling();
  Serial.print("Sampling time: ");
  Serial.println(conversion_time);

  // Inital altitude average calculation
  for(int i = 0; i < 20; i++)
  {
    initial_altitude += ms5611.getAltitude(ms5611.readPressure());
  }
  initial_altitude /= 20.0;

  for(int i = 0; i<altitude_samples; i++)
  {
    altitude_memory[i] = ms5611.readPressure();
    altitude_sum += altitude_memory[i];
  }

  altitude_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(altitude_timer, &altitude_isr, true);
  timerAlarmWrite(altitude_timer, conversion_time*1000.0, true);
  /***** MS5611 Setup end ***********/

  /***** BME280 Setup start ***********/
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
    bme280_initial_altitude += 44330.0*(1.0-pow((comp_data.pressure/100.0)/SEALEVELPRESSURE_HPA, 1.0/5.255));
    delay(13);
  }
  bme280_initial_altitude /= 20.0;

  bme280_altitude_timer = timerBegin(1, 80, true);
  timerAttachInterrupt(bme280_altitude_timer, &bme280_altitude_isr, true);
  timerAlarmWrite(bme280_altitude_timer, 13000.0, true);
  /***** BME280 Setup end ***********/

  Serial.print("MS5611 initial_altitude: ");
  Serial.println(initial_altitude);
  Serial.print("BME280 initial_altitude: ");
  Serial.println(bme280_initial_altitude);

  ms5611.requestTemperature();
  delay(conversion_time);
  rawTemp = ms5611.readRawTemperature();
  ms5611.requestPressure();
  delay(conversion_time);

  timerAlarmEnable(altitude_timer);
  timerAlarmEnable(bme280_altitude_timer);
}

void loop()
{
  if(update_altitude)
  {
    if(tp_counter == 20)
    {
      rawTemp = ms5611.readRawTemperature();
      ms5611.requestPressure();
      tp_counter = 0;
    }
    else if(tp_counter == 19)
    {
      unfiltered_pressure = ms5611.readPressure(rawTemp);
      ms5611.requestTemperature();
    }
    else
    {
      unfiltered_pressure = ms5611.readPressure(rawTemp);
      ms5611.requestPressure();
    }

    altitude_sum -= altitude_memory[altitude_index];
    altitude_memory[altitude_index] = unfiltered_pressure;
    altitude_sum += altitude_memory[altitude_index];
    altitude_index++;

    if(altitude_index == altitude_samples) altitude_index = 0;
    filtered_pressure = altitude_sum/altitude_samples;
    //Serial.println(pressure);
    //Serial.println(ms5611.getAltitude(pressure));
    
    update_altitude = 0;
  }

  if(bme280_update_altitude)
  {
    rslt = bme280_get_sensor_data(BME280_PRESS, &comp_data, &bme280); // This parses and compensates
    //double altitude = 44330.0*(1.0-pow((comp_data.pressure/100.0)/SEALEVELPRESSURE_HPA, 1.0/5.255));// - initial_altitude;
    double comp_filter = 0.8*filtered_pressure + 0.2*comp_data.pressure;
    //prev_pressure = 0.8*prev_pressure + 0.2*comp_filter;
    Serial.print(76855);
    Serial.print(" ");
    Serial.print(76955);
    Serial.print(" ");
    //Serial.print(unfiltered_pressure);
    //Serial.print(" ");
    Serial.print(filtered_pressure);
    Serial.print(" ");
    Serial.print(comp_data.pressure);
    Serial.print(" ");
    Serial.println(comp_filter);
    //Serial.print(" ");
    //Serial.println(prev_pressure);
    bme280_update_altitude = 0;
  }
}

/***** BME280 Helper functions ***********/
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
