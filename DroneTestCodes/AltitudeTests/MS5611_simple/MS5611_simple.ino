#include <Wire.h>
#include "MS5611.h"

MS5611 ms5611;

// rolling memory filter
double altitude_memory[20] = {0};
double altitude_sum = 0;
uint8_t altitude_index = 0;
double altitude_samples = 20.0; //20
double prev_pressure = 0;

double comp_pressure = 0;

uint8_t conversion_time;
uint32_t rawTemp = 0;
double pressure = 0;
double initial_altitude = 0;
uint8_t tp_counter = 0;

// Altitude timer variables
volatile uint8_t update_altitude;
hw_timer_t * altitude_timer = NULL;
void IRAM_ATTR altitude_isr() {
  tp_counter++;
  update_altitude = 1;
}

void setup() 
{
  Serial.begin(500000);
  //Wire.begin();
  //Wire.setClock(400000);
  Wire1.begin(9, 15, 400000); // SDA = 9, SCL = 15

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
  //Serial.print("Sampling time: ");
  //Serial.println(conversion_time);

  // Inital altitude average calculation
  for(int i = 0; i < 20; i++)
  {
    initial_altitude += ms5611.getAltitude(ms5611.readPressure());
  }
  initial_altitude /= 20.0;
  //Serial.print("initial_altitude: ");
  //Serial.println(initial_altitude);

  for(int i = 0; i<altitude_samples; i++)
  {
    altitude_memory[i] = ms5611.readPressure();
    altitude_sum += altitude_memory[i];
  }

  altitude_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(altitude_timer, &altitude_isr, true);
  timerAlarmWrite(altitude_timer, conversion_time*1000.0, true);

  ms5611.requestTemperature();
  delay(conversion_time);
  rawTemp = ms5611.readRawTemperature();
  ms5611.requestPressure();
  delay(conversion_time);

  timerAlarmEnable(altitude_timer);
}

void loop()
{
  if(update_altitude)
  {
    unsigned long prev_time = micros();
    if(tp_counter == 20)
    {
      rawTemp = ms5611.readRawTemperature();
      ms5611.requestPressure();
      tp_counter = 0;
    }
    else if(tp_counter == 19)
    {
      pressure = ms5611.readPressure(rawTemp);
      ms5611.requestTemperature();
    }
    else
    {
      pressure = ms5611.readPressure(rawTemp);
      ms5611.requestPressure();
    }

    comp_pressure = 0.97*comp_pressure + 0.03*pressure;

    altitude_sum -= altitude_memory[altitude_index];
    altitude_memory[altitude_index] = comp_pressure;
    altitude_sum += altitude_memory[altitude_index];
    altitude_index++;
    if(altitude_index == altitude_samples) altitude_index = 0;

    double filtered_pressure = altitude_sum/altitude_samples;
    if(abs(filtered_pressure-prev_pressure) < 0.4) filtered_pressure = prev_pressure;
    prev_pressure = filtered_pressure;

    double t_delta = (micros() - prev_time); // Just here to get measurement time
    
    Serial.println(pressure);
    //Serial.print(" ");
    //Serial.print(filtered_pressure);
    //Serial.print(" ");
    //Serial.println(comp_pressure);
   // Serial.println(ms5611.getAltitude(filtered_pressure)*100.0);
    
    update_altitude = 0;
  }
}
