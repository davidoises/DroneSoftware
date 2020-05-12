#include <Wire.h>
#include "Adafruit_BMP280.h"

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
float initial_pressure = 0;

void setup() {
  Serial.begin(500000);
  Serial.println(F("BMP280 test"));

  if (!bmp.begin(0x76, 0x58)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X4,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

  delay(1000);

  for(int i = 0; i < 5; i++)
  {
    initial_pressure += bmp.readPressure()/100.0;
    delay(50);
  }
  initial_pressure /= 5;
}

void loop() {

    Serial.println(bmp.readAltitude(initial_pressure)*100.0); /* Adjusted to local forecast! */

    delay(50);
}
