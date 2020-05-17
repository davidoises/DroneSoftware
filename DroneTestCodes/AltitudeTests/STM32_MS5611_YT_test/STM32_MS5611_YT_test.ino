///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////
//Working
///////////////////////////////////////////////////////////////////////////////////////
//This program will plot the altitude on the Arduino plotter. It needs 5 seconds to 
//stabilize before the correct values are printed on the screen.
//Connect the MS5611 to the I2C port of the STM32 (B10 = clock & B11 = data)
///////////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
TwoWire HWire = TwoWire(0);          //Initiate I2C port 2 at 400kHz.

//Barometer variables.
//Pressure variables.
float pid_error_gain_altitude, pid_throttle_gain_altitude;
uint16_t C[7];
uint8_t barometer_counter, temperature_counter, average_temperature_mem_location, start;
int64_t OFF, OFF_C2, SENS, SENS_C1, P;
uint32_t raw_pressure, raw_temperature, temp, raw_temperature_rotating_memory[6], raw_average_temperature_total;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altutude_hold_pressure;
int32_t dT, dT_C5;
//Altitude PID variables
float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude, pid_last_altitude_d_error;

uint8_t parachute_rotating_mem_location;
int32_t parachute_buffer[35], parachute_throttle;
float pressure_parachute_previous;

int32_t pressure_rotating_mem[50], pressure_total_avarage;
uint8_t pressure_rotating_mem_location;
float pressure_rotating_mem_actual;
uint32_t loop_timer;
float top_line, bottom_line;

uint8_t MS5611_address = 0x77;             //The I2C address of the MS5611 barometer is 0x77 in hexadecimal form.

void setup() {
  Serial.begin(500000);                                          //Set the serial output to 57600 kbps.
  delay(100);
  //Check if the MS5611 is responding.
  //HWire.begin(9, 15, 400000);                                                  //Start the I2C as master
  HWire.begin();                                                  //Start the I2C as master
  HWire.beginTransmission(MS5611_address);                        //Start communication with the MS5611.
  if (HWire.endTransmission() != 0) {                                               //If the exit status is not 0 an error occurred.
    Serial.print("MS5611 is not responding on address: ");        //Print the error on the screen.
    Serial.println(MS5611_address, HEX);
    while (1);
  }
  else {                                                          //If the MS5611 is responding normal
    Serial.print("MS5611 found on address: ");                    //Print the conformation on the screen.
    Serial.println(MS5611_address, HEX);

    //For calculating the pressure the 6 calibration values need to be polled from the MS5611.
    //These 2 byte values are stored in the memory location 0xA2 and up.
    for (start = 1; start <= 6; start++) {
      HWire.beginTransmission(MS5611_address);                    //Start communication with the MPU-6050.
      HWire.write(0xA0 + start * 2);                              //Send the address that we want to read.
      HWire.endTransmission();                                    //End the transmission.

      HWire.requestFrom(MS5611_address, 2);                       //Request 2 bytes from the MS5611.
      C[start] = HWire.read() << 8 | HWire.read();                //Add the low and high byte to the C[x] calibration variable.
    }
    start = 0;
    //Print the 6 calibration values on the screen.
    Serial.print("C1 = ");
    Serial.println(C[1]);
    Serial.print("C2 = ");
    Serial.println(C[2]);
    Serial.print("C3 = ");
    Serial.println(C[3]);
    Serial.print("C4 = ");
    Serial.println(C[4]);
    Serial.print("C5 = ");
    Serial.println(C[5]);
    Serial.print("C6 = ");
    Serial.println(C[6]);

    OFF_C2 = C[2] * pow(2, 16);                                   //This value is pre-calculated to offload the main program loop.
    SENS_C1 = C[1] * pow(2, 15);                                  //This value is pre-calculated to offload the main program loop.
  }
}

void loop() {

  barometer_counter ++;

  if (barometer_counter == 1) {
    if (temperature_counter == 0) {
      //Get temperature data from MS-5611
      HWire.beginTransmission(MS5611_address);
      HWire.write(0x00);
      HWire.endTransmission();
      HWire.requestFrom(MS5611_address, 3);

      raw_average_temperature_total -= raw_temperature_rotating_memory[average_temperature_mem_location];
      raw_temperature_rotating_memory[average_temperature_mem_location] = HWire.read() << 16 | HWire.read() << 8 | HWire.read();
      raw_average_temperature_total += raw_temperature_rotating_memory[average_temperature_mem_location];
      average_temperature_mem_location++;
      if (average_temperature_mem_location == 5)average_temperature_mem_location = 0;
      raw_temperature = raw_average_temperature_total / 5;
    }
    else {
      //Get pressure data from MS-5611
      HWire.beginTransmission(MS5611_address);
      HWire.write(0x00);
      HWire.endTransmission();
      HWire.requestFrom(MS5611_address, 3);
      raw_pressure = HWire.read() << 16 | HWire.read() << 8 | HWire.read();
    }

    temperature_counter ++;
    if (temperature_counter == 20) {
      temperature_counter = 0;
      //Request temperature data
      HWire.beginTransmission(MS5611_address);
      HWire.write(0x58);
      HWire.endTransmission();
    }
    else {
      //Request pressure data
      HWire.beginTransmission(MS5611_address);
      HWire.write(0x48);
      HWire.endTransmission();
    }
  }
  if (barometer_counter == 2) {
    //Calculate pressure as explained in the datasheet of the MS-5611.
    dT = C[5];
    dT <<= 8;
    dT *= -1;
    dT += raw_temperature;
    OFF = OFF_C2 + ((int64_t)dT * (int64_t)C[4]) / pow(2, 7);
    SENS = SENS_C1 + ((int64_t)dT * (int64_t)C[3]) / pow(2, 8);
    P = ((raw_pressure * SENS) / pow(2, 21) - OFF) / pow(2, 15);

    //Let's use a rotating mem
    pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location];    //Subtract the current memory position to make room for the new value.
    pressure_rotating_mem[pressure_rotating_mem_location] = P;                          //Calculate the new change between the actual pressure and the previous measurement.
    pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location];    //Add the new value to the long term avarage value.
    pressure_rotating_mem_location++;                                                   //Increase the rotating memory location.
    if (pressure_rotating_mem_location == 20)pressure_rotating_mem_location = 0;        //Start at 0 when the memory location 20 is reached.
    actual_pressure_fast = (float)pressure_total_avarage / 20.0;                        //Calculate the avarage pressure value of the last 20 readings.
    if(millis() < 5000)actual_pressure_slow = actual_pressure_fast;                     //Keep the slow and fast avareges the same for the first 5 seconds.

    actual_pressure_slow = actual_pressure_slow * (float)0.985 + actual_pressure_fast * (float)0.015;
    
    actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;
    if (actual_pressure_diff > 8)actual_pressure_diff = 8;
    if (actual_pressure_diff < -8)actual_pressure_diff = -8;
    if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0;
    actual_pressure = actual_pressure_slow;
  }

  if (barometer_counter == 3) {                                                         //When the barometer counter is 3
    //Serial.print(top_line);
    //Serial.print(",");
    //Serial.print(bottom_line);
    //Serial.print(",");
    //Serial.print(actual_pressure);
    //Serial.print(",");
    double alt = 44330.0f * (1.0f - pow(actual_pressure / 101325.0f, 0.1902949f));
    Serial.println(alt*100.0);
    barometer_counter = 0;                                                              //Set the barometer counter to 0 for the next measurements.

  }

  if (millis() > 5000 && start == 0) {
    start = 1;
    top_line = actual_pressure + 20;
    bottom_line = actual_pressure - 20;
  }

  while (loop_timer > micros());
  loop_timer = micros() + 4000;                                                         //Set the loop_timer variable to the current micros() value + 4000.
}
