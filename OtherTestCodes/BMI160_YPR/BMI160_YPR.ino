#include "BMI160.h"
#include "Math3D.h"
#include <math.h>

float t;
//double attitude_quat[4] = {1, 0, 0, 0};
double attitude_ypr[3] = {0};

void setup() {
  //pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  Serial.begin(115200);
  //Serial.begin(1000000);
  Serial.println("\r\n");
  delay(100);
  //Wire.begin(SDA,SCL)
  //Wire.begin(D4, D5); // ESP8266
  Wire.begin(D1, D2); // ESP8266
  //Wire.begin(33,32); // ESP32
  //Wire.begin(); // ESP32 default SDa=21, SCL=22
  Wire.setClock(400000);

  bmi160_init();
}

void loop() {

  float t_minus = t;
  t = get_time();
  float t_delta = t - t_minus;
  if(t_delta < 0)
  {
      if(t_delta > -20)
          return;
      else
          t_delta += pow(2, 49)*.000039;
  }

  int16_t gyr[3];
  double gyr_data[3];
  get_sensor_data('G', gyr);
  gyr_data[0] = ((float) gyr[0])*gyr_res*t_delta*(M_PI/180.0);
  gyr_data[1] = ((float) gyr[1])*gyr_res*t_delta*(M_PI/180.0);
  gyr_data[2] = ((float) gyr[2])*gyr_res*t_delta*(M_PI/180.0);

  int16_t acc[3];
  double acc_data[3]; // Only to elements used, X and Y angle obtained from accelerations
  get_sensor_data('A', acc);
  acc_data[0] = (atan(( (float) acc[1])/sqrt(pow(( (float) acc[0]), 2) + pow(( (float) acc[2]), 2))));// * 180.0/3.141592653589793;
  acc_data[1] = (atan(-1.0*( (float) acc[0])/sqrt(pow(( (float) acc[1]), 2) + pow(( (float) acc[2]), 2))));// *180.0/3.141592653589793; // Este esta invertido

  attitude_ypr[0] = 0.98*(attitude_ypr[0] + gyr_data[0]) + 0.02*acc_data[0];
  //attitude_ypr[0] = 0.98*(attitude_ypr[0] + gyr_data[0]) + 0.02*(attitude_ypr[0] + gyr_data[0]);
  //attitude_ypr[0] = acc_data[0];
  //attitude_ypr[0] += gyr_data[0];
  
  attitude_ypr[1] = 0.98*(attitude_ypr[1] + gyr_data[1]) + 0.02*acc_data[1];
  //attitude_ypr[1] = 0.98*(attitude_ypr[1] + gyr_data[1]) + 0.02*(attitude_ypr[1] + gyr_data[1]);
  //attitude_ypr[1] = acc_data[1];
  //attitude_ypr[1] += gyr_data[1];
  
  attitude_ypr[2] += gyr_data[2];
  
  Serial.print("y");
  Serial.print(attitude_ypr[0], 10);
  Serial.print(",");
  Serial.print(attitude_ypr[1], 10);
  Serial.print(",");
  Serial.print(attitude_ypr[2], 10);
  Serial.print("$\n");
}
