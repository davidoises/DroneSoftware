#include "BMI160.h"
#include "Math3D.h"
#include <ESP8266WiFi.h>

// Global variables
float t;
double attitude_quat[4] = {1, 0, 0, 0};
int port = 8888;  //Port number
WiFiServer server(port);

// Function prototypes
void process_imu_data();

void setup() {
  //pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  //Serial.begin(115200);
  Serial.begin(500000);
  //Serial.begin(1000000);
  Serial.println("\r\n");
  delay(100);
  
  Wire.begin(D4, D5);
  Wire.setClock(400000);
  bmi160_init();

  WiFi.begin("Cablevision127", "Contra_Word.");
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  delay(500);
  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop() {
  //process_imu_data(); 
  WiFiClient client = server.available();
  
  if (client) {
    if(client.connected())
    {
      Serial.println("Client Connected");
    }
    
    while(client.connected()){      
      while(client.available()>0){
        // read data from the connected client
        Serial.write(client.read()); 
      }
      //Send Data to connected client
      while(Serial.available()>0)
      {
        client.write(Serial.read());
      }
    }
    client.stop();
    Serial.println("Client disconnected");    
  }
}

void process_imu_data()
{
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
  gyr_data[0] = ((float) gyr[0])*gyr_res*(M_PI/180.0);
  gyr_data[1] = ((float) gyr[1])*gyr_res*(M_PI/180.0);
  gyr_data[2] = ((float) gyr[2])*gyr_res*(M_PI/180.0);

  double gyro_quat[4];
  double norm = vec_normalize(gyr_data, gyr_data);
  quat_generate(gyro_quat, norm*t_delta, gyr_data);

  quat_multiply(attitude_quat, attitude_quat, gyro_quat);

  int16_t tmp[3];
  double acc_data[4];
  get_sensor_data('A', tmp);
  acc_data[0] = 0;
  acc_data[1] = (double) tmp[0];
  acc_data[2] = (double) tmp[1];
  acc_data[3] = (double) tmp[2];

  double temp_quat[4];
  quat_multiply(temp_quat ,attitude_quat, acc_data);

  double conj_attitude[4];
  quat_conjugate(conj_attitude, attitude_quat);

  double world_quat[4];
  quat_multiply(world_quat, temp_quat, conj_attitude);
  double world_vec[3] = {world_quat[1], world_quat[2], world_quat[3]};

  double correction_axis[3] = {world_vec[1], -world_vec[0], 0};
  double correction_angle = atan2(sqrt(pow(world_vec[0], 2) + pow(world_vec[1], 2)), world_vec[2]) * .1;
  double correction_quat[4];
  vec_normalize(correction_axis, correction_axis);
  quat_generate(correction_quat, correction_angle, correction_axis);

  quat_multiply(attitude_quat , correction_quat, attitude_quat);
  quat_normalize(attitude_quat, attitude_quat);

  Serial.print("y");
  Serial.print(attitude_quat[0], 10);
  Serial.print(",");
  Serial.print(attitude_quat[1], 10);
  Serial.print(",");
  Serial.print(attitude_quat[2], 10);
  Serial.print(",");
  Serial.print(attitude_quat[3], 10);
  Serial.print("$\n");
}
