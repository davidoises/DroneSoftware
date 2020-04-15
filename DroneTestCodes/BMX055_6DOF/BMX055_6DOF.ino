#include "BMX055.h"
#include "SensorFusion.h"

// BMX055 IMU addresses
#define AM_DEV 0x18
#define G_DEV 0x68
#define MAG_DEV 0x10

// Class objects for data acquisition and sensor fusion
BMX055 imu = BMX055(AM_DEV, G_DEV, MAG_DEV, USE_MAG_CALIBRATION);
SensorFusion orientation;

// timer variables
volatile uint8_t update_orientation;
volatile uint8_t update_position;
hw_timer_t * orientation_timer = NULL;
hw_timer_t * position_timer = NULL;
//portMUX_TYPE orientation_mux = portMUX_INITIALIZER_UNLOCKED;
//portMUX_TYPE position_mux = portMUX_INITIALIZER_UNLOCKED;

// Orientation ISR
void IRAM_ATTR orientation_isr() {
  //portENTER_CRITICAL_ISR(&orientation_mux);
  update_orientation = 1;
  //portEXIT_CRITICAL_ISR(&orientation_mux);
}

// Position ISR
void IRAM_ATTR position_isr() {
  //portENTER_CRITICAL_ISR(&position_mux);
  update_position = 1;
  //portEXIT_CRITICAL_ISR(&position_mux);
}

double roll = 0, pitch = 0, yaw = 0;
unsigned long prev_time;
double velx = 0;
double posx = 0;
double prev_accx = 0;
double prev_velx = 0;
uint8_t accx_count = 0;

void setup() {
  Serial.begin(115200);
  //Serial.begin(1000000);
  
  delay(100);
  Serial.println("\r\n");
  delay(100);
  
  //Wire.begin(SDA,SCL)
  //Wire.begin(D4, D5); // ESP8266
  //Wire.begin(D1, D2); // ESP8266
  //Wire.begin(33,32); // ESP32
  Wire.begin(); // ESP32 default SDa=21, SCL=22
  Wire.setClock(400000);
  
  imu.acc_init();
  imu.gyr_init();
  imu.mag_init();
  //delay(1000);

  // orientation timer set up
  orientation_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(orientation_timer, &orientation_isr, true);
  timerAlarmWrite(orientation_timer, 16000, true);

  // position timer set up
  position_timer = timerBegin(2, 80, true);
  timerAttachInterrupt(position_timer, &position_isr, true);
  timerAlarmWrite(position_timer, 50000, true);

  orientation.set_yaw_offset(imu.magnetometer.yaw_offset);
  orientation.init(0, 0, MAG_TARGET);

  for(int i = 0; i < 50; i++)
  {
    // Get orientation
    imu.get_gyr_data();
    imu.get_acc_data();
    imu.get_mag_data();
    orientation.fuse_sensors(imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z,
                            imu.gyroscope.x*imu.gyroscope.res, imu.gyroscope.y*imu.gyroscope.res, imu.gyroscope.z*imu.gyroscope.res,
                            imu.magnetometer.x, imu.magnetometer.y, imu.magnetometer.z);
    delay(16);
  }
  timerAlarmEnable(orientation_timer);
  timerAlarmEnable(position_timer);
  prev_time = 0;
}

void loop() {
  if(update_orientation)
  {
    // Get orientation
    imu.get_gyr_data();
    imu.get_acc_data();
    imu.get_mag_data();
    orientation.fuse_sensors(imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z,
                            imu.gyroscope.x*imu.gyroscope.res, imu.gyroscope.y*imu.gyroscope.res, imu.gyroscope.z*imu.gyroscope.res,
                            imu.magnetometer.x, imu.magnetometer.y, imu.magnetometer.z);
    roll = orientation.get_roll();
    pitch = orientation.get_pitch();
    yaw = orientation.get_yaw();
    
    update_orientation = 0;
  }

  if(update_position)
  {
    // Get time delta between updates
    unsigned long current_time = millis();
    double t_delta = ((double)current_time - (double)prev_time)/1000;
    prev_time = current_time;
  
    // Acceleration in the body frame
    double ax = imu.accelerometer.x*imu.accelerometer.res;
    double ay = imu.accelerometer.y*imu.accelerometer.res;
    double az = imu.accelerometer.z*imu.accelerometer.res;
  
    // Acceleration in the world frame
    double wax = az*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) - ay*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + ax*cos(pitch)*cos(yaw);
    double way = ay*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - az*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + ax*cos(pitch)*sin(yaw);
    double waz = az*cos(pitch)*cos(roll) - ax*sin(pitch) + ay*cos(pitch)*sin(roll) -9.81;
  
    //wax = ax;
    if(abs(wax) <= 0.15)
    {
      wax = 0;
    }
    
    if(wax == 0)
    {
      accx_count++;  
    }
    else
    {
      accx_count = 0;
    }
    velx += t_delta*(prev_accx + wax)/2.0;
    if(accx_count >= 10)
    {
      velx = 0;
      accx_count = 0;
    }
  
    posx += t_delta*(prev_velx + velx)/2.0;
    prev_accx = wax;
    prev_velx = velx;
    
  
    /*
    // To complex, but also works
    // Get the gravity in the body frame
    double gravity_x = -9.81*sin(pitch);
    double gravity_y = 9.81*cos(pitch)*sin(roll);
    double gravity_z = 9.81*cos(pitch)*cos(roll);
    
    // substract gravity from accelerations measured in the body frame
    double body_ax = ax - gravity_x;
    double body_ay = ay - gravity_y;
    double body_az = az - gravity_z;
    
    double wax2 = body_az*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) - body_ay*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + body_ax*cos(pitch)*cos(yaw);
    double way2 = body_ay*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - body_az*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) + body_ax*cos(pitch)*sin(yaw);
    double waz2 = body_az*cos(pitch)*cos(roll) - body_ax*sin(pitch) + body_ay*cos(pitch)*sin(roll);
    */
    
    
    Serial.print(wax);
    Serial.print(" ");
    Serial.print(velx*100);
    Serial.print(" ");
    Serial.println(posx*100);
    
    update_position = 0;
  }
}
