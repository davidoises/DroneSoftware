#include "RollingMemory.h"
#include "BMX055.h"
#include "SensorFusion.h"

// BMX055 IMU addresses
#define AM_DEV 0x18
#define G_DEV 0x68
#define MAG_DEV 0x10

// Attitude sampling
#define ORIENTATION_SAMPLING 0.004f

// Class objects for data acquisition and sensor fusion
BMX055 imu = BMX055(AM_DEV, G_DEV, MAG_DEV, USE_MAG_CALIBRATION);
SensorFusion orientation;

// timer variables
volatile uint8_t update_orientation;
hw_timer_t * orientation_timer = NULL;

// Timer ISR
void IRAM_ATTR orientation_isr() {
  update_orientation = 1;
}

// Angular measurement variables
unsigned long orientation_prev_time = 0;
double roll_rate = 0;
double pitch_rate = 0;
double yaw_rate = 0;
double roll = 0;
double pitch = 0;
double yaw = 0;

double g_roll = 0;
double g_pitch = 0;
double simple_roll = 0;

//RollingMemory acc

double initial_acc = 0;

void setup() {

  // UART Initialization
  //Serial.begin(115200);
  Serial.begin(500000);
  //Serial.begin(1000000);
  delay(100);
  Serial.println("\r\n");
  delay(100);

  // I2C initialization
  Wire.begin(); // ESP32 default SDa=21, SCL=22
  Wire.setClock(400000);

  // IMU sensors initialization
  imu.acc_init();
  imu.gyr_init();
  //imu.mag_init();
  for(int i = 0; i < 200; i++)
  {
    imu.get_acc_data();
    delay(5);
  }
  for(int i = 0; i < 100; i++)
  {
    imu.get_acc_data();
    initial_acc += imu.accelerometer.z*imu.accelerometer.res;
    delay(4);
  }
  initial_acc /= 100.0;

  // Orientation timer
  orientation_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(orientation_timer, &orientation_isr, true);
  timerAlarmWrite(orientation_timer, ORIENTATION_SAMPLING*1000000.0, true);
  //orientation.set_yaw_offset(imu.magnetometer.yaw_offset);
  orientation.init(0, 0, MAG_TARGET);
  
  // Start timer
  timerAlarmEnable(orientation_timer);
  
  orientation_prev_time = millis();
}

void loop() {
  
  if(update_orientation)
  {
    // Loop time for pid and time integration
    unsigned long current_time = millis();
    double dt = (current_time - orientation_prev_time)/1000.0;
    orientation_prev_time = millis();

    // Imu data collection and attitude sensor fusion
    imu.get_gyr_data();
    imu.get_acc_data();
    //imu.get_mag_data();
    /*orientation.fuse_sensors(imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z,
                            imu.gyroscope.x*imu.gyroscope.res, imu.gyroscope.y*imu.gyroscope.res, imu.gyroscope.z*imu.gyroscope.res,
                            imu.magnetometer.x, imu.magnetometer.y, imu.magnetometer.z);
    */
    double gx = imu.gyroscope.x*imu.gyroscope.res;
    double gy = imu.gyroscope.y*imu.gyroscope.res;
    double gyr_roll = gx*dt*(PI/180.0);
    double gyr_pitch = gy*dt*(PI/180.0);

    g_roll += gyr_roll;
    g_pitch += gyr_pitch;
    
    double ax = imu.accelerometer.x;
    double ay = imu.accelerometer.y;
    double az = imu.accelerometer.z;
    double acc_roll = atan2(ay, sqrt(pow(ax, 2) + pow(az, 2)));
    double acc_pitch = atan2(-1.0*ax, sqrt(pow(ay, 2) + pow(az, 2)));

    double acc_total_vector = sqrt((ax*ax) + (ay*ay) + (az*az));    //Calculate the total accelerometer vector.
    double angle_pitch_acc = asin((float)ay / acc_total_vector);              //Calculate the pitch angle.
    double angle_roll_acc = asin((float)ax / acc_total_vector);               //Calculate the roll angle


    // Angular position measurement
    
    simple_roll = 0.99*(simple_roll+gyr_roll) + 0.01*acc_roll;
    //pitch = 0.99*(pitch+gyr_pitch) + 0.01*acc_pitch;
    double k1 = 5.0/10.0; //gyr noise/acc noise
    double dr = acc_roll - roll;
    roll = roll + gyr_roll + k1*dt*dr;
    
    // Angular rate measurements
    roll_rate = roll_rate*0.7 + imu.gyroscope.x*imu.gyroscope.res*0.3;
    pitch_rate = pitch_rate*0.7 + imu.gyroscope.y*imu.gyroscope.res*0.3;
    yaw_rate = yaw_rate*0.7 + imu.gyroscope.z*imu.gyroscope.res*0.3;

    Serial.print(acc_roll);
    Serial.print(" ");
    Serial.println(angle_pitch_acc);
    //Serial.print(g_roll);
    //Serial.print(" ");
    //Serial.print(roll);
    //Serial.print(" ");
    //Serial.print(simple_roll);
    //Serial.print(" ");
    //Serial.println(g_roll*0.9996 + acc_roll*0.0004);
    
    //Serial.println(roll*180/PI);
    //Serial.print(" ");
    //Serial.println(pitch*180/PI);
    //Serial.print(roll_rate);
    //Serial.print(" ");
    //Serial.println(pitch_rate);

    
    update_orientation = 0;
  }
}
