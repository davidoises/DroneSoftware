#include "BMX055.h"
#include "SensorFusion.h"

// UI Control variables
double eStop = 1;
double throttle = 1000.0;
double kp = 0;
double kd = 0;
double ki = 0;

#include "ui_conf.h"

// BMX055 IMU addresses
#define AM_DEV 0x18
#define G_DEV 0x68
#define MAG_DEV 0x10

// HW Pins
#define PWMA 32
#define PWMB 33
#define PWMC 14
#define PWMD 10
#define SERVO1_X 13
#define SERVO1_Y 12
#define SERVO2_X 26
#define SERVO2_Y 25
#define ADCA 37
#define ADCB 38
#define ADCC 34
#define ADCD 35

// setting PWM properties
#define FREQ 250
#define RESOLUTION 12
#define ledChannelA 0
#define ledChannelB 1
#define ledChannelC 2
#define ledChannelD 3

// Constants for ESC control
#define MAX_PWM (1<<RESOLUTION)-1
#define MAX_PERIOD 1000.0f/((double)FREQ)
#define MS_TO_PWM(x) ((double)x)*((double)MAX_PWM)/(((double)MAX_PERIOD)*1000.0f)

#define ledChannel ledChannelB

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

// Measurement variables
double roll_rate = 0;
double pitch_rate = 0;
double yaw_rate = 0;

void setup() {
  Serial.begin(500000);
  //Serial.begin(1000000);
  
  delay(100);
  Serial.println("\r\n");
  delay(100);
  
  Wire.begin(); // ESP32 default SDa=21, SCL=22
  Wire.setClock(400000);

  ledcSetup(ledChannelA, FREQ, RESOLUTION);
  ledcAttachPin(PWMA, ledChannelA); // negative torque
  ledcWrite(ledChannelA, MS_TO_PWM(1000));

  ledcSetup(ledChannelB, FREQ, RESOLUTION);
  ledcAttachPin(PWMB, ledChannelB); // negative torque
  ledcWrite(ledChannelB, MS_TO_PWM(1000));

  ledcSetup(ledChannelC, FREQ, RESOLUTION);
  ledcAttachPin(PWMC, ledChannelC); // Positive toruqe
  ledcWrite(ledChannelC, MS_TO_PWM(1000));

  ledcSetup(ledChannelD, FREQ, RESOLUTION);
  ledcAttachPin(PWMD, ledChannelD); // Positive toruqe
  ledcWrite(ledChannelD, MS_TO_PWM(1000));
  
  imu.acc_init();
  imu.gyr_init();
  //imu.mag_init();
  delay(1000);

  #if WIFI_GUI
  Blynk.begin(auth, ssid, pass);
  #else
  Blynk.begin(auth);
  #endif

  orientation_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(orientation_timer, &orientation_isr, true);
  timerAlarmWrite(orientation_timer, ORIENTATION_SAMPLING*1000000.0, true);

  delay(4000);
  
  orientation.set_yaw_offset(imu.magnetometer.yaw_offset);
  orientation.init(0, 0, MAG_TARGET);
  timerAlarmEnable(orientation_timer);
}

void loop() {
  
  Blynk.run();

  if(eStop)
  {
    ledcWrite(ledChannel, MS_TO_PWM(1000));
  }
  
  if(update_orientation)
  {
    
    imu.get_gyr_data();
    imu.get_acc_data();
    //imu.get_mag_data();

    
    orientation.fuse_sensors(imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z,
                            imu.gyroscope.x*imu.gyroscope.res, imu.gyroscope.y*imu.gyroscope.res, imu.gyroscope.z*imu.gyroscope.res,
                            imu.magnetometer.x, imu.magnetometer.y, imu.magnetometer.z);

    double ax = imu.accelerometer.x*imu.accelerometer.res;
    double ay = imu.accelerometer.y*imu.accelerometer.res;
    double az = imu.accelerometer.z*imu.accelerometer.res;
    double total_acc_vec = sqrt(pow(ax, 2) + pow(ay, 2) + pow(az, 2));

    roll_rate = roll_rate*0.7 + imu.gyroscope.x*imu.gyroscope.res*0.3;
    pitch_rate = pitch_rate*0.7 + imu.gyroscope.y*imu.gyroscope.res*0.3;
    yaw_rate = yaw_rate*0.7 + imu.gyroscope.z*imu.gyroscope.res*0.3;

    if(!eStop)
    {
      ledcWrite(ledChannel, MS_TO_PWM(throttle));
    }
    
    Serial.print(total_acc_vec);
    Serial.print(" ");
    Serial.print(roll_rate);
    Serial.print(" ");
    Serial.print(pitch_rate);
    Serial.print(" ");
    Serial.println(yaw_rate);
    
    update_orientation = 0;
  }
}
