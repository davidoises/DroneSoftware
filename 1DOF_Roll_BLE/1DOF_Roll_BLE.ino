#include "BMX055.h"
#include "SensorFusion.h"
#include "ble_conf.h"

// BMX055 IMU addresses
#define AM_DEV 0x18
#define G_DEV 0x68
#define MAG_DEV 0x10

// Motor Pins
#define PWMA 32
#define PWMB 33
#define PWMC 14
#define PWMD 10

// setting PWM properties
#define FREQ 50
#define RESOLUTION 12
#define ledChannelA 0
#define ledChannelC 1

// Constants for ESC control
#define MAX_PWM (1<<RESOLUTION)-1
#define MAX_PERIOD 1000.0f/((float)FREQ)
#define MS_TO_PWM(x) ((float)x)*((float)MAX_PWM)/(((float)MAX_PERIOD)*1000.0f)

// Class objects for data acquisition and sensor fusion
BMX055 imu = BMX055(AM_DEV, G_DEV, MAG_DEV, USE_MAG_CALIBRATION);
SensorFusion orientation;

// variables fisicas dron
double dcm = 0.19757; // meters
double max_thrust = .4; // kg
double max_torque = dcm*max_thrust; //kg*m
double base_throttle = 1250.0;

// Control stuff
double setpoint = 0;
double prev_error = 0;
double integral = 0;
double kp = 3.4; // 3.7 // 0.11
double kd = 1.1; // .015
double ki = 0.1; // 0.02 // 0.015
unsigned long prev_time = 0;

// timer variables
volatile uint8_t update_orientation;
volatile uint8_t update_orientation_pid;
hw_timer_t * orientation_timer = NULL;

// Timer ISR
void IRAM_ATTR orientation_isr() {
  update_orientation = 1;
  update_orientation_pid = 1;
}

void setup() {

  ledcSetup(ledChannelA, FREQ, RESOLUTION);
  ledcAttachPin(PWMA, ledChannelA);
  ledcWrite(ledChannelA, MS_TO_PWM(1000));

  ledcSetup(ledChannelC, FREQ, RESOLUTION);
  ledcAttachPin(PWMC, ledChannelC);
  ledcWrite(ledChannelC, MS_TO_PWM(1000));
  
  Serial.begin(115200);
  ble_setup(kp, ki, kd);

  delay(100);
  Serial.println("\r\n");
  delay(100);
  
  Wire.begin(); // ESP32 default SDa=21, SCL=22
  Wire.setClock(400000);

  Serial.println("Initializing sensor in 3 seconds");
  delay(3000);
  
  imu.acc_init();
  imu.gyr_init();
  imu.mag_init();

  orientation_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(orientation_timer, &orientation_isr, true);
  timerAlarmWrite(orientation_timer, 50000, true);
  
  orientation.set_yaw_offset(imu.magnetometer.yaw_offset);
  orientation.init(0, 0, MAG_TARGET);
  timerAlarmEnable(orientation_timer);
  prev_time = millis();
}

void loop() {
  // Get orientation stuff
  if(update_orientation)
  {
    imu.get_gyr_data();
    imu.get_acc_data();
    imu.get_mag_data();
  
    orientation.fuse_sensors(imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z,
                            imu.gyroscope.x*imu.gyroscope.res, imu.gyroscope.y*imu.gyroscope.res, imu.gyroscope.z*imu.gyroscope.res,
                            imu.magnetometer.x, imu.magnetometer.y, imu.magnetometer.z);

    update_orientation = 0;
  }

  // update constants and control stuff
  if(ble_triggered)
  {
    kp = ble_kp;
    ki = ble_ki;
    kd = ble_kd;
    ble_triggered = 0;
    prev_error = 0;
    integral = 0;
    //prev_integral = 0;
  }

  if(update_orientation_pid)
  {
    // Calculate the elapsed time
    unsigned long current_time = millis();
    double t_delta = ((double)current_time - (double)prev_time)/1000;
    prev_time = current_time;
  
    // Error, error derivative and error integral calculation
    double roll = orientation.get_roll()*180.0/PI;
    double error = setpoint - roll;
    double diff = (error - prev_error)/t_delta;
    integral += (t_delta/2.0)*(error + prev_error);
    prev_error = error;

    // PID calculation
    double pid_p = kp*error;
    double pid_d = kd*diff;
    double pid_i = ki*integral;
    double torque_control_law = pid_p + pid_d + pid_i;
    
    // Just use integral part when the error is small. This is just to reduce stationary error
    /*if(abs(error) > 0.05)
    {
      torque_control_law = pid_p + pid_d;
    }
    */
    
    /** internet's method **/
    torque_control_law = constrain(torque_control_law, -500, 500);
    double m1 = base_throttle + torque_control_law/2.0;
    double m2 = base_throttle - torque_control_law/2.0;

    ledcWrite(ledChannelA, MS_TO_PWM(m1));
    ledcWrite(ledChannelC, MS_TO_PWM(m2));
    //Serial.print(MS_TO_PWM(m1));
    //Serial.print(" ");
    //Serial.println(MS_TO_PWM(m2));
    Serial.print(error);
    Serial.print(" ");
    Serial.println(torque_control_law);
    
    update_orientation_pid = 0;
  }
}
