#include "BMX055.h"
#include "MPU6050.h"
#include "SensorFusion.h"
#include "MS5611.h"

// UI Control variables
double eStop = 1;
double throttle = 1000.0;
double kr = 8; //11

double kp_roll = 3.7;
double ki_roll = 3.1;
double kd_roll = 0.5;

double kp_pitch = kp_roll;
double ki_pitch = ki_roll*1.1;
double kd_pitch = kd_roll*1.1;

double kp_yaw = 1;
double ki_yaw = 0;
double kd_yaw = 0.2;

double kp_alt = 0;//10
double ki_alt = 0;
double kd_alt = 0;//10

double roll_setpoint = 0;
double pitch_setpoint = 0;
double yaw_setpoint = 0;

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

// Attitude sampling
#define ORIENTATION_SAMPLING 0.004f
#define ALTITUDE_SAMPLING 0.009f

// Rolling average strcture
typedef struct {
  double memory[30] = {0};
  double sum = 0;
  uint8_t index = 0;
  double samples = 20.0;
  uint8_t averaged_samples = 0;
}RollingMemory;

// Class objects for data acquisition and sensor fusion
MS5611 ms5611;
MPU6050 mpu;
BMX055 imu = BMX055(AM_DEV, G_DEV, MAG_DEV, USE_MAG_CALIBRATION);
SensorFusion orientation;

// timer variables
volatile uint8_t update_orientation;
volatile uint8_t tp_counter = 0;
volatile uint8_t alt_counter = 0;
volatile uint8_t update_altitude;
hw_timer_t * orientation_timer = NULL;

// Timer ISR
void IRAM_ATTR orientation_isr() {
  update_orientation = 1;
  // Update altitude every 3 orientation samples (ORIENTATION SAMPLING *3)
  alt_counter++;
  if(alt_counter == 3)
  {
    update_altitude = 1;
    alt_counter = 0;
  }
}

// Angular measurement variables
unsigned long orientation_prev_time = 0;
double roll_rate = 0;
double pitch_rate = 0;
double yaw_rate = 0;
double roll = 0;
double pitch = 0;
double yaw = 0;

// Altitude measurement variables
unsigned long altitude_prev_time = 0;
uint32_t raw_temp = 0;
double unfiltered_pressure = 0;
double filtered_pressure = 0;
double pressure = 0;
double pressure_diff = 0;
double prev_pressure = 0;
RollingMemory pres_av;
RollingMemory pres_diff_av;

// Altitude acceleration measurements
double bmx055_initial_acc = 0;
double mpu6050_initial_acc = 0;
double comp_acc_Z = 0;
double lpf_acc_Z = 0;
double other_lpf_acc_Z = 0;
RollingMemory acc_av;
RollingMemory acc_av2;

double alt_rate_acc = 0;
double alt_rate = 0;
double alt_rate_filter = 0;
uint8_t acc_counter = 0;

// Orientation PID calculation variables
double roll_integral = 0;
double roll_prev_error = 0;
double pitch_integral = 0;
double pitch_prev_error = 0;
double yaw_integral = 0;
double yaw_prev_error = 0;

// Altitude PID calculation variables
double alt_integral = 0;
//double alt_prev_error = 0;
double alt_prev_diff = 0;
double alt_setpoint = 0;
double alt_throttle_pid = 0;


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
  Wire1.begin(9, 15, 400000); // SDA = 9, SCL = 15

  // ESC initialization, all set to 1000ms pulse
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

  // Start user interface while ESCs set up
  #if WIFI_GUI
  Blynk.begin(auth, ssid, pass);
  #else
  Blynk.begin(auth);
  #endif

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
    bmx055_initial_acc += imu.accelerometer.z*imu.accelerometer.res;
    delay(4);
  }
  bmx055_initial_acc /= 100.0;
  acc_av.samples = 20.0;
  acc_av2.samples = 20.0;

  // Second IMU initialization
  mpu.begin(&Wire1);
  for(int i = 0; i < 100; i++)
  {
    mpu.get_acc_data();  
    mpu6050_initial_acc -= mpu.sensed.acc_z;
    delay(4);
  }
  mpu6050_initial_acc /= 100.0;

  Serial.println(bmx055_initial_acc);
  Serial.println(mpu6050_initial_acc);

  // Altitude sensor initialization
  ms5611.begin();
  ms5611.setOversampling(MS5611_ULTRA_HIGH_RES);
  pres_diff_av.samples = 30;
  // needs some initial readings to stabilize measurement
  for(int i = 0; i<pres_av.samples; i++)
  {
    pressure = ms5611.readPressure();
    pres_av.memory[i] = pressure;
    pres_av.sum += pres_av.memory[i];
    prev_pressure = pressure;
  }
  filtered_pressure = pres_av.sum/pres_av.samples;
  delay(2000);

  // Get the first measurements for future calculations
  ms5611.requestTemperature();
  delay(ALTITUDE_SAMPLING*1000.0);
  raw_temp = ms5611.readRawTemperature();
  ms5611.requestPressure();
  delay(ALTITUDE_SAMPLING*1000.0);

  // Orientation timer
  orientation_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(orientation_timer, &orientation_isr, true);
  timerAlarmWrite(orientation_timer, ORIENTATION_SAMPLING*1000000.0, true);
  //orientation.set_yaw_offset(imu.magnetometer.yaw_offset);
  orientation.init(0, 0, MAG_TARGET);

  while(throttle < 1100)
  {
    Blynk.run();
  }
  
  // Start timer
  timerAlarmEnable(orientation_timer);
  
  orientation_prev_time = millis();
  altitude_prev_time = millis();
}

void loop() {
  
  Blynk.run();

  if(ui_callback)
  {    
    roll_integral = 0;
    roll_prev_error = 0;
    pitch_integral = 0;
    pitch_prev_error = 0;
    yaw_integral = 0;
    yaw_prev_error = 0;
    ui_callback = 0;
  }
  if(alt_callback)
  {
    if(alt_hold)
    {
      alt_setpoint = pressure;
      alt_integral = 0;
    }
    else
    {
      alt_throttle_pid = 0;
    }
    
    alt_callback = 0;
  }

  if(eStop)
  {
    ledcWrite(ledChannelA, MS_TO_PWM(1000));
    ledcWrite(ledChannelB, MS_TO_PWM(1000));
    ledcWrite(ledChannelC, MS_TO_PWM(1000));
    ledcWrite(ledChannelD, MS_TO_PWM(1000));
  }
  
  if(update_orientation)
  {
    unsigned long current_time = millis();
    double dt = (current_time - orientation_prev_time)/1000.0;
    orientation_prev_time = millis();
    
    mpu.get_acc_data();
    double mpu6050x =  -1.0*mpu.sensed.acc_y;
    double mpu6050y =  -1.0*mpu.sensed.acc_x;
    double mpu6050z =  -1.0*mpu.sensed.acc_z;
    
    imu.get_gyr_data();
    imu.get_acc_data();
    //imu.get_mag_data();

    
    orientation.fuse_sensors(imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z,
                            imu.gyroscope.x*imu.gyroscope.res, imu.gyroscope.y*imu.gyroscope.res, imu.gyroscope.z*imu.gyroscope.res,
                            imu.magnetometer.x, imu.magnetometer.y, imu.magnetometer.z);
    
    roll_rate = roll_rate*0.7 + imu.gyroscope.x*imu.gyroscope.res*0.3;
    pitch_rate = pitch_rate*0.7 + imu.gyroscope.y*imu.gyroscope.res*0.3;
    yaw_rate = yaw_rate*0.7 + imu.gyroscope.z*imu.gyroscope.res*0.3;

  
    double rolly = orientation.get_roll();
    double pitchy = orientation.get_pitch();
    double bmx055_az = (imu.accelerometer.z*cos(pitchy)*cos(rolly) - imu.accelerometer.x*sin(pitchy) + imu.accelerometer.y*cos(pitchy)*sin(rolly))*imu.accelerometer.res - bmx055_initial_acc;// - 9.7;
    double mpu6050_az = mpu6050z*cos(pitchy)*cos(rolly) - mpu6050x*sin(pitchy) + mpu6050y*cos(pitchy)*sin(rolly) - mpu6050_initial_acc;// - 9.7;
    
    comp_acc_Z = 0.5*bmx055_az + 0.5*mpu6050_az;
    
    acc_av.sum -= acc_av.memory[acc_av.index];
    acc_av.memory[acc_av.index] = comp_acc_Z;
    acc_av.sum += acc_av.memory[acc_av.index];
    acc_av.index++;
    if(acc_av.index == acc_av.samples) acc_av.index = 0;
    double acc_z_av = acc_av.sum/acc_av.samples;
    lpf_acc_Z = 0.65*lpf_acc_Z + 0.36*acc_z_av;

    acc_av2.sum -= acc_av2.memory[acc_av2.index];
    acc_av2.memory[acc_av2.index] = bmx055_az;
    acc_av2.sum += acc_av2.memory[acc_av2.index];
    acc_av2.index++;
    if(acc_av2.index == acc_av2.samples) acc_av2.index = 0;
    double acc_z_av2 = acc_av2.sum/acc_av2.samples;
    other_lpf_acc_Z = 0.65*other_lpf_acc_Z + 0.36*acc_z_av2;
    
    alt_rate_acc -= acc_z_av*dt;
    /*if(abs(az) < 0.75)
    {
      acc_counter++;
    }
    else
    {
      acc_counter = 0;
    }
    if(acc_counter == 5)
    {
      alt_rate_acc = 0;
      acc_counter = 0;
    }*/
    double anewcomp = 0.5*lpf_acc_Z + 0.5*other_lpf_acc_Z;
    Serial.print(lpf_acc_Z);
    Serial.print(" ");
    Serial.print(other_lpf_acc_Z);
    Serial.print(" ");
    Serial.println(anewcomp);
    
    //alt_rate = 0.98*alt_rate_acc + 0.02*pressure_diff;
    //alt_rate = 0.3*alt_rate_acc + 0.7*pressure_diff;
    alt_rate = alt_rate_acc;
    alt_rate_filter = 0.8*alt_rate_filter + 0.2*alt_rate;

    double ma = throttle;
    double mb = throttle;
    double mc = throttle;
    double md = throttle;

    double roll_rate_setpoint = kr*(roll_setpoint - orientation.get_roll()*180.0/PI);
    double pitch_rate_setpoint = kr*(pitch_setpoint - orientation.get_pitch()*180.0/PI);
    double yaw_rate_setpoint = 0;//kr*(yaw_setpoint - orientation.get_yaw()*180.0/PI);
    
    if(throttle >= 1100)
    {
      // Roll PID
      double roll_error = roll_rate_setpoint - roll_rate;
      double roll_diff = (roll_error - roll_prev_error)/dt;
      roll_integral += roll_error*dt;
      //roll_integral = constrain(roll_integral, -100, 100);
      roll_prev_error = roll_error;
      double roll_pid = kp_roll*roll_error + ki_roll*roll_integral + kd_roll*roll_diff;

      // Pitch PID
      double pitch_error = pitch_rate_setpoint - pitch_rate;
      double pitch_diff = (pitch_error - pitch_prev_error)/dt;
      pitch_integral += pitch_error*dt;
      //pitch_integral = constrain(roll_integral, -100, 100);
      pitch_prev_error = pitch_error;
      double pitch_pid = kp_pitch*pitch_error + ki_pitch*pitch_integral + kd_pitch*pitch_diff;

      // Yaw PID
      double yaw_error = yaw_rate_setpoint - yaw_rate;
      double yaw_diff = (yaw_error - yaw_prev_error)/dt;
      yaw_integral += yaw_error*dt;
      yaw_prev_error = yaw_error;
      double yaw_pid = kp_yaw*yaw_error + ki_yaw*yaw_integral + kd_yaw*yaw_diff;

      ma = constrain(throttle - alt_throttle_pid - roll_pid/4.0 - pitch_pid/4.0 + yaw_pid/4.0, 1100, 2000);
      mb = constrain(throttle - alt_throttle_pid - roll_pid/4.0 + pitch_pid/4.0 - yaw_pid/4.0, 1100, 2000);
      mc = constrain(throttle - alt_throttle_pid + roll_pid/4.0 + pitch_pid/4.0 + yaw_pid/4.0, 1100, 2000);
      md = constrain(throttle - alt_throttle_pid + roll_pid/4.0 - pitch_pid/4.0 - yaw_pid/4.0, 1100, 2000);

      /*ma = constrain(throttle - roll_pid/4.0 - pitch_pid/4.0 + yaw_pid/4.0, 1100, 2000);
      mb = constrain(throttle - roll_pid/4.0 + pitch_pid/4.0 - yaw_pid/4.0, 1100, 2000);
      mc = constrain(throttle + roll_pid/4.0 + pitch_pid/4.0 + yaw_pid/4.0, 1100, 2000);
      md = constrain(throttle + roll_pid/4.0 - pitch_pid/4.0 - yaw_pid/4.0, 1100, 2000);*/

      /*Serial.print(roll_pid/4.0);
      Serial.print(" ");
      Serial.print(pitch_pid/4.0);
      Serial.print(" ");*/
      //Serial.println(throttle + alt_throttle_pid);
      
    }

    if(!eStop)
    {
      ledcWrite(ledChannelA, MS_TO_PWM(ma));
      ledcWrite(ledChannelB, MS_TO_PWM(mb));
      ledcWrite(ledChannelC, MS_TO_PWM(mc));
      ledcWrite(ledChannelD, MS_TO_PWM(md));
    }
    
    update_orientation = 0;
  }

  if(update_altitude)
  {
    
    
    unsigned long current_time = millis();
    double dt = (current_time - altitude_prev_time)/1000.0;
    
    if(dt*1000.0 > 9.2)
    {
      altitude_prev_time = millis();
      tp_counter++;
      
      if(tp_counter == 20)
      {
        raw_temp = ms5611.readRawTemperature();
        ms5611.requestPressure();
        tp_counter = 0;
      }
      else if(tp_counter == 19)
      {
        unfiltered_pressure = ms5611.readPressure(raw_temp);
        ms5611.requestTemperature();
      }
      else
      {
        unfiltered_pressure = ms5611.readPressure(raw_temp);
        ms5611.requestPressure();
      }
  
      pres_av.sum -= pres_av.memory[pres_av.index];
      pres_av.memory[pres_av.index] = unfiltered_pressure;
      pres_av.sum += pres_av.memory[pres_av.index];
      pres_av.index++;
      if(pres_av.index == pres_av.samples) pres_av.index = 0;
      double fast_pressure = pres_av.sum/pres_av.samples;

      filtered_pressure = 0.985*filtered_pressure + 0.015*fast_pressure;

      double measured_pressure_diff = filtered_pressure - fast_pressure;
      measured_pressure_diff = constrain(measured_pressure_diff, -8, 8);
      if(abs(measured_pressure_diff)>1) filtered_pressure -= measured_pressure_diff/6.0;
      pressure = filtered_pressure;

      pres_diff_av.sum -= pres_diff_av.memory[pres_diff_av.index];
      pres_diff_av.memory[pres_diff_av.index] = (unfiltered_pressure - prev_pressure)/dt;
      pres_diff_av.sum += pres_diff_av.memory[pres_diff_av.index];
      prev_pressure = unfiltered_pressure;
      pres_diff_av.index++;
      if(pres_diff_av.index == pres_diff_av.samples) pres_diff_av.index = 0;
      pressure_diff = pres_diff_av.sum/pres_diff_av.samples;

  
      // Calculate PID if alt_hold is activated
      if(alt_hold)
      {
        alt_setpoint = 0;
        // Error and integral of error
        double alt_error = alt_setpoint - pressure_diff;
        double alt_diff_error = (alt_error - alt_prev_diff)/dt;
        alt_integral += alt_error*dt;
        alt_prev_diff = alt_error;

        /*double kp_gain_altitude = 0;
        if (alt_error > 10 || alt_error < -10) {                             //If the error between the setpoint and the actual pressure is larger than 10 or smaller then -10.
          kp_gain_altitude = (abs(alt_error) - 10) / 20.0;                 //The positive pid_error_gain_altitude variable is calculated based based on the error.
          if (kp_gain_altitude > 3)kp_gain_altitude = 3;                 //To prevent extreme P-gains it must be limited to 3.
        }*/
        
        alt_throttle_pid = kp_alt*alt_error + ki_alt*alt_integral + kd_alt*alt_diff_error;
        alt_throttle_pid = constrain(alt_throttle_pid, -100, 100);
        
        //alt_throttle_pid = kp_alt*alt_error + ki_alt*alt_integral + kd_alt*alt_lpf_diff;
        /*if((throttle + alt_throttle_pid) > 1430)
        {
          alt_throttle_pid = 1420-throttle;
        }
        if((throttle + alt_throttle_pid) < 1350)
        {
          alt_throttle_pid = 1370-throttle;
        }*/
        
      }
      update_altitude = 0;
    }
  }
}
