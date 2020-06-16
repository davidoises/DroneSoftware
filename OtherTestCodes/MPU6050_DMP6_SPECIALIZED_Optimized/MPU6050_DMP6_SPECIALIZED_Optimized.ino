#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// UI Control variables
double eStop = 1;
double throttle = 1000.0;
double roll = 1500.0;
double pitch = 1500.0;
double yaw = 1500.0;
double aux1 = 1000.0;
double ui_callback = 0;

#include "ui_conf.h"

#define ORIENTATION_SAMPLING 0.004f

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 5/3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the ESP8266 GPIO15
   pin.
 * ========================================================================= */

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector


// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_TEAPOT_YAWPITCHROLL

#define OUTPUT_READABLE_GYRO

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL


float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float prev_roll = 0;
float prev_pitch = 0;
float prev_yaw = 0;
float gyr[3];
float acc[3];

#define INTERRUPT_PIN 19 // use pin 15 on ESP8266

const char DEVICE_NAME[] = "mpu6050";

unsigned long prev_imu_time = 0;
unsigned long prev_pid_time = 0;

TaskHandle_t imu_handle = NULL;
TaskHandle_t blynk_handle = NULL;


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void ICACHE_RAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

hw_timer_t * orientation_timer = NULL;
volatile uint8_t update_orientation;
void IRAM_ATTR orientation_isr() {
  update_orientation = 1;
}


void blynkLoop(void *pvParameters ) {  //task to be created by FreeRTOS and pinned to core 0
  while (true) {
    //Serial.println("hola");
    Blynk.run();
    vTaskDelay(50);
  }
}
void mpu_loop(void *pvParameters )
{
  while(true)
  {
    // if programming failed, don't try to do anything
    //if (!dmpReady) return;
    while(!dmpReady)
    {
      vTaskDelay(50);
    }
  
    // wait for MPU interrupt or extra packet(s) available
    //if (!mpuInterrupt && fifoCount < packetSize) return;
    while(!mpuInterrupt && fifoCount < packetSize)
    {
      vTaskDelay(50);
    }
  
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
  
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
  
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));
  
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
  
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
  
      unsigned long current_time = millis();
      float dt = (current_time - prev_imu_time)/1000.0;
      prev_imu_time = current_time;

      // Gyroscope values obtention
      int16_t temp[3];
      mpu.dmpGetGyro(temp, fifoBuffer);
      gyr[0] = (float)temp[0];
      gyr[1] = (float)temp[1];
      gyr[2] = (float)temp[2];
      
      // Yaw, pitch, roll obtention
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      // Gyroscope derivated values
      acc[0] = (gyr[0] - prev_roll)/dt;
      acc[1] = (gyr[1] - prev_pitch)/dt;
      acc[2] = (gyr[2] - prev_yaw)/dt;
      prev_roll = gyr[0];
      prev_pitch = gyr[1];
      prev_yaw = gyr[2];

      // Acceleration obtention
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity); // Rotates pitch and roll
      //mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q); // rotates yaw
    }
  }
}

void setup(void)
{
  //Serial.begin(115200);
  Serial.begin(1000000);
  Serial.println(F("\nOrientation Sensor OSC output")); Serial.println();
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // Start user interface while ESCs set up
  #if WIFI_GUI
  Blynk.begin(auth, ssid, pass);
  #else
  Blynk.begin(auth);
  #endif
  
  //Serial.println(ESP.getFreeHeap());

  orientation_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(orientation_timer, &orientation_isr, true);
  timerAlarmWrite(orientation_timer, ORIENTATION_SAMPLING*1000000.0, true);
  
  //this is where we start the Blynk.run() loop pinned to core 0, given priority "1" (which gives it thread priority over "0")
  xTaskCreatePinnedToCore(
    blynkLoop,      /* Function to implement the task */
    "blynk core 0", /* Name of the task */
    5000,         /* Stack size in words */
    NULL,           /* Task input parameter */
    1,              /* Priority of the task */
    &blynk_handle,           /* Task handle. */
    0);             /* Core where the task should run */

  //Serial.println(ESP.getFreeHeap());
  
  xTaskCreatePinnedToCore(
    mpu_loop,      /* Function to implement the task */
    "IMU core 0", /* Name of the task */
    5000,         /* Stack size in words */
    NULL,           /* Task input parameter */
    2,              /* Priority of the task */
    &imu_handle,           /* Task handle. */
    0);             /* Core where the task should run */

  timerAlarmEnable(orientation_timer);
  Serial.println("Starting");
  //total heap = 177360

  prev_imu_time = millis();
  prev_pid_time = millis();
}

void loop(void)
{
  if(update_orientation)
  {
    unsigned long current_time = millis();
    float dt = current_time - prev_pid_time;
    prev_pid_time = current_time;

    Serial.print(uxTaskGetStackHighWaterMark(blynk_handle));
    Serial.print("\t");
    Serial.print(uxTaskGetStackHighWaterMark(imu_handle));
    Serial.print("\t");
    Serial.print(dt);
    Serial.print("\t");
    Serial.print(throttle);
    Serial.print("\t");

    float roll = ypr[2];
    float pitch = -ypr[1];
    float yaw = -ypr[0];
    
    Serial.print(yaw * 180/M_PI); // yaw
    Serial.print(",");
    Serial.print(pitch * 180/M_PI); // pitch
    Serial.print(",");
    Serial.println(roll * 180/M_PI); // roll
    //Serial.print(",");
    //Serial.println(gyr[2]); // yaw
    //Serial.print(",");
    //Serial.println(gyr[1]); // pitch
    //Serial.print(",");
    //Serial.print(gyr[0]); // roll
    //Serial.print(",");
    //Serial.println(acc[2]);
    //Serial.print(",");
    //Serial.println(acc[1]);
    //Serial.print(",");
    //Serial.println(acc[0]);

    /*
    Serial.print(aa.x);
    Serial.print(",");
    Serial.print(aa.y);
    Serial.print(",");
    Serial.print(aa.z);
    Serial.print(",");
    Serial.print(aaReal.x);
    Serial.print(",");
    Serial.print(aaReal.y);
    Serial.print(",");
    Serial.println(aaReal.z);
    */

    update_orientation = 0;
  }
  //Serial.println(uxTaskGetStackHighWaterMark(blynk_handle));
  //delay(50);
}
