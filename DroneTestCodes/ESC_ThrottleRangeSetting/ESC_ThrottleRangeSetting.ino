#define PWMA 32
#define PWMB 33
#define PWMC 14
#define PWMD 10

// Delay between duty cycle steps
const int delay_ms = 15;

// setting PWM properties
const int freq = 50;
const int resolution = 12;
const int ledChannelA = 0;
const int ledChannelC = 1;

// Variables for ESC control
const double max_pwm = pow(2, resolution)-1;
const double max_period = 1000.0/freq; //ms, 50hz = 20ms
int min_rpm = 1.0*max_pwm/max_period;
int max_rpm = 2.0*max_pwm/max_period;
 
void setup(){
  Serial.begin(115200);
  Serial.println("");
  Serial.println("");
  Serial.println("Starting throttle range calibration");
  Serial.println("Press any char to change from min to max PWM or viceversa");
  
  ledcSetup(ledChannelA, freq, resolution);
  ledcAttachPin(PWMA, ledChannelA);

  ledcSetup(ledChannelC, freq, resolution);
  ledcAttachPin(PWMC, ledChannelC);

  /* Beginning of throttle range calibration */
  // Starting at minimum throttle
  ledcWrite(ledChannelA, max_rpm);
  ledcWrite(ledChannelC, max_rpm);
  Serial.print("Current PWM = ");
  Serial.println(max_rpm);

  // Wait for input to change to next step
  while(!Serial.available())
  {}
  char c = (char) Serial.read();
  
  // Changing to maximum throttle
  ledcWrite(ledChannelA, min_rpm);
  ledcWrite(ledChannelC, min_rpm);
  Serial.print("Current PWM = ");
  Serial.println(min_rpm);

  /*
  // Wait for input to change to next step
  while(!Serial.available())
  {}
  c = (char) Serial.read();
  
  // Changing to minimum throttle
  ledcWrite(ledChannel, min_rpm);
  Serial.print("Current PWM = ");
  Serial.println(min_rpm);
  */

  Serial.println("Finished calibration!");
}
 
void loop(){
}
