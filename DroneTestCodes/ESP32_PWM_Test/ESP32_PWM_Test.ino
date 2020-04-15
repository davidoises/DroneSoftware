#define PWMA 32
#define PWMB 33
#define PWMC 14
#define PWMD 10

// the number of the LED pin
const int ledPin = PWMC;  // 16 corresponds to GPIO16

// Delay between duty cycle steps
const int delay_ms = 50;

// setting PWM properties
const int freq = 50;
const int ledChannel = 0;
const int resolution = 12;

// Variables for ESC control
const double max_pwm = pow(2, resolution)-1;
const double max_period = 1000.0/freq; //ms, 50hz = 20ms
int min_rpm = 1.0*max_pwm/max_period;
int max_rpm = 2.0*max_pwm/max_period;
 
void setup(){
  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin, ledChannel);

  ledcWrite(ledChannel, min_rpm);
  delay(10000);
}
 
void loop(){
  // increase the LED brightness
  for(int dutyCycle = min_rpm; dutyCycle <= max_rpm; dutyCycle++){   
    // changing the LED brightness with PWM
    ledcWrite(ledChannel, dutyCycle);
    delay(delay_ms);
  }

  // decrease the LED brightness
  for(int dutyCycle = max_rpm; dutyCycle >= min_rpm; dutyCycle--){
    // changing the LED brightness with PWM
    ledcWrite(ledChannel, dutyCycle);   
    delay(delay_ms);
  }
}
