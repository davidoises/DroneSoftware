#define RXD2 9
#define TXD2 15

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
#define FREQ 50
#define RESOLUTION 12
#define ledChannelA 0

// Constants for ESC control
#define MAX_PWM (1<<RESOLUTION)-1
#define MAX_PERIOD 1000.0f/((float)FREQ)
#define MS_TO_PWM(x) ((float)x)*((float)MAX_PWM)/(((float)MAX_PERIOD)*1000.0f)

// Variables for serial messages synchronization
bool started = false;
String message = "";
double pwm = 1000;
 
void setup(){
  Serial.begin(500000, SERIAL_8N1);
  Serial2.begin(500000, SERIAL_8N1, RXD2, TXD2);
  delay(100);
  Serial.println("");
  
  ledcSetup(ledChannelA, FREQ, RESOLUTION);
  ledcAttachPin(SERVO1_X, ledChannelA);
  ledcWrite(ledChannelA, MS_TO_PWM(1000));
}
 
void loop(){
  
  // Read and parse from serial
  while(Serial.available())
  {
    char temp = char(Serial.read());
    if(temp == '=')
    {
      started = true;
    }
    else if(temp == '\n')
    {
      started = false;
      pwm = message.toDouble();
      message = "";
    }
    else if(started)
    {
      message += temp;
    }
  }

  Serial2.println(analogRead(ADCA)*33.0/4095.0);

  // Write PWM
  ledcWrite(ledChannelA, MS_TO_PWM(pwm));
}
