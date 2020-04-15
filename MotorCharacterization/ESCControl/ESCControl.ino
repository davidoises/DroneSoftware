#define RXD2 9//16
#define TXD2 15// 17

// HW Pins
#define PWMA 25//32
#define PWMB 33
#define PWMC 14
#define PWMD 10

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
  Serial.begin(115200, SERIAL_8N1);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  delay(100);
  Serial.println("");
  
  ledcSetup(ledChannelA, FREQ, RESOLUTION);
  ledcAttachPin(PWMA, ledChannelA);
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

  Serial2.println(analogRead(35)*33.0/4095.0);

  // Write PWM
  ledcWrite(ledChannelA, MS_TO_PWM(pwm));
}
