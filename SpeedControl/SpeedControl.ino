// HW Pins
#define PWMA 32
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

// Serial Pins
#define RXD2 9//16
#define TXD2 15// 17
#define EXTERNAL_ADC 0 
#define ADC_PIN 35

// Variables for serial messages synchronization
bool started = false;
String message = "";
double voltage = 0;

// Time measurement
hw_timer_t * pid_timer = NULL;

// PID variables
double pwm = 1000.0;
double prev_error = 0;
double integral = 0;
double set_point = 0;

/*double kp = 198.5112;
double ki = 827.13;
double kd = 0;*/

/*
double kp = 22.2730;
double ki = 69.603;
double kd = 0;
*/

double kp = 26.0576;
double ki = 76.64;
double kd = 0;

// Sampling ISR
void IRAM_ATTR pid_isr() {
  
  double error = set_point - voltage;
  double proportional = kp*error;
  double derivative = kd*(error - prev_error)/0.005;
  integral +=  ki*(error + prev_error)*(0.005/2.0);
  prev_error = error;

  double pwm_control_law = proportional + integral + derivative;
  pwm = constrain(1000.0 + pwm_control_law, 1000.0, 2000.0);

  Serial.print(set_point);
  Serial.print(" ");
  Serial.println(voltage);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  ledcSetup(ledChannelA, FREQ, RESOLUTION);
  ledcAttachPin(PWMA, ledChannelA);
  ledcWrite(ledChannelA, MS_TO_PWM(1000.0));

  // Wait for an "s" to start test
  while(!Serial.available());
  while(char(Serial.read()) != 's');
  
  pid_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(pid_timer, &pid_isr, true);
  timerAlarmWrite(pid_timer, 5000.0, true);
  timerAlarmEnable(pid_timer);
}

void loop() {
#if EXTERNAL_ADC
  // Read sensor value
  while(Serial2.available())
  {
    char temp = char(Serial2.read());
    if(temp == '=')
    {
      started = true;
    }
    else if(temp == '\n')
    {
      started = false;
      voltage = message.toDouble();
      message = "";
    }
    else if(started)
    {
      message += temp;
    }
  }
#else
  voltage = analogRead(ADC_PIN)*33.0/4095.0;
#endif

  while(Serial.available())
  {
    set_point = Serial.parseFloat();
  }
  
  ledcWrite(ledChannelA, MS_TO_PWM(pwm));
}
