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

// Variables for serial messages synchronization
bool started = false;
String message = "";
double voltage = 0;

// sine input
double pwm_offset = 1000;
double pwm = 0;
double step_delay = 50; // millisecods
hw_timer_t * update_timer = NULL;
unsigned long counter = 0;

// Time measurement
unsigned long prev_time = 0;
hw_timer_t * sampling_timer = NULL;

// Timer ISR
void IRAM_ATTR test_isr() {
  // sine from 1100 to 1300 pwm signal
  pwm = 100.0*sin(counter/20.0) + 1200.0;
  counter++;
}

// Sampling ISR
void IRAM_ATTR sampling_isr() {
  // Calculate the elapsed time
  unsigned long current_time = millis();
  double t_delta = ((double)current_time - (double)prev_time)/1000;
  prev_time = current_time;

  Serial.print(t_delta, 3);
  Serial.print(" ");
  Serial.print(pwm - pwm_offset);
  Serial.print(" ");
  Serial.println(voltage);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  pwm = pwm_offset;
  ledcSetup(ledChannelA, FREQ, RESOLUTION);
  ledcAttachPin(PWMA, ledChannelA);
  ledcWrite(ledChannelA, MS_TO_PWM(pwm));

  update_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(update_timer, &test_isr, true);
  timerAlarmWrite(update_timer, step_delay*1000.0, true);

  sampling_timer = timerBegin(1, 80, true);
  timerAttachInterrupt(sampling_timer, &sampling_isr, true);
  timerAlarmWrite(sampling_timer, 5000.0, true);
  timerAlarmEnable(sampling_timer);

  prev_time = millis();
}

void loop() {  
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

  // Wait for an "s" to start test
  while(Serial.available())
  {
    if(char(Serial.read()) == 's')
    {
      // Start timer every step_delay time
      timerAlarmEnable(update_timer);
    }
  }

  ledcWrite(ledChannelA, MS_TO_PWM(pwm));
}
