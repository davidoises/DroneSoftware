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

// General Definitation for more flexibility
#define PWM_PIN PWMD
#define ADC_PIN ADCD

// setting PWM properties
#define FREQ 50
#define RESOLUTION 12
#define ledChannelA 0

// Constants for ESC control
#define MAX_PWM (1<<RESOLUTION)-1
#define MAX_PERIOD 1000.0f/((double)FREQ)
#define MS_TO_PWM(x) ((double)x)*((double)MAX_PWM)/(((double)MAX_PERIOD)*1000.0f)

// sine input
double pwm_offset = 1000;
double pwm = 0;
double step_delay = 1500; // millisecods
hw_timer_t * update_timer = NULL;
unsigned long counter = 0;

// Time measurement
hw_timer_t * sampling_timer = NULL;

// Timer ISR
void IRAM_ATTR test_isr() {
  switch(counter)
  {
    case 0:
      pwm = 1300;
      break;
    case 1:
      pwm = 1000;
      break;
    case 2:
      pwm = 1400;
      break;
    case 3:
      pwm = 1200;
      break;
    case 4:
      pwm = 1300;
      break;
    case 5:
      pwm = 1000;
      break;
    case 6:
      pwm = 1250;
      break;
    case 7:
      pwm = 1400;
      break;
    case 8:
      pwm = 1100;
      break;
    case 9:
      pwm = 1000;
      counter = -1;
      break;
  }
  ledcWrite(ledChannelA, MS_TO_PWM(pwm));
  counter++;
}

// Sampling ISR
void IRAM_ATTR sampling_isr() {
  double voltage = analogRead(ADC_PIN)*33.0/4095.0;
  double force = 0.0095*sq(voltage) + 0.0068*voltage;
  Serial.print(pwm - pwm_offset);
  Serial.print(" ");
  Serial.println(force);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);

  pwm = pwm_offset;
  ledcSetup(ledChannelA, FREQ, RESOLUTION);
  ledcAttachPin(PWM_PIN, ledChannelA);
  ledcWrite(ledChannelA, MS_TO_PWM(pwm));

  update_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(update_timer, &test_isr, true);
  timerAlarmWrite(update_timer, step_delay*1000.0, true);

  sampling_timer = timerBegin(1, 80, true);
  timerAttachInterrupt(sampling_timer, &sampling_isr, true);
  timerAlarmWrite(sampling_timer, 5000.0, true);
  timerAlarmEnable(sampling_timer);

  // Wait for an "s" to start test
  while(!Serial.available());
  while(char(Serial.read()) != 's');

  timerAlarmEnable(update_timer);
}

void loop() {
}
