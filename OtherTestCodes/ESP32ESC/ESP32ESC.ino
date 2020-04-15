#define CANAL_A 13  
#define CANAL_B 12
#define CANAL_C 16
#define CANAL_D 17
#define CANAL_E 19
#define CANAL_F 21

#define IN1 0
#define IN2 1
#define IN3 2
#define IN4 3
#define IN5 4
#define IN6 5

#define RES 8
#define FREQ 1000
#define MIN_PWM 0
//#define MIN_PWM 35

#define PWM_PIN1 23//27
#define PWM_PIN2 22//14
#define PWM_PIN3 4
#define PWM_PIN4 2
#define PWM_PIN5 27//22
#define PWM_PIN6 14//23

#define FLOAT   0
#define DOWN    1
#define UP      2

uint8_t seq_step = 0;

void polarize(uint8_t hs, uint8_t ls, uint8_t state)
{
  switch(state)
  {
    case FLOAT:
    {
      digitalWrite(hs, LOW);
      digitalWrite(ls, HIGH);
      break;
    }
    case DOWN:
    {
      digitalWrite(hs, LOW);
      digitalWrite(ls, LOW);
      break;
    }
    case UP:
    {
      digitalWrite(hs, HIGH);
      digitalWrite(ls, HIGH);
      break;
    }
  }
}

void set_a(uint8_t state)
{
  polarize(PWM_PIN1, PWM_PIN2, state);
}

void set_b(uint8_t state)
{
  polarize(PWM_PIN3, PWM_PIN4, state);
}

void set_c(uint8_t state)
{
  polarize(PWM_PIN5, PWM_PIN6, state);
}

void setup() { 
  Serial.begin(115200); 
  
  pinMode(PWM_PIN1, OUTPUT);
  pinMode(PWM_PIN2, OUTPUT);
  
  pinMode(PWM_PIN3, OUTPUT);
  pinMode(PWM_PIN4, OUTPUT);

  pinMode(PWM_PIN5, OUTPUT);
  pinMode(PWM_PIN6, OUTPUT);
} 

void loop()
{
  switch(seq_step)
  {
    case 0:
    {
      set_a(UP);
      set_b(DOWN);
      set_c(FLOAT);
      seq_step++;
      break; 
    }
    case 1:
    {
      set_a(UP);
      set_b(FLOAT);
      set_c(DOWN);
      seq_step++;
      break;
    }
    case 2:
    {
      set_a(FLOAT);
      set_b(UP);
      set_c(DOWN);
      seq_step++;
      break;
    }
    case 3:
    {
      set_a(DOWN);
      set_b(UP);
      set_c(FLOAT);
      seq_step++;
      break;
    }
    case 4:
    {
      set_a(DOWN);
      set_b(FLOAT);
      set_c(UP);
      seq_step++;
      break;
    }
    case 5:
    {
      set_a(FLOAT);
      set_b(DOWN);
      set_c(UP);
      seq_step = 0;
      break;
    }
  }
  //delay(1);
  delayMicroseconds(100);
}
