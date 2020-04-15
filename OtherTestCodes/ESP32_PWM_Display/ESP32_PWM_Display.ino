#define INTERRUPT_PIN 4

volatile uint8_t pwm = 0;
volatile uint8_t flag = 0;

void IRAM_ATTR pwm_isr()
{
  pwm = digitalRead(INTERRUPT_PIN); 
  flag = 1;
}

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(115200);
  Serial.begin(500000);
  delay(500);

  //Antes 4 = RF INT
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(INTERRUPT_PIN, pwm_isr, CHANGE);
}

void loop() {
  Serial.println(pwm);
  
  /*if(flag)
  {
    Serial.println("Received");
    flag = 0;
  }*/
  
}
