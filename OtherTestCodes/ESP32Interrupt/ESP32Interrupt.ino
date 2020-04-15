bool msg_flag = false;
int counter = 0;

//void IRAM_ATTR msg_flag_isr()
void msg_flag_isr()
{
  msg_flag = true; 
}

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(115200);
  Serial.begin(500000);
  delay(500);

  //Antes 4 = RF INT or 15
  pinMode(4, INPUT_PULLUP);
  attachInterrupt(4, msg_flag_isr, CHANGE);
}

void loop() {
  
  // put your main code here, to run repeatedly:
  if(msg_flag)
  {
    counter++;
    Serial.println(counter);
    msg_flag = false;
  }

}
