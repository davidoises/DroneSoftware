#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define INTERRUPT_PIN 15

RF24 radio(4, 5, 18, 19, 23); // CE, CSN, SCK, MISO, MOSI
const byte address[6] = "00001";
unsigned long prev_time = 0;

bool msg_flag = false;
void msg_flag_isr()
{
  msg_flag = true; 
}


void setup() {
  Serial.begin(115200);
  
  radio.begin();
  radio.maskIRQ(1,1,0);
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(INTERRUPT_PIN, msg_flag_isr, FALLING);
  
  prev_time = millis();
}

void loop() {
  //if (radio.available()) {
  if(msg_flag || 1)
  {
    unsigned long current_time = millis();
    int dt = current_time - prev_time;
    prev_time = current_time;
    
    char text[4] = "";
    radio.read(&text, sizeof(text));
    
    Serial.print(dt);
    Serial.print(" ");
    Serial.println(String(text));
    msg_flag = false;
  }
}
