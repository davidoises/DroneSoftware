#include "CC1125.h"

CC1125 rf_comm;

bool msg_flag = false;
uint8_t rxBuffer[128] = {0};
uint8_t pkt_size = 0;

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
  
  rf_comm.begin();
  Serial.println("CC1125 Initialized!");
  rf_comm.manualCalibration();
  Serial.println("Finished calibration");
  rf_comm.receive();
  Serial.println("Available for message recpetion");

  //Antes 4 = RF INT or 15
  pinMode(4, INPUT_PULLUP);
  attachInterrupt(4, msg_flag_isr, FALLING);
}

void loop() {
  
  // put your main code here, to run repeatedly:
  if(msg_flag)
  {
    rf_comm.get_packet(rxBuffer, pkt_size);
    msg_flag = false;
  }

  if(pkt_size != 0)
  {
    for(int i = 0; i < pkt_size; i++)
    {
      Serial.print("0x");
      Serial.print(rxBuffer[i], HEX);
      Serial.print(", ");
    }
    Serial.println("");
    //pkt_size = 0;
  }

}
