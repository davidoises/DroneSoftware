#include "CC1125.h"

CC1125 rf_comm;

// Transmission timer
hw_timer_t * timer = NULL;
uint8_t reached_timer;
void IRAM_ATTR onTimer() {
  reached_timer = 1;
}

// Message buffers
uint8_t rxBuffer[128] = {0};
uint8_t txBuffer[128] = {0};
uint8_t pkt_size = 0;

// reception ISR
bool msg_flag = false;
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

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000, true);
  //timerAlarmEnable(timer);
}

void loop() {

  if(reached_timer)
  {
    txBuffer[1] = 0xAA;// Seems not important
    txBuffer[2] = 0xFF;// Seems not important
    txBuffer[3] = 0xBB;
    txBuffer[4] = 0xCC;
    uint8_t len = 5;
    txBuffer[0] = len -1;
    rf_comm.sendPacket(txBuffer, len);
    Serial.println("Sent");

    reached_timer = 0;
  }
  
  if(msg_flag)
  {
    rf_comm.get_packet(rxBuffer, pkt_size);
    msg_flag = false;
  }

  if(pkt_size != 0)
  {
    for(int i = 0; i < pkt_size; i++)
    {
      //Serial.print("0x");
      //Serial.print(rxBuffer[i], HEX);
      //Serial.print(", ");
      Serial.print((char)rxBuffer[i]);
    }
    Serial.println("");
    pkt_size = 0;
  }
}
