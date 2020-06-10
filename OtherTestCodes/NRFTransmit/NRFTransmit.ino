#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

RF24 radio(4, 5, 18, 19, 23); // CE, CSN, SCK, MISO, MOSI

const byte address[6] = "00001";
String switch_status = "KK";

void setup() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  char text[4];
  switch_status.toCharArray(text, 4);
  radio.write(&text, sizeof(text));
  delay(200);
}
