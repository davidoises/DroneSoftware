#define TRX 0
#define SEND 0

#if SEND && !TRX
#define RXD2 15//16
#define TXD2 9// 17
#else
#define RXD2 9//16
#define TXD2 15// 17
#endif

void setup() {
  // put your setup code here, to run once:
  Serial.begin(500000);
  Serial.println("test");
  delay(1000);
  //Serial2.begin(115200);
  Serial2.begin(500000, SERIAL_8N1, RXD2, TXD2);
}

void loop() {
#if SEND || TRX
  Serial2.println("TEst");
#endif
#if TRX || !SEND
  while(Serial2.available())
  {
    Serial.print(char(Serial2.read()));
  }
#endif
}
