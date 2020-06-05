#define BLYNK_PRINT Serial

#define BLYNK_USE_DIRECT_CONNECT
#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>

//Blynk setup
char auth[] = "eYcrTSn-5ZZDlGWLoPxtSGaHWEAMGeNb";
//char auth[] = "fV5UwNfaq5iEsARKMJmhurZP8aXybvJ0";

uint8_t eStop = 1;
uint8_t ui_callback = 0;

int counter = 0;

BLYNK_WRITE(V0) {  //Plain called function
  eStop = param.asInt();
  ui_callback = 1;
}


void blynkLoop(void *pvParameters ) {  //task to be created by FreeRTOS and pinned to core 0
  while (true) {
    Blynk.run();
    vTaskDelay(50);
    counter++;
  }
}

void setup() {
  Serial.begin(115200);

  Blynk.begin(auth);

  //this is where we start the Blynk.run() loop pinned to core 0, given priority "1" (which gives it thread priority over "0")
  xTaskCreatePinnedToCore(
    blynkLoop,      /* Function to implement the task */
    "blynk core 0", /* Name of the task */
    100000,         /* Stack size in words */
    NULL,           /* Task input parameter */
    1,              /* Priority of the task */
    NULL,           /* Task handle. */
    0);             /* Core where the task should run */

  Serial.println(" Blynk loop running");
}

void loop() {
  /*if(ui_callback == 1)
  {
    Serial.println(eStop);
    ui_callback = 0;
  }*/
  Serial.println(counter);
  delay(5);
}
