/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial


#define BLYNK_USE_DIRECT_CONNECT
#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "eYcrTSn-5ZZDlGWLoPxtSGaHWEAMGeNb";

// Your WiFi credentials.
// Set password to "" for open networks.
//char ssid[] = "Cablevision127";
//char pass[] = "Contra_Word.";
//char ssid[] = "IZZI28681";
//char pass[] = "37662DCDAF5B0E8E";

// Global variables related to user interface
uint8_t ui_callback = 0;
uint8_t eStop = 1;
int throttle = 1000;
double kp = 0;
double ki = 0;
double kd = 0;

BLYNK_WRITE(V0)
{
  eStop = param.asInt(); // assigning incoming value from pin V1 to a variable
  ui_callback = 1;
}

BLYNK_WRITE(V1)
{
  throttle = param.asInt(); // assigning incoming value from pin V1 to a variable
  ui_callback = 1;
}

BLYNK_WRITE(V2)
{
  kp = param.asDouble(); // assigning incoming value from pin V1 to a variable
  ui_callback = 1;
}

BLYNK_WRITE(V3)
{
  ki = param.asDouble(); // assigning incoming value from pin V1 to a variable
  ui_callback = 1;
}

BLYNK_WRITE(V4)
{
  kd = param.asDouble(); // assigning incoming value from pin V1 to a variable
  ui_callback = 1;
}

BLYNK_CONNECTED()
{
  Blynk.virtualWrite(V0, eStop);
  Blynk.virtualWrite(V1, throttle);
  Blynk.virtualWrite(V2, kp);
  Blynk.virtualWrite(V3, ki);
  Blynk.virtualWrite(V4, kd);
}

void setup()
{
  // Debug console
  Serial.begin(115200);

  Blynk.begin(auth);
}

void loop()
{
  Blynk.run();
  if(ui_callback)
  {
    Serial.print("eStop= ");
    Serial.print(eStop);
    Serial.print("; throttle= ");
    Serial.print(throttle);
    Serial.print("; kp= ");
    Serial.print(kp);
    Serial.print("; ki= ");
    Serial.print(ki);
    Serial.print("; kd= ");
    Serial.println(kd);
    ui_callback = 0;  
  }
}
