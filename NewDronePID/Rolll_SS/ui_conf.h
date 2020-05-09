#ifndef UI_CONF
#define UI_CONF

#define BLYNK_PRINT Serial


#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "eYcrTSn-5ZZDlGWLoPxtSGaHWEAMGeNb";

// Your WiFi credentials.
// Set password to "" for open networks.
//char ssid[] = "Cablevision127";
//char pass[] = "Contra_Word.";
//char ssid[] = "IZZI28681";
//char pass[] = "37662DCDAF5B0E8E";
//char ssid[] = "IZZI28681";
//char pass[] = "37662DCDAF5B0E8E";
char ssid[] = "INFINITUM0C3E71";
char pass[] = "06653252FF";
//char ssid[] = "iPhone de Enrique";
//char pass[] = "danielar";


// Global variables related to user interface
uint8_t ui_callback = 0;
/*uint8_t eStop = 1;
double throttle = 1000;
double kp = 0;
double ki = 0;
double kd = 0;*/

BLYNK_WRITE(V0)
{
  eStop = param.asInt(); // assigning incoming value from pin V1 to a variable
  ui_callback = 1;
}

BLYNK_WRITE(V1)
{
  throttle = constrain(param.asInt(), 1000, 1800); // assigning incoming value from pin V1 to a variable
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

#endif
