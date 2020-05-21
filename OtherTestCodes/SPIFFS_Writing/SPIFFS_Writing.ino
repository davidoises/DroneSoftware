#include "SPIFFS.h"

File file;
 
void setup() {
 
  Serial.begin(500000);
 
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
 
  file = SPIFFS.open("/test.txt", FILE_WRITE);
 
  if (!file) {
    Serial.println("There was an error opening the file for writing");
    return;
  }
  if (file.println("Puto el que lo lea")) {
    Serial.println("File was written");
  } else {
    Serial.println("File write failed");
  }

  if (file.print(1.5)) {
    Serial.println("File was written");
  } else {
    Serial.println("File write failed");
  }
 
  file.close();
}
 
void loop() {}
