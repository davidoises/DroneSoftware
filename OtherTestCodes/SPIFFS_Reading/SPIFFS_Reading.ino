#include "SPIFFS.h"
 
void setup() {
 
   Serial.begin(500000);
   delay(500);
   Serial.println("\n");
 
   if(!SPIFFS.begin(true)){
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
   }
 
    //--------- Write to file
    File fileToWrite = SPIFFS.open("/test.txt", FILE_WRITE);
 
    if(!fileToWrite){
        Serial.println("There was an error opening the file for writing");
        return;
    }
 
    if(fileToWrite.println("ORIGINAL LINE")){
        Serial.println("File was written");;
    } else {
        Serial.println("File write failed");
    }
 
    fileToWrite.close();
 
    //--------- Apend content to file
    File fileToAppend = SPIFFS.open("/test.txt", FILE_APPEND);
 
    if(!fileToAppend){
        Serial.println("There was an error opening the file for appending");
        return;
    }
 
    if(fileToAppend.println("APPENDED LINE")){
        Serial.println("File content was appended");
    } else {
        Serial.println("File append failed");
    }
 
    fileToAppend.close();
 
    //---------- Read file
    File fileToRead = SPIFFS.open("/test.txt");
 
    if(!fileToRead){
        Serial.println("Failed to open file for reading");
        return;
    }
 
    Serial.println("File Content:");
 
    while(fileToRead.available()){
 
        Serial.write(fileToRead.read());
    }
 
    fileToRead.close();
}
 
void loop() {}
