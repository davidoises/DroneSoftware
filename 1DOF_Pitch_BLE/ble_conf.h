#ifndef MATH3D_LIB
#define MATH3D_LIB

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <sstream>

#define SERVICE_UUID        "5b06735f-c5dc-47c8-b4aa-bc0a980443bd"
#define P_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define I_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define D_UUID "beb5483e-36e1-4688-b7f5-ea07361b26aa"

#define BLE_DEBUG 0

double ble_kp;
double ble_ki;
double ble_kd;
double ble_triggered = 0;

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string uuid = pCharacteristic->getUUID().toString();
      std::string value = pCharacteristic->getValue();
      
      if(uuid == P_UUID)
      { 
        ble_kp = ::atof(value.c_str());
        pCharacteristic->setValue("kp = " + value);
#if BLE_DEBUG
        Serial.print("kp = ");
        Serial.println(ble_kp, 4);
#endif
      }
      else if(uuid == I_UUID)
      {
        ble_ki = ::atof(value.c_str());
        pCharacteristic->setValue("ki = " + value);
#if BLE_DEBUG
        Serial.print("ki = ");
        Serial.println(ble_ki, 4);
#endif
      }
      else if(uuid == D_UUID)
      {
        ble_kd = ::atof(value.c_str());
        pCharacteristic->setValue("kd = " + value);
#if BLE_DEBUG
        Serial.print("kd = ");
        Serial.println(ble_kd, 4);
#endif
      }
      else
      {
        return;
      }
      ble_triggered = 1;
    }
};

std::string to_string(double number)
{
  std::ostringstream os;
  os << number;
  return os.str();
}

void ble_setup(double p, double i, double d)
{
  ble_kp = p;
  ble_ki = i;
  ble_kd = d;
  
  // Creation of the BLE server and service
  BLEDevice::init("PIDSettings");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Characteristic for proportional gain
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         P_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->setValue("kp = " + to_string(ble_kp));

  // characteristic for integral gain
  BLECharacteristic *iCharacteristic = pService->createCharacteristic(
                                         I_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  iCharacteristic->setCallbacks(new MyCallbacks());
  iCharacteristic->setValue("ki = " + to_string(ble_ki));

  // characteristic for derivatice gain
  BLECharacteristic *dCharacteristic = pService->createCharacteristic(
                                         D_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  dCharacteristic->setCallbacks(new MyCallbacks());
  dCharacteristic->setValue("kd = " + to_string(ble_kd));

  // Start all BLE services
  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}

#endif
