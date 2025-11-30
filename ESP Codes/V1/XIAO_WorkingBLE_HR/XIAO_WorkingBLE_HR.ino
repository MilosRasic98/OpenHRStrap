#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Standard Heart Rate Service / Characteristic UUIDs
#define HEART_RATE_SERVICE_UUID        "180D"
#define HEART_RATE_MEASUREMENT_UUID    "2A37"

BLEServer* pServer = nullptr;
BLECharacteristic* pHeartRateChar = nullptr;
bool deviceConnected = false;
uint8_t bpm = 50;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected, restarting advertising...");
    pServer->getAdvertising()->start();
  }
};

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Starting BLE Heart Rate Monitor...");

  BLEDevice::init("ESP32 HRM");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create Heart Rate Service
  BLEService* hrService = pServer->createService(HEART_RATE_SERVICE_UUID);

  // Heart Rate Measurement Characteristic (Notify only)
  pHeartRateChar = hrService->createCharacteristic(
      HEART_RATE_MEASUREMENT_UUID,
      BLECharacteristic::PROPERTY_NOTIFY
  );

  // Descriptor required for notifications
  pHeartRateChar->addDescriptor(new BLE2902());

  hrService->start();

  // Advertising
  BLEAdvertising* pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(HEART_RATE_SERVICE_UUID);
  pAdvertising->setAppearance(832); // Heart Rate Sensor
  pAdvertising->start();

  Serial.println("BLE HRM ready, advertising...");
}

void loop() {
  if (deviceConnected) {
    // Heart Rate Measurement packet (Flags + HR value)
    uint8_t hrmPacket[2];
    hrmPacket[0] = 0x00; // Flags: 8-bit HR value
    hrmPacket[1] = bpm;  // Fake heart rate

    pHeartRateChar->setValue(hrmPacket, 2);
    pHeartRateChar->notify();

    Serial.printf("Sent HR: %d bpm\n", bpm);

    // Sawtooth pattern 50 → 60 → 50
    bpm++;
    if (bpm > 60) bpm = 50;

    delay(1000); // Send every second
  }
}
