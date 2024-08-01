#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID        "12345678-1234-1234-1234-123456789012"
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-210987654321"

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) {
    Serial.println("Device connected");
    // In ra thông tin của thiết bị kết nối
    Serial.printf("Connected device MAC: %s\n", pServer->getConnId());
  }

  void onDisconnect(BLEServer* pServer) {
    Serial.println("Device disconnected");
  }
};

void setup() {
  Serial.begin(115200);

  // Khởi tạo BLE
  BLEDevice::init("Robot");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
//tạo một dịch vụ với UUID 
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Tạo đặc tính
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );

  pCharacteristic->setValue("Hello World");
  pService->start();

  // Quảng bá BLE
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a client connection to notify...");
}

void loop() {
  // Đặt nội dung mã lệnh chính tại đây
}
