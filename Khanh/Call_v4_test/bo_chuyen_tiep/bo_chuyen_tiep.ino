#include <IRremote.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

int IRpin = 4;  // Pin nhận IR

// Tạo đối tượng BLE
BLEServer* pServer;
BLEService* pService;
BLECharacteristic* pCharacteristic;

// Class xử lý connect event
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    Serial.println("Device connected");
  }

  void onDisconnect(BLEServer* pServer) {
    Serial.println("Device disconnected");
    pServer->startAdvertising(); // Bắt đầu quảng bá lại
  }
};

// Thời gian từ lần đầu nhận được IR
unsigned long lastReceivedTime = 0;
unsigned long currentData = 0;
unsigned long lastData = 0;

void setup() {
  Serial.begin(115200);
  IrReceiver.begin(IRpin, ENABLE_LED_FEEDBACK);      // Khởi tạo bộ thu IR
  BLEDevice::init("ESP32_IR_BLE");                   // Khởi tạo BLE với tên
  pServer = BLEDevice::createServer();               // Tạo một BLE server
  pServer->setCallbacks(new MyServerCallbacks());    // Gán các callback để xử lý sự kiện kết nối/ngắt kết nối.
  pService = pServer->createService(SERVICE_UUID);   // Tạo dịch vụ BLE với UUID định trước.
  pCharacteristic = pService->createCharacteristic(  // Tạo một characteristic với UUID định trước và các thuộc tính đọc và thông báo.
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

  pService->start();
  // Quảng bá BLE
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a client connection to notify...");

  // In ra địa chỉ MAC của ESP32
  Serial.print("ESP32 MAC address: ");
  Serial.println(BLEDevice::getAddress().toString().c_str());
}

void loop() {
  // Nếu có dữ liệu IR nhận được
  if (IrReceiver.decode()) {
    currentData = IrReceiver.decodedIRData.decodedRawData;  // Lấy dữ liệu IR được giải mã
    if (currentData == lastData) {
      if (millis() - lastReceivedTime > 10000) {
        char dataStr[9];
        sprintf(dataStr, "%08X", currentData);  // Chuyển giá trị thành chuỗi hex
        pCharacteristic->setValue(dataStr);     // Đặt giá trị cho characteristic
        pCharacteristic->notify();              // Gửi thông báo qua BLE
        Serial.println("IR code sent via BLE: " + String(dataStr));
      }
    } else {
      lastReceivedTime = millis();
      lastData = currentData;
    }
    IrReceiver.resume();  // Sẵn sàng nhận dữ liệu mới
  }

  // Kiểm tra xem có dữ liệu từ bàn phím không
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    pCharacteristic->setValue(input.c_str());  // Đặt giá trị từ bàn phím cho characteristic
    pCharacteristic->notify();                 // Gửi thông báo qua BLE
    Serial.println("Data sent via BLE: " + input);
  }
}
