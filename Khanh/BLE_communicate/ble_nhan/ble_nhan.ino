#include "BLEDevice.h"

// Định nghĩa UUID cho dịch vụ và đặc tính mà chúng ta muốn kết nối
static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
static BLEUUID charUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");

static boolean doConnect = false;  // Cờ để xác định xem có nên kết nối không
static boolean connected = false;  // Cờ để xác định xem đã kết nối chưa
static BLERemoteCharacteristic* pRemoteCharacteristic;  // Con trỏ đến đặc tính từ xa
static BLEAdvertisedDevice* myDevice;  // Con trỏ đến thiết bị quảng cáo mà chúng ta muốn kết nối
static BLEScan* pBLEScan;  // Con trỏ đến đối tượng quét BLE

// Hàm callback khi nhận được thông báo từ đặc tính từ xa
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
  Serial.print("Notify callback for characteristic ");
  Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
  Serial.print(" of data length ");
  Serial.println(length);
  Serial.print("data: ");
  int receivedValue;
  memcpy(&receivedValue, pData, sizeof(receivedValue)); // Copy byte array to int
  Serial.println(receivedValue); // Print received int value
}

// Lớp callback cho các sự kiện của BLE client
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    connected = true;
    Serial.println("Connected to BLE Server");
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("Disconnected from BLE Server, attempting to reconnect...");
    doConnect = true;  // Cố gắng kết nối lại
  }
};

// Hàm kết nối đến server
bool connectToServer() {
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  BLEClient* pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Kết nối đến BLE Server từ xa
  if (!pClient->connect(myDevice)) {
    Serial.println(" - Failed to connect to server");
    return false;
  }
  Serial.println(" - Connected to server");
  pClient->setMTU(517);

  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");

  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our characteristic");

  if (pRemoteCharacteristic->canRead()) {
    String value = pRemoteCharacteristic->readValue().c_str();
    Serial.print("The characteristic value was: ");
    Serial.println(value);
  }

  if (pRemoteCharacteristic->canNotify())
    pRemoteCharacteristic->registerForNotify(notifyCallback);

  connected = true;
  return true;
}

// Lớp callback khi tìm thấy một thiết bị quảng cáo
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // Kiểm tra nếu thiết bị quảng cáo có UUID dịch vụ mà chúng ta đang tìm kiếm
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      // Kiểm tra nếu tên thiết bị quảng cáo trùng khớp
      if (advertisedDevice.getName() == "Long name works now") {
        pBLEScan->stop();
        myDevice = new BLEAdvertisedDevice(advertisedDevice);
        doConnect = true;
      }
    }
  }
};

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(1, false);  // Bắt đầu quét không giới hạn thời gian
}

void loop() {
  if (doConnect) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("Failed to connect to the server; retrying...");
    }
    doConnect = false;
  }

  if (!connected) {
    pBLEScan->start(1);  // Liên tục quét để tìm thiết bị
  }
  delay(1000); // Delay để tránh quét quá nhanh
}
