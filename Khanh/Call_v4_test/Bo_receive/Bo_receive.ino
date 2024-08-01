#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <DFRobotDFPlayerMini.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>

#define NUM_LEDS 120
#define DFPLAYER_TX_PIN 21
#define DFPLAYER_RX_PIN 22
#define BUTTON_PIN 15  // Pin kết nối với nút nhấn
#define SERVICE_UUID "12345678-1234-1234-1234-123456789012"
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-210987654321"

char color = 0;
Adafruit_NeoPixel strip(NUM_LEDS, 13, NEO_GRB + NEO_KHZ800);
DFRobotDFPlayerMini dfPlayer;
SoftwareSerial mySoftwareSerial(DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);

BLEClient* pClient;
BLERemoteService* pRemoteService;
BLERemoteCharacteristic* pRemoteCharacteristic;

bool isPlaying = false;  // Trạng thái phát nhạc
bool isPressed = false;  // Cờ để kiểm tra nút nhấn
unsigned long isPressedLastTime = 0;  // Thời gian lần cuối nhấn nút
const unsigned long TIME_STOP = 30000;  // 30 giây tạm dừng nhạc
volatile bool buttonPressed = false;  // Trạng thái nút nhấn trước đó
volatile bool buttonState = false;  // Trạng thái của nút nhấn
volatile unsigned long lastDebounceTime = 0;  // Thời gian lần cuối xử lý ngắt nút nhấn
const unsigned long DEBOUNCE_DELAY = 50;  // Thời gian debounce để tránh nhiều lần ngắt khi nhấn nút
bool isConnected = false;  // Trạng thái kết nối BLE

// Hàm kết nối với DFPlayer Mini
bool connectDFPlayer() {
  Serial2.begin(9600, SERIAL_8N1, DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);  
  while (!dfPlayer.begin(Serial2)) {  
    delay(1000);
    Serial.println("DFPlayer Mini OFFLINE");
  }
  Serial.println("DFPlayer Mini online");
  delay(1000);
  dfPlayer.volume(30);  // Đặt âm lượng ban đầu
  dfPlayer.setTimeOut(500);  // Đặt thời gian chờ
  dfPlayer.volume(20);  // Đặt âm lượng cho DFPlayer Mini
  dfPlayer.EQ(DFPLAYER_EQ_NORMAL);  // Đặt chế độ EQ
  dfPlayer.outputDevice(DFPLAYER_DEVICE_SD);  // Đặt thiết bị đầu ra là thẻ SD

  return true;  // Trả về trạng thái kết nối thành công
}

// Hàm phát nhạc
void playMusic() {
  dfPlayer.playFolder(2, 7);  // Chơi file nhạc từ thư mục 2, file 7
  Serial.println("Robot da den noi.");
  isPlaying = true;
}

// Hàm dừng nhạc
void stopMusic() {
  dfPlayer.stop();  // Dừng phát nhạc
  Serial.println("Stopping music...");
  isPlaying = false;  // Đặt lại trạng thái phát nhạc
  strip.fill(strip.Color(255, 165, 0), 0, NUM_LEDS);  // Đặt màu cam
  strip.show();
}

// Hàm xử lý ngắt khi nhấn nút
void IRAM_ATTR handleButtonPress() {
  unsigned long currentTime = millis();  // Lấy thời gian hiện tại

  // Kiểm tra thời gian debounce
  if ((currentTime - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (!buttonPressed) {  // Kiểm tra xem nút nhấn có thay đổi trạng thái từ không nhấn sang nhấn không
      buttonState = true;  // Đặt trạng thái của buttonState
      buttonPressed = true;  // Cập nhật trạng thái nút nhấn trước đó
      lastDebounceTime = currentTime;  // Cập nhật thời gian debounce
    }
  }
}

// Callbacks cho BLE Client
class MyClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient* pClient) {
    Serial.println("Connected to server");
    isConnected = true;  // Cập nhật trạng thái kết nối BLE
  }

  void onDisconnect(BLEClient* pClient) {
    Serial.println("Disconnected from server");
    isConnected = false;  // Cập nhật trạng thái mất kết nối BLE
  }
};

// Callbacks cho thiết bị BLE được quảng bá
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // Kiểm tra xem thiết bị có tên là "ESP32_IR_BLE" không
    if (advertisedDevice.haveName() && advertisedDevice.getName() == "ESP32_IR_BLE") {
      BLEAddress advertisedAddress = advertisedDevice.getAddress();  // Lấy địa chỉ MAC của thiết bị
      Serial.print("Found device: ");
      Serial.println(advertisedAddress.toString().c_str());  // In địa chỉ MAC của thiết bị ra Serial Monitor
      pClient->connect(advertisedAddress);  // Kết nối với thiết bị sử dụng địa chỉ MAC

      // Chờ đến khi kết nối thành công
      while (!pClient->isConnected()) {
        delay(100);
      }

      pRemoteService = pClient->getService(BLEUUID(SERVICE_UUID));  // Lấy dịch vụ từ thiết bị

      if (pRemoteService != nullptr) {
        pRemoteCharacteristic = pRemoteService->getCharacteristic(BLEUUID(CHARACTERISTIC_UUID));  // Lấy characteristic từ dịch vụ

        if (pRemoteCharacteristic != nullptr) {
          pRemoteCharacteristic->registerForNotify(notifiedCallback);  // Đăng ký nhận thông báo từ characteristic
          Serial.println("Registered for notifications");
        } else {
          Serial.println("Failed to find characteristic");
        }
      } else {
        Serial.println("Failed to find service");
      }
    }
  }
};

// Callback khi nhận được thông báo từ characteristic BLE
void notifiedCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  String receivedData = "";
  for (int i = 0; i < length; i++) {
    receivedData += (char)pData[i];  // Chuyển dữ liệu nhận được thành chuỗi
  }
  Serial.println("Received BLE notification: " + receivedData);

  if (receivedData == "1AE54DB2") {  // Kiểm tra nếu dữ liệu nhận được là mã IR cụ thể
    playMusic();  // Phát nhạc
    strip.fill(strip.Color(0, 255, 0), 0, NUM_LEDS);  // Đặt màu xanh
    strip.show();
  } else {
    // Xử lý các dữ liệu khác nhận được từ bộ chuyển tiếp
    Serial.println("Received data: " + receivedData);
    strip.fill(strip.Color(255, 0, 0), 0, NUM_LEDS);  // Đặt màu đỏ
    strip.show();
  }
}

// Hàm khởi tạo BLE Client
void initBLEClient() {
  BLEDevice::init("ESP32_BLE_Client");
  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallbacks());

  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true);
  pBLEScan->setActiveScan(true);  // Kích hoạt chế độ quét chủ động
  pBLEScan->setInterval(100);  // Đặt khoảng thời gian quét
  pBLEScan->setWindow(99);  // Đặt cửa sổ quét
}

// Hàm xử lý sự kiện nút nhấn
void handleButtonEvent() {
  if (buttonState) {
    Serial.println("Button is pressed");
    if (isPlaying) {
      isPressed = true;
      isPressedLastTime = millis();  // Lưu thời gian nhấn nút
      stopMusic();  // Dừng nhạc khi nhấn nút
    }
    buttonState = false;  // Đặt lại trạng thái buttonState sau khi xử lý
  }

  // Kiểm tra nếu thời gian từ lần nhấn nút cuối cùng vượt quá 30 giây thì tắt cờ isPressed
  if ((millis() - isPressedLastTime) > TIME_STOP) {
    isPressed = false;
    strip.fill(strip.Color(255, 0, 0), 0, NUM_LEDS);  // Đặt màu đỏ
    strip.show();
  }

  // Kiểm tra nếu nhạc đã phát đủ 7 giây hoặc nút nhấn được nhấn thì dừng nhạc
  if (isPressed || (isPlaying && (millis() - isPressedLastTime > 7000))) {
    stopMusic();
  }
}

// Hàm kiểm tra và khôi phục kết nối BLE
void checkBLEConnection() {
  if (!isConnected) {
    Serial.println("Reconnecting BLE...");
    initBLEClient();  // Khởi tạo lại BLE Client
  }
}

//create color by value color
void colorWipe(uint32_t color, int wait) {
  for (int i = 0; i < 120; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
  delay(wait);
}

void setup() {
  Serial.begin(115200);
  strip.begin();
  strip.show();  // Initialize all pixels to 'off'
  
  bool connectDFPlayerStatus = connectDFPlayer();  // Gọi hàm kết nối DFPlayer Mini

  if (connectDFPlayerStatus) {
    Serial.println("DFPlayer Mini connected successfully.");
  } else {
    Serial.println("Failed to connect DFPlayer Mini.");
  }

  initBLEClient();  // Khởi tạo BLE Client

  pinMode(BUTTON_PIN, INPUT);  // Thiết lập chân nút nhấn là INPUT
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, FALLING);  // Gắn hàm ngắt cho nút nhấn
}

void loop() {
  BLEDevice::getScan()->start(1, false);  // Quét BLE trong 1 giây
  handleButtonEvent();  // Gọi hàm xử lý sự kiện nút nhấn
  checkBLEConnection();  // Kiểm tra và khôi phục kết nối BLE nếu cần
}
