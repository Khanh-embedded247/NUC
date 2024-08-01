#include <Arduino.h>
// Thư viện Bluetooth
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <DFRobotDFPlayerMini.h>
// Thư viện DFPlayer
#include <SoftwareSerial.h>

// Khai báo các chân
#define Button 15
#define potPin 4
#define LED_1 16
#define LED_2 17
#define LED_3 5
#define LED_4 18
#define LED_5 19
#define LED_6 32
#define DFPLAYER_TX_PIN 21
#define DFPLAYER_RX_PIN 22

// Khởi tạo object Bluetooth
BLEScan* pBLEScan;
// Khởi tạo object DFPlayer
DFRobotDFPlayerMini myDFPlayer;
SoftwareSerial mySoftwareSerial(DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);

// Các biến toàn cục
int rssi_resistor;
bool rssi_check = false;
bool isPlaying = false;
int rssi = 0;  // Biến toàn cục để lưu giá trị RSSI
bool buttonPressed = false;
unsigned long lastPlayTime = 0;
unsigned long buttonPressTime = 0;
unsigned long lastPrintTime = 0;
const unsigned long debounceDelay = 100;   // Thời gian trễ chống rung
const unsigned long stopDuration = 90000;  // Thời gian dừng âm thanh trong 60 giây
const unsigned long printInterval = 1000;  // In thiết bị và RSSI mỗi giây

// Lớp xử lý kết quả thiết bị Bluetooth
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.haveName() && advertisedDevice.getName() == "Robot") {
      rssi = advertisedDevice.getRSSI();
      Serial.printf("Device found with RSSI: %d\n", rssi);
      if (rssi > rssi_resistor) {
        rssi_check = true;
      } else {
        rssi_check = false;
      }
      if (rssi == 0) {
        rssi_check = false;
      }
    }
  }
};

// Cập nhật trạng thái của các đèn LED dựa trên giá trị biến trở
void updateLEDs(int potValue) {
  // digitalWrite(LED_1, potValue >= 20 ? HIGH : LOW);
  // digitalWrite(LED_2, potValue >= 40 ? HIGH : LOW);
  // digitalWrite(LED_3, potValue >= 60 ? HIGH : LOW);
  // digitalWrite(LED_4, potValue >= 80 ? HIGH : LOW);
  // digitalWrite(LED_5, potValue == 100 ? HIGH : LOW);
  digitalWrite(LED_1, potValue < 100 ? HIGH : LOW);
  digitalWrite(LED_2, potValue <= 80 ? HIGH : LOW);
  digitalWrite(LED_3, potValue <= 60 ? HIGH : LOW);
  digitalWrite(LED_4, potValue <= 40 ? HIGH : LOW);
  digitalWrite(LED_5, 20 > potValue > 0 ? HIGH : LOW);
}

// Xử lý sự kiện nút nhấn
void IRAM_ATTR handleButtonPress() {
  unsigned long currentTime = millis();
  if (currentTime - buttonPressTime > debounceDelay) {
    buttonPressed = true;
    buttonPressTime = currentTime;
  }
}

// Cấu hình hệ thống
void setup() {
  Serial.begin(115200);

  pinMode(LED_6, OUTPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(LED_4, OUTPUT);
  pinMode(LED_5, OUTPUT);

  mySoftwareSerial.begin(9600);
  while (!myDFPlayer.begin(mySoftwareSerial)) {
    Serial.println("DFPlayer Mini not detected. Retrying...");
    delay(1000);
  }
  Serial.println("DFPlayer Mini connected.");
  myDFPlayer.setTimeOut(500);
  myDFPlayer.volume(20);
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
  //Khởi tạo BLE
  BLEDevice::init("ESP Caller Y4");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true);
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  pinMode(Button, INPUT_PULLUP);                                               // Cấu hình nút nhấn
  attachInterrupt(digitalPinToInterrupt(Button), handleButtonPress, FALLING);  // Gắn ngắt ngoài cho nút nhấn
}

// Phát nhạc
void playMusic() {
  myDFPlayer.playFolder(2, 7);  // Chơi file nhạc a.mp3
  isPlaying = true;
  Serial.println("Playing music...");
}

// Dừng nhạc
void stopMusic() {
  myDFPlayer.stop();
  isPlaying = false;
  Serial.println("Stopping music...");
}

// Kiểm tra và xử lý trạng thái nút nhấn
void Button_press() {
  if (buttonPressed&&isPlaying) {
    Serial.println("Button released, stopping music for 60 seconds...");
    stopMusic();

    unsigned long stopStartTime = millis();
    while (millis() - stopStartTime < stopDuration) {
      if (digitalRead(Button) == LOW) {
        handleButtonPress();
        stopStartTime = millis();  // Reset thời gian dừng nếu nút nhấn lại
      }
      int potPercentage = res_value();
      updateRSSIThreshold(potPercentage);
      updateLEDs(potPercentage);  // Cập nhật trạng thái LED ngay lập tức
      delay(10);  // Giảm tải CPU
    }
    
    buttonPressed = false;  // Reset trạng thái nút nhấn
                            // if (rssi_check) {  // Nếu điều kiện RSSI vẫn thỏa mãn, tiếp tục phát âm thanh
                            //   playMusic();
                            //   lastPlayTime = millis();
                            // }
  }
}

// Đọc giá trị biến trở và trả về phần trăm
int res_value() {
  int potValue = analogRead(potPin);
  int potPercentage = map(potValue, 0, 4095, 0, 100);
  Serial.printf("Potentiometer value: %d\n", potPercentage);
  return potPercentage;
}

// Cập nhật ngưỡng RSSI dựa trên giá trị biến trở
void updateRSSIThreshold(int potPercentage) {
  rssi_resistor = map(potPercentage, 0, 100, -100, -40);
}

// In giá trị RSSI và ngưỡng RSSI mỗi giây
void printRSSI() {
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= printInterval) {
    lastPrintTime = currentTime;
    Serial.printf("Device found with RSSI: %d, RSSI threshold: %d\n", rssi, rssi_resistor);
  }
}

// Vòng lặp chính
void loop() {
  int potPercentage = res_value();
  updateRSSIThreshold(potPercentage);
  updateLEDs(potPercentage);
Button_press();
 pBLEScan->start(1, false);  // Quét BLE trong 0.1 giây
  if (rssi_check && !isPlaying) {
    playMusic();
    isPlaying = true;
    lastPlayTime = millis();

  } else {
    if (millis() - lastPlayTime >= 7000) {
      stopMusic();
      rssi_check = false;
    }
  }

  // if (isPlaying && (millis() - lastPlayTime > 7000)) {
  //   playMusic();
  //   lastPlayTime = millis();
  // }

 
  
  printRSSI();
}
