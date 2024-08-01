#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <DFRobotDFPlayerMini.h>
#include <SoftwareSerial.h>

#define DFPLAYER_TX_PIN 21
#define DFPLAYER_RX_PIN 22
// Khai báo các chân
const int buttonPin = 15;
#define analogPin 4
#define LED_1 19
#define LED_2 18
#define LED_3 5
#define LED_4 17
#define LED_5 16
#define LED_6 32
#define RSSI_xa -95   // bộ gọi xa robot nhất
#define RSSI_gan -35  // bộ gọi gần robot nhất
#define TIME1 30000   // 30s thời gian ngừng kêu khi nhấn nút

DFRobotDFPlayerMini myDFPlayer;
SoftwareSerial mySoftwareSerial(DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);
BLEScan* pBLEScan;
bool isPlaying = false;
bool rssi_check = false;
unsigned long lastPlayTime = 0;
int rssi = 0;
int analog = 0;
bool ble_connect = false;

unsigned long rssiStartTime = 0;
const unsigned long rssiCheckDuration = 10000;  // 10 seconds

void playMusic() {
  myDFPlayer.playFolder(2, 7);  // Chơi file nhạc a.mp3
  Serial.println("Playing music...");
}

void stopMusic() {
  myDFPlayer.stop();
  Serial.println("Stopping music...");
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.haveName() && advertisedDevice.getName() == "Robot") {
      rssi = advertisedDevice.getRSSI();
      ble_connect = true;
    }
  }
};

//------------------- xu ly button ------------------

// Biến để lưu trạng thái của nút nhấn
volatile bool buttonState = false;
bool button_v2 = false;
unsigned long button_v2_trc;
const unsigned long button_v2_delta = TIME1;

// Biến để lưu trữ thời gian ngắt gần nhất
volatile unsigned long lastDebounceTime = 0;

// Thời gian debounce (ms)
const unsigned long debounceDelay = 50;

void IRAM_ATTR handleButtonPress() {
  // Lấy thời gian hiện tại
  unsigned long currentTime = millis();

  // Kiểm tra nếu thời gian hiện tại trừ đi thời gian ngắt gần nhất lớn hơn debounceDelay
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    // Đổi trạng thái của buttonState khi nút nhấn được nhấn
    buttonState = !buttonState;
    // Cập nhật thời gian ngắt gần nhất
    lastDebounceTime = currentTime;
    // In trạng thái của nút nhấn ra màn hình
    if (buttonState) {
      Serial.println("Button is pressed");
    }

    if (isPlaying) {
      button_v2 = true;
      button_v2_trc = millis();
    }
  }
}

// Đọc giá trị biến trở và trả về phần trăm
int analog_read() {
  int potValue = analogRead(analogPin);
  int potPercentage = map(potValue, 0, 4095, RSSI_xa, RSSI_gan);
  return potPercentage;
}

void controlLEDs(bool l1, bool l2, bool l3, bool l4, bool l5) {
  digitalWrite(LED_5, l1);
  digitalWrite(LED_4, l2);
  digitalWrite(LED_3, l3);
  digitalWrite(LED_2, l4);
  digitalWrite(LED_1, l5);
}

void controlLEDs_v2(int muc) {

  switch (muc) {
    case 0:
      controlLEDs(1, 1, 1, 1, 1);
      delay(50);
      controlLEDs(0, 0, 0, 0, 0);
      delay(50);
      controlLEDs(1, 1, 1, 1, 1);
      delay(50);
      controlLEDs(0, 0, 0, 0, 0);
      delay(100);
      break;

    case 1:
      controlLEDs(1, 1, 1, 1, 1);
      delay(50);
      controlLEDs(1, 0, 0, 0, 0);
      delay(50);
      controlLEDs(1, 1, 1, 1, 1);
      delay(50);
      controlLEDs(1, 0, 0, 0, 0);
      delay(100);
      break;

    case 2:
      controlLEDs(1, 1, 1, 1, 1);
      delay(50);
      controlLEDs(1, 1, 0, 0, 0);
      delay(50);
      controlLEDs(1, 1, 1, 1, 1);
      delay(50);
      controlLEDs(1, 1, 0, 0, 0);
      delay(100);
      break;

    case 3:
      controlLEDs(1, 1, 1, 1, 1);
      delay(50);
      controlLEDs(1, 1, 1, 0, 0);
      delay(50);
      controlLEDs(1, 1, 1, 1, 1);
      delay(50);
      controlLEDs(1, 1, 1, 0, 0);
      delay(100);
      break;

    case 4:
      controlLEDs(1, 1, 1, 1, 1);
      delay(50);
      controlLEDs(1, 1, 1, 1, 0);
      delay(50);
      controlLEDs(1, 1, 1, 1, 1);
      delay(50);
      controlLEDs(1, 1, 1, 1, 0);
      delay(100);
      break;

    case 5:
      controlLEDs(1, 1, 1, 1, 1);
      break;

    default:
      controlLEDs(0, 0, 0, 0, 0);
  }
}

void setup() {
  Serial.begin(115200);
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

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true);
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  // Thiết lập chân nút nhấn là INPUT_PULLUP để dùng ngắt
  pinMode(buttonPin, INPUT);

  // Gắn hàm ngắt cho nút nhấn
  attachInterrupt(digitalPinToInterrupt(buttonPin), handleButtonPress, FALLING);

  pinMode(LED_6, OUTPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(LED_4, OUTPUT);
  pinMode(LED_5, OUTPUT);
}

void testLed() {
  controlLEDs(0, 0, 1, 1, 1);  // ana - rssi < -20
  delay(1000);

  controlLEDs(0, 0, 1, 1, 0);  // -5 < ana - rssi <= -20
  delay(1000);

  controlLEDs(0, 0, 1, 0, 0);  // 5 <= ana - rssi <= -5
  delay(1000);

  controlLEDs(0, 1, 1, 0, 0);  // 5 < ana - rssi <= 20
  delay(1000);

  controlLEDs(1, 1, 1, 0, 0);  // ana - rssi > 20
  delay(1000);
}

void loop() {
  // testLed();

  pBLEScan->start(1, false);  // Quét BLE trong 0.1 giây
  int ana = analog_read();

  if (ble_connect == true) {
    Serial.printf("ana: %d  -  rssi: %d  = %d ", ana, rssi, (ana - rssi));
    Serial.println();
    int delta = -(ana - rssi);
    if (delta <= 0) controlLEDs_v2(0);
    if (delta > 0 && delta <= 10) controlLEDs_v2(1);
    if (delta > 10 && delta <= 20) controlLEDs_v2(2);
    if (delta > 20 && delta <= 30) controlLEDs_v2(3);
    if (delta > 30 && delta <= 40) controlLEDs_v2(4);
    if (delta > 40) controlLEDs_v2(5);

    if (rssi > ana) {
      if (rssiStartTime == 0) {
        rssiStartTime = millis();  // Start the timer
      } else if (millis() - rssiStartTime >= rssiCheckDuration) {
        rssi_check = true;  // Set rssi_check to true if the condition holds for 5 seconds
      }
    } else {
      rssi_check = false;
      rssiStartTime = 0;  // Reset the timer if the condition is not met
    }
    Serial.printf("rssi_check: %d\n", rssi_check);

    if (rssi_check) {
      if (!isPlaying) {
        playMusic();
        isPlaying = true;
        lastPlayTime = millis();  // Ghi lại thời gian bắt đầu phát nhạc
      }
    }

    // sau 30s , chuyển button_s2 --> false
    Serial.print("button_v2: ");
    Serial.println(button_v2);

    if ((millis() - button_v2_trc) > button_v2_delta) {
      button_v2 = false;
    }

    // Kiểm tra nếu nhạc đã chơi đủ 7 giây thì dừng lại
    if (button_v2 || (isPlaying && (millis() - lastPlayTime > 7000))) {
      stopMusic();
      Serial.println("3");
      isPlaying = false;
    }

  } else {
    controlLEDs(0, 0, 0, 0, 0);
  }

  ble_connect = false;
}
