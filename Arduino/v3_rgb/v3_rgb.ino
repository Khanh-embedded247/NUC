#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <DFRobotDFPlayerMini.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>

// Định nghĩa các hằng số và chân
#define DFPLAYER_TX_PIN 21
#define DFPLAYER_RX_PIN 22
#define ANALOG_PIN 4
#define NUM_LEDS 10
#define RSSI_XA -95
#define RSSI_GAN -35
#define TIME1 30000
#define DEBOUNCE_DELAY 50
#define RSSI_CHECK_DURATION 10000

class Call_Notification {
public:
  Call_Notification(int buttonPin, int ledPin);
  void setup();
  void loop();

private:
  void playMusic();                                                               // Phương thức điều khiển phát nhạc
  void stopMusic();                                                               // Phương thức điều khiển dừng nhạc
  static void handleButtonPressISR();                                             // Hàm ngắt tĩnh để xử lý nút nhấn
  void handleButtonPress();                                                       // Phương thức xử lý nút nhấn
  int readAnalog();                                                               // Đọc giá trị analog từ biến trở
  void setLEDsBasedOnRSSI(int delta);                                             // Thiết lập dải đèn LED dựa trên giá trị RSSI
  static void onBLEAdvertisedDeviceResult(BLEAdvertisedDevice advertisedDevice);  // Hàm tĩnh xử lý thiết bị BLE quảng cáo

  // Các thuộc tính của lớp
  int buttonPin_;
  int ledPin_;
  DFRobotDFPlayerMini dfPlayer_;
  SoftwareSerial softwareSerial_;
  Adafruit_NeoPixel strip_;
  BLEScan* pBLEScan_;

  bool isPlaying_;
  bool rssiCheck_;
  unsigned long lastPlayTime_;
  int rssi_;
  int analog_;
  bool bleConnect_;
  unsigned long rssiStartTime_;
  volatile bool buttonState_;
  bool buttonV2_;
  unsigned long buttonV2LastPress_;
  volatile unsigned long lastDebounceTime_;
  static Call_Notification* instance_;
};
//Biến tĩnh để tham chiếu đến bất kỳ đối tượng nào của lớp.
Call_Notification* Call_Notification::instance_ = nullptr;

// Hàm khởi tạo, khởi tạo các thuộc tính với các giá trị ban đầu
Call_Notification::Call_Notification(int buttonPin, int ledPin)
  : buttonPin_(buttonPin),
    ledPin_(ledPin),
    softwareSerial_(DFPLAYER_RX_PIN, DFPLAYER_TX_PIN),
    strip_(NUM_LEDS, ledPin, NEO_GRB + NEO_KHZ800),
    isPlaying_(false),
    rssiCheck_(false),
    lastPlayTime_(0),
    rssi_(0),
    analog_(0),
    bleConnect_(false),
    rssiStartTime_(0),
    buttonState_(false),
    buttonV2_(false),
    buttonV2LastPress_(0),
    lastDebounceTime_(0) {
  instance_ = this;
}

// Phương thức cài đặt hệ thống
void Call_Notification::setup() {
  Serial.begin(115200);
  softwareSerial_.begin(9600);

  // Khởi tạo DFPlayer Mini
  while (!dfPlayer_.begin(softwareSerial_)) {
    Serial.println("DFPlayer Mini not detected. Retrying...");
    delay(1000);
  }
  Serial.println("DFPlayer Mini connected.");
  dfPlayer_.setTimeOut(500);
  dfPlayer_.volume(20);
  dfPlayer_.EQ(DFPLAYER_EQ_NORMAL);
  dfPlayer_.outputDevice(DFPLAYER_DEVICE_SD);

  // Khởi tạo BLE
  BLEDevice::init("");
  pBLEScan_ = BLEDevice::getScan();
  pBLEScan_->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true);
  pBLEScan_->setActiveScan(true);
  pBLEScan_->setInterval(100);
  pBLEScan_->setWindow(99);

  // Thiết lập chân nút nhấn và gắn hàm ngắt
  pinMode(buttonPin_, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin_), handleButtonPressISR, FALLING);

  // Khởi tạo dải đèn LED RGB
  strip_.begin();
  strip_.show();
}

// Phương thức vòng lặp chính
void Call_Notification::loop() {
  pBLEScan_->start(1, false);
  int ana = readAnalog();

  if (bleConnect_) {
    Serial.printf("ana: %d  -  rssi: %d  = %d\n", ana, rssi_, (ana - rssi_));
    int delta = -(ana - rssi_);
    setLEDsBasedOnRSSI(delta);

    if (rssi_ > ana) {
      if (rssiStartTime_ == 0) {
        rssiStartTime_ = millis();
      } else if (millis() - rssiStartTime_ >= RSSI_CHECK_DURATION) {
        rssiCheck_ = true;
      }
    } else {
      rssiCheck_ = false;
      rssiStartTime_ = 0;
    }
    Serial.printf("rssi_check: %d\n", rssiCheck_);

    if (rssiCheck_) {
      if (!isPlaying_) {
        playMusic();
        isPlaying_ = true;
        lastPlayTime_ = millis();
      }
    }

    if ((millis() - buttonV2LastPress_) > TIME1) {
      buttonV2_ = false;
    }

    if (buttonV2_ || (isPlaying_ && (millis() - lastPlayTime_ > 7000))) {
      stopMusic();
      isPlaying_ = false;
    }

  } else {
    for (int i = 0; i < NUM_LEDS; i++) {
      strip_.setPixelColor(i, strip_.Color(0, 0, 0));
    }
    strip_.show();
  }

  bleConnect_ = false;
}

// Phương thức điều khiển phát nhạc
void Call_Notification::playMusic() {
  dfPlayer_.playFolder(2, 7);
  Serial.println("Playing music...");
}

// Phương thức điều khiển dừng nhạc
void Call_Notification::stopMusic() {
  dfPlayer_.stop();
  Serial.println("Stopping music...");
}

// Hàm ngắt tĩnh để xử lý nút nhấn
void IRAM_ATTR Call_Notification::handleButtonPressISR() {
  instance_->handleButtonPress();
}

// Phương thức xử lý nút nhấn
void Call_Notification::handleButtonPress() {
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTime_) > DEBOUNCE_DELAY) {
    buttonState_ = !buttonState_;
    lastDebounceTime_ = currentTime;
    if (buttonState_) {
      Serial.println("Button is pressed");
    }

    if (isPlaying_) {
      buttonV2_ = true;
      buttonV2LastPress_ = millis();
    }
  }
}

// Đọc giá trị analog từ biến trở
int Call_Notification::readAnalog() {
  int potValue = analogRead(ANALOG_PIN);
  int potPercentage = map(potValue, 0, 4095, RSSI_XA, RSSI_GAN);
  return potPercentage;
}

// Thiết lập dải đèn LED dựa trên giá trị RSSI
void Call_Notification::setLEDsBasedOnRSSI(int delta) {
  int numLedsToLight = 0;
  if (delta <= 0) {
    numLedsToLight = 0;
  } else if (delta > 0 && delta <= 10) {
    numLedsToLight = 2;
  } else if (delta > 10 && delta <= 20) {
    numLedsToLight = 4;
  } else if (delta > 20 && delta <= 30) {
    numLedsToLight = 6;
  } else if (delta > 30 && delta <= 40) {
    numLedsToLight = 8;
  } else if (delta > 40) {
    numLedsToLight = 10;
  }

  for (int i = 0; i < NUM_LEDS; i++) {
    if (i < numLedsToLight) {
      strip_.setPixelColor(i, strip_.Color(0, 150, 0));  // Màu xanh lá
    } else {
      strip_.setPixelColor(i, strip_.Color(0, 0, 0));  // Tắt
    }
  }
  strip_.show();
}

// Hàm tĩnh xử lý thiết bị BLE quảng cáo
void Call_Notification::onBLEAdvertisedDeviceResult(BLEAdvertisedDevice advertisedDevice) {
  if (advertisedDevice.haveName() && advertisedDevice.getName() == "Robot") {
    instance_->rssi_ = advertisedDevice.getRSSI();
    instance_->bleConnect_ = true;
  }
}

// Lớp callback để xử lý thiết bị BLE quảng cáo
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Call_Notification::onBLEAdvertisedDeviceResult(advertisedDevice);
  }
};

// Khởi tạo đối tượng Call_Notification
Call_Notification callNotification(2, 17);

void setup() {
  callNotification.setup();
}

void loop() {
  callNotification.loop();
}
