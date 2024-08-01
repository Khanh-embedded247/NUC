#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <DFRobotDFPlayerMini.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>

// Định nghĩa các hằng số và chân
//***DFPLAYER***
#define DFPLAYER_TX_PIN 21
#define DFPLAYER_RX_PIN 22

#define ANALOG_PIN 14  //Triet ap
#define NUM_LEDS 10    //so led rgb
#define RSSI_XA -125    //output_start
#define RSSI_GAN -60   //output_end
#define TIME1 30000    //time of button
#define DEBOUNCE_DELAY 50
#define RSSI_CHECK_DURATION 10000  //time of check

class Call_Nofitication {
public:
  /*COnstructor cos tham so*/
  Call_Nofitication(int buttonPin, int ledPin);
  void setup();
  void loop();

private:
  void playMusic();                                                               // Method điều khiển phát nhạc
  void stopMusic();                                                               // Method điều khiển dừng nhạc
  static void handleButtonPressISR();                                             // Hàm ngắt tĩnh để xử lý nút nhấn
  void handleButtonPress();                                                       // Method xử lý nút nhấn
  int readAnalog();                                                               // Đọc giá trị analog từ biến trở
  void setLEDsBasedOnRSSI(int delta);                                             // Thiết lập dải đèn LED dựa trên giá trị RSSI
  static void onBLEAdvertisedDeviceResult(BLEAdvertisedDevice advertisedDevice);  // Hàm tĩnh xử lý thiết bị BLE quảng cáo
  void flashRGB(int duration);
  //create objects
  DFRobotDFPlayerMini dfPlayer_;
  SoftwareSerial softwareSerial_;
  Adafruit_NeoPixel strip_;
  BLEScan* pBLEScan_;
  // Khai bao cac atributes
  int buttonPin_;
  int ledPin_;

  bool isPlaying_;
  bool rssiCheck_;
  bool bleConnect_;
  bool buttonV2_;
  volatile bool buttonState_;
  unsigned long lastPlayTime_;
  int rssi_;
  int analog_;
  unsigned long rssiStartTime_;
  unsigned long buttonV2LastPress_;
  volatile unsigned long lastDebounceTime_;
  static Call_Nofitication* instance_;

  // Lớp callback để xử lý thiết bị BLE quảng cáo
  class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Call_Nofitication::onBLEAdvertisedDeviceResult(advertisedDevice);
    }
  };
};
//Static variable để reference đến bất kỳ objects nào của class.
Call_Nofitication* Call_Nofitication::instance_ = nullptr;

// COnstructor have parameters
Call_Nofitication::Call_Nofitication(int buttonPin, int ledPin)
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

// Phương thức cài đặt
void Call_Nofitication::setup() {
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
  attachInterrupt(digitalPinToInterrupt(buttonPin_), handleButtonPressISR, FALLING);  //Interrupt

  // Khởi tạo dải đèn LED RGB
  strip_.begin();
  strip_.show();
}

// Phương thức vòng lặp chính
void Call_Nofitication::loop() {
  pBLEScan_->start(1, false);  // Quét BLE trong 1 giây mà không tái khởi động
  int ana = readAnalog();

  if (bleConnect_) {
    flashRGB(100);  // Nhấp nháy đèn LED mỗi giây khi có tín hiệu BLE

    Serial.printf("delta=ana -  rssi = %d - %d = %d\n", ana, rssi_, ana - (rssi_));
    int delta = -(ana - rssi_);
    Serial.println("**********Delta***********");
    Serial.println(delta);
    setLEDsBasedOnRSSI(delta);
    //cường độ tín hiệu BLE> giá trị từ biến trở (ana) hay không
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
    //reset button sau 30s
    if ((millis() - buttonV2LastPress_) > TIME1) {
      buttonV2_ = false;
    }

    if (buttonV2_ || (isPlaying_ && (millis() - lastPlayTime_ > 5000))) {
      stopMusic();
      isPlaying_ = false;
    }

  } else {
    for (int i = 0; i < NUM_LEDS; i++) {
      strip_.setPixelColor(i, strip_.Color(0, 0, 0));  // Tắt
    }
    strip_.show();
  }

  bleConnect_ = false;
}

// Phương thức điều khiển phát nhạc
void Call_Nofitication::playMusic() {
  dfPlayer_.play(1);
  Serial.println("Playing music...");
}

// Phương thức điều khiển dừng nhạc
void Call_Nofitication::stopMusic() {
  dfPlayer_.stop();
  Serial.println("Stopping music...");
}

// Hàm ngắt tĩnh để xử lý nút nhấn
void IRAM_ATTR Call_Nofitication::handleButtonPressISR() {
  instance_->handleButtonPress();
}

// Phương thức xử lý nút nhấn
void Call_Nofitication::handleButtonPress() {
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
int Call_Nofitication::readAnalog() {
  int potValue = analogRead(ANALOG_PIN);
  Serial.println("-------Gia tri analog-------");
  Serial.println(potValue);
  int potPercentage = map(potValue, 0, 4095, RSSI_GAN, RSSI_XA);  //chuyển đổi điện áp analog (potValue) sang thang đo RSSI
  Serial.println("******gia tri quy doi sang rssi*******");
  Serial.println(potPercentage);
  return potPercentage;
}

// Thiết lập dải đèn LED dựa trên giá trị cheenh lech RSSI
void Call_Nofitication::setLEDsBasedOnRSSI(int delta) {
  int numLedsToLight = 0;
  if (delta <= 0) {
    numLedsToLight = 0;
  } else if (delta > 0 && delta <= 5) {
    numLedsToLight = 1;
  } else if (delta > 5 && delta <= 10) {
    numLedsToLight = 2;
  } else if (delta > 10 && delta <= 15) {
    numLedsToLight = 3;
  } else if (delta > 15 && delta <= 20) {
    numLedsToLight = 4;
  } else if (delta > 20 && delta <= 25) {
    numLedsToLight = 5;
  } else if (delta > 25 && delta <= 30) {
    numLedsToLight = 6;
  } else if (delta > 30 && delta <= 35) {
    numLedsToLight = 7;
  } else if (delta > 35 && delta <= 40) {
    numLedsToLight = 8;
  } else if (delta > 40 && delta <= 45) {
    numLedsToLight = 9;
  } else if (delta > 45) {//RSSI=-60;
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
// Nhấp nháy đèn LED trong khoảng thời gian nhất định
void Call_Nofitication::flashRGB(int duration) {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip_.setPixelColor(i, strip_.Color(0, 150, 0));  // Bật đèn LED
  }
  strip_.show();
  delay(duration);
}
// Hàm tĩnh xử lý thiết bị BLE quảng cáo
void Call_Nofitication::onBLEAdvertisedDeviceResult(BLEAdvertisedDevice advertisedDevice) {
  if (advertisedDevice.haveName() && advertisedDevice.getName() == "Robot") {
    instance_->rssi_ = advertisedDevice.getRSSI();
    instance_->bleConnect_ = true;
  }
}

// Khởi tạo đối tượng Call_Nofitication
int buttonPin = 15;                                 // Định nghĩa chân nút nhấn
Call_Nofitication callNotification(buttonPin, 17);  //17 led rgb

void setup() {
  callNotification.setup();
}

void loop() {
  callNotification.loop();
}
/*

PotValue (0-4095)	potPercentage (RSSI_XA đến RSSI_GAN)
0	    -95
500	  -80
1000   -65
1500	 -50
2000	 -35
2500	 -20
3000	 -5
3500	 10
4095	 25
*/