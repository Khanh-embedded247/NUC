#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEClient.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <EEPROM.h>
//Library for dfplayer 
#include <DFRobotDFPlayerMini.h>
//Library for RGB led
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SoftwareSerial.h>

// Định nghĩa pin
#define BUTTON_PIN 15
#define LED_PIN 2
#define BATTERY_PIN 34
#define IR_PIN 4
#define DFPLAYER_TX_PIN 21
#define DFPLAYER_RX_PIN 22
#define NEOPIXEL_PIN 14

// Định nghĩa số lượng LED
#define NUM_LEDS 120

// Định nghĩa UUID BLE
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Các hằng số khác
#define EEPROM_ADDRESS 0
#define ADD_MODE_HOLD_TIME 2000
#define DEBOUNCE_DELAY 50
#define RANDOM_CODE_LENGTH 8

// Biến trạng thái
bool buttonHeld = false;
bool inBroadcastMode = false;
bool soundMuted = false;
unsigned long buttonPressTime = 0;
unsigned long lastDebounceTime = 0;
unsigned long muteStartTime = 0;
bool newIRData = false;
unsigned long lastReceiveTime = 0;

// BLE client và server
BLEClient *pClient;
BLEServer *pServer;
BLECharacteristic *pCharacteristic;

// IR Receiver
IRrecv irrecv(IR_PIN);
decode_results results_IR;

// DFPlayer
SoftwareSerial mySoftwareSerial(DFPLAYER_RX, DFPLAYER_TX);
DFRobotDFPlayerMini myDFPlayer;

// NeoPixel
Adafruit_NeoPixel strip(NUM_LEDS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  irrecv.enableIRIn();
  EEPROM.begin(512);

  // Khởi tạo nút nhấn và LED
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  ledcSetup(0, 5000, 8);
  ledcAttachPin(LED_PIN, 0);

  // Khởi tạo BLE
  BLEDevice::init("Phenikaa");
  pClient = BLEDevice::createClient();
  pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
  );
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  // Khởi tạo DFPlayer
  mySoftwareSerial.begin(9600);
  if (!myDFPlayer.begin(mySoftwareSerial)) {
    Serial.println("DFPlayer không được tìm thấy.");
    while (true);
  }
  myDFPlayer.setTimeOut(500);
  myDFPlayer.volume(10);

  // Khởi tạo NeoPixel
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  handleButtonPress();
  if (!inBroadcastMode) {
    updateBatteryLED();
  }

  // Kiểm tra kết nối DFPlayer và khôi phục nếu mất kết nối
  if (!myDFPlayer.available()) {
    myDFPlayer.begin(mySoftwareSerial);
  }

  // Xử lý chế độ phát
  if (inBroadcastMode) {
    unsigned long currentTime = millis();
    if (currentTime - buttonPressTime <= 5000) {
      uint32_t randomCode = random(0x10000000, 0xFFFFFFFF);
      irrecv.sendNEC(randomCode, 32);
      saveToEEPROM(randomCode);
      newIRData = true;
      Serial.print("Phát mã ngẫu nhiên: ");
      Serial.println(randomCode, HEX);
    } else {
      inBroadcastMode = false;
      // Chuyển LED sang màu tím trong 2 giây
      for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, strip.Color(128, 0, 128));
      }
      strip.show();
      delay(2000);
      // Trở về chế độ bình thường
      updateBatteryLED();
    }
  }

  // Kiểm tra và gửi dữ liệu qua BLE khi nhận được mã IR hợp lệ ở chế độ bình thường
  if (irrecv.decode(&results_IR)) {
    if (results_IR.value == getEEPROMCode()) {
      myDFPlayer.play(1); // Giả sử file nhạc số 1
    }
    irrecv.resume();  // Chuẩn bị cho dữ liệu tiếp theo
  }

  // Dừng âm thanh nếu nhấn nút khi âm thanh đang kêu
  if (myDFPlayer.available() && myDFPlayer.readType() == DFPlayerPlayFinished && soundMuted) {
    unsigned long currentTime = millis();
    if (currentTime - muteStartTime >= 30000) {
      soundMuted = false;
    }
  }
}

void handleButtonPress() {
  unsigned long currentTime = millis();
  if ((currentTime - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (digitalRead(BUTTON_PIN) == LOW && !buttonHeld) {
      buttonPressTime = millis();
      buttonHeld = true;
    } else if (digitalRead(BUTTON_PIN) == HIGH && buttonHeld) {
      buttonHeld = false;
      if (millis() - buttonPressTime >= ADD_MODE_HOLD_TIME) {
        inBroadcastMode = true;
        buttonPressTime = millis();
      } else if (myDFPlayer.available()) {
        myDFPlayer.stop();
        soundMuted = true;
        muteStartTime = millis();
      }
    }
    lastDebounceTime = currentTime;
  }
}

void updateBatteryLED() {
  int batteryLevel = analogRead(BATTERY_PIN);
  int batteryPercent = map(batteryLevel, 0, 4095, 0, 100);
  if (batteryPercent < 20) {
    // Nháy LED nếu pin dưới 20%
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
  } else {
    // Điều chỉnh độ sáng của LED tương ứng với % pin
    int brightness = map(batteryPercent, 20, 100, 51, 255);
    ledcWrite(0, brightness);
  }
}

void saveToEEPROM(uint32_t code) {
  EEPROM.put(EEPROM_ADDRESS, code);
  EEPROM.commit();
}

uint32_t getEEPROMCode() {
  uint32_t code;
  EEPROM.get(EEPROM_ADDRESS, code);
  return code;
}
