#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <EEPROM.h>

constexpr char SERVICE_UUID[] = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
constexpr char CHARACTERISTIC_UUID[] = "beb5483e-36e1-4688-b7f5-ea07361b26a8";

//Initialize the pinout
constexpr uint8_t BUTTON_PIN = 15;
constexpr uint8_t LED_PIN = 2;
constexpr uint16_t kRecvPin = 4;
//Initialize time to enter modes
constexpr unsigned long ADD_MODE_HOLD_TIME = 5000;
constexpr unsigned long ADD_MODE_RESET = 10000;
constexpr unsigned long MAX_PRESS_DURATION = 2000;

constexpr uint16_t EEPROM_SIZE = 512;
constexpr unsigned long VALID_IR_CODE = 0x24DB07F8;
constexpr unsigned long BLINK_INTERVAL = 500;  

bool inAddMode = false;

// Lớp quản lý nút nhấn
class ButtonManager {
public:
  ButtonManager(uint8_t pin)
    : buttonPin(pin), lastButtonState(HIGH), buttonState(HIGH), buttonPressTime(0), lastDebounceTime(0) {}

  void begin() {
    pinMode(buttonPin, INPUT_PULLUP);
    lastButtonState = digitalRead(buttonPin);
    buttonState = lastButtonState;
  }

  int handleButtonPress() {
    unsigned long currentTime = millis();
    int reading = digitalRead(buttonPin);

    if (reading != lastButtonState) {
      lastDebounceTime = currentTime;
    }

    if ((currentTime - lastDebounceTime) > debounceDelay) {
      if (reading != buttonState) {
        buttonState = reading;
        if (buttonState == LOW) {
          buttonPressTime = currentTime;
        } else {
          unsigned long pressDuration = currentTime - buttonPressTime;
          Serial.print("Press Duration: ");
          Serial.println(pressDuration);
          lastButtonState = reading;
          return pressDuration;
        }
      }
    }
    lastButtonState = reading;
    return -1;
  }

private:
  uint8_t buttonPin;
  int buttonState;
  int lastButtonState;
  unsigned long buttonPressTime;
  unsigned long lastDebounceTime;
  static constexpr unsigned long debounceDelay = 50;
};

// Lớp quản lý EEPROM
class EEPROMManager {
public:
  EEPROMManager(uint16_t size)
    : eepromSize(size) {}

  void begin() {
    EEPROM.begin(eepromSize);
  }

  
  int findEmptySlot() const {
    for (uint16_t i = 0; i < eepromSize; i += sizeof(unsigned long)) {
      unsigned long storedCode;
      EEPROM.get(i, storedCode);
      if (storedCode == 0xFFFFFFFF || storedCode == 0) {
        return i;
      }
    }
    return -1;
  }

  // Kiểm tra xem giá trị có tồn tại trong EEPROM không
  bool isValueInEEPROM(unsigned long value) const {
    for (uint16_t i = 0; i < eepromSize; i += sizeof(unsigned long)) {
      unsigned long storedCode;
      EEPROM.get(i, storedCode);
      if (storedCode == value) {
        return true;
      }
    }
    return false;
  }

  // Lưu giá trị vào vị trí trống đầu tiên trong EEPROM
  void saveToEEPROM(unsigned long data) {
    int emptySlot = findEmptySlot();
    if (emptySlot != -1) {
      EEPROM.put(emptySlot, data);
      EEPROM.commit();
      Serial.print("Saved to EEPROM at address: ");
      Serial.println(emptySlot);
    } else {
      Serial.println("No empty slot available in EEPROM!");
    }
  }

  // Đặt lại EEPROM bằng cách điền giá trị mặc định
  void resetEEPROM() {
    for (uint16_t i = 0; i < eepromSize; i += sizeof(unsigned long)) {
      EEPROM.put(i, 0xFFFFFFFF);
    }
    EEPROM.commit();
    Serial.println("EEPROM has been reset");
  }

  
  void printAllData() const {
    Serial.println("All EEPROM Data:");
    for (uint16_t i = 0; i < eepromSize; i += sizeof(unsigned long)) {
      unsigned long storedCode;
      EEPROM.get(i, storedCode);
      if (storedCode != 0xFFFFFFFF && storedCode != 0) {
        Serial.print("Address ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(storedCode, HEX);
      }
    }
  }

private:
  uint16_t eepromSize;
};

// Lớp quản lý LED
class LEDManager {
public:
  LEDManager(uint8_t pin)
    : ledPin(pin), lastBlinkTime(0), blinkState(false), brightness(255) {}

  void begin() {
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);
  }


  void blink(int times, int delayTime) const {
    for (int i = 0; i < times; i++) {
      digitalWrite(ledPin, HIGH);
      delay(delayTime);
      digitalWrite(ledPin, LOW);
      delay(delayTime);
    }
  }
  void setBrightness(int brightness) const {
    analogWrite(ledPin, brightness);
  }

  void continuousBlink(unsigned long interval) {
    unsigned long currentTime = millis();
    if (currentTime - lastBlinkTime >= interval) {
      lastBlinkTime = currentTime;
      blinkState = !blinkState;
      digitalWrite(ledPin, blinkState ? HIGH : LOW);
    }
  }

  // Làm mờ LED dần
  void fadeOut() {
    for (int i = brightness; i >= 0; i--) {
      analogWrite(ledPin, i);
      delay(10);
    }
    brightness = 0;
  }

  void fadeIn() {
    for (int i = 0; i <= 255; i++) {
      analogWrite(ledPin, i);
      delay(10);
    }
    brightness = 255;
  }

  void turnOff() {
    digitalWrite(ledPin, LOW);
  }

  void turnOn() {
    digitalWrite(ledPin, HIGH);
  }

private:
  uint8_t ledPin;
  unsigned long lastBlinkTime;
  bool blinkState;
  int brightness;
};

// Lớp quản lý BLE
class BLEManager {
public:
  BLEManager(const char* serviceUUID, const char* characteristicUUID)
    : serviceUUID(serviceUUID), characteristicUUID(characteristicUUID), pServer(nullptr), pCharacteristic(nullptr) {}

  void begin() {
    BLEDevice::init("Robot H1");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService* pService = pServer->createService(serviceUUID);
    pCharacteristic = pService->createCharacteristic(
      characteristicUUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(serviceUUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    Serial.println("Bluetooth device started advertising");
  }

  // Thông báo các thiết bị kết nối với giá trị được cho
  void notify(unsigned long value) {
    pCharacteristic->setValue((uint8_t*)&value, sizeof(value));
    pCharacteristic->notify();
    Serial.print("Notified value: ");
    Serial.println(value, HEX);
  }

private:
  class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
      Serial.println("Device connected");
    }

    void onDisconnect(BLEServer* pServer) override {
      Serial.println("Device disconnected");
      BLEDevice::startAdvertising();
    }
  };

  const char* serviceUUID;
  const char* characteristicUUID;
  BLEServer* pServer;
  BLECharacteristic* pCharacteristic;
};

// Lớp quản lý IR
class IRManager {
public:
  IRManager(uint16_t recvPin)
    : irRecv(recvPin) {}

  void begin() {
    irRecv.enableIRIn();
  }

  bool decode() {
    if (irRecv.decode(&results)) {
      currentCode = results.value;
      irRecv.resume();
      return true;
    }
    return false;
  }

  unsigned long getResult() const {
    return currentCode;
  }

private:
  IRrecv irRecv;
  decode_results results;
  unsigned long currentCode;
};

// Lớp quản lý thiết bị chính
class Device {
public:
  Device(uint16_t irPin, uint8_t buttonPin, uint16_t eepromSize, uint8_t ledPin)
    : irManager(irPin), buttonManager(buttonPin), eepromManager(eepromSize), ledManager(ledPin), bleManager(SERVICE_UUID, CHARACTERISTIC_UUID) {}

  void begin() {
    irManager.begin();
    buttonManager.begin();
    eepromManager.begin();
    ledManager.begin();
    bleManager.begin();
    eepromManager.printAllData();
  }

  void loop() {
    int pressDuration = buttonManager.handleButtonPress();
    if (pressDuration == 0) {
      ledManager.turnOn();
    } else if (pressDuration != -1) {
      if (pressDuration < MAX_PRESS_DURATION) {
        Serial.println("Nut nhan da duoc nhan");
        ledManager.blink(4, 250);
      } else if (pressDuration >= ADD_MODE_RESET) {
        Serial.println("Resetting EEPROM");
        ledManager.fadeOut();
        delay(5000);
        eepromManager.resetEEPROM();
        ledManager.turnOn();
        delay(1000);
        ledManager.turnOff();
        eepromManager.printAllData();
        ledManager.begin();  
      } else if (pressDuration >= ADD_MODE_HOLD_TIME) {
        Serial.println("Entering ADD mode");
        inAddMode = true;
        handleAddMode();
        ledManager.begin();  
      }
    }

    if (irManager.decode()) {
      unsigned long irValue = irManager.getResult();
      Serial.print("Received IR code: ");
      Serial.println(irValue, HEX);
      receivedCodes[irValue]++;
      handleIRCode(irValue);
    }

    //handleBatteryStatus();  
  }

private:
  IRManager irManager;
  ButtonManager buttonManager;
  EEPROMManager eepromManager;
  LEDManager ledManager;
  BLEManager bleManager;
  std::map<unsigned long, int> receivedCodes;

  // void handleAddMode() {
  //   unsigned long startTime = millis();
  //   bool newIRData = false;
  //   unsigned long receivedCode = 0;

  //   while (millis() - startTime < 5000) {
  //     if (irManager.decode()) {
  //       receivedCode = irManager.getResult();
  //       Serial.print("Received IR code in ADD mode: ");
  //       Serial.println(receivedCode, HEX);
  //       receivedCodes[receivedCode]++;
  //       newIRData = true;
  //       ledManager.turnOn();  // Đèn nhấp nháy khi có dữ liệu mới
  //       delay(500);
  //       ledManager.turnOff();
  //       delay(500);
  //       break;
  //     }
  //   }

  //   if (newIRData) {
  //     if (!eepromManager.isValueInEEPROM(receivedCode)) {
  //       eepromManager.saveToEEPROM(receivedCode);
  //       Serial.print("Saved received IR code to EEPROM: ");
  //       Serial.println(receivedCode, HEX);
  //     } else {
  //       Serial.println("Received IR code already in EEPROM or invalid");
  //     }
  //   } else {
  //     Serial.println("No new IR data received");
  //     ledManager.turnOff();  // Tắt đèn nếu không có dữ liệu mới
  //   }

  //   inAddMode = false;
  //   digitalWrite(LED_PIN, LOW);
  //   Serial.println("Exiting ADD mode");
  // }
  void handleAddMode() {
    unsigned long startTime = millis();
    bool newIRData = false;
    std::map<unsigned long, int> irCodeCounts;

    while (millis() - startTime < 5000) {
      if (irManager.decode()) {
        unsigned long receivedCode = irManager.getResult();
        Serial.print("Received IR code in ADD mode: ");
        Serial.println(receivedCode, HEX);
        irCodeCounts[receivedCode]++;
        newIRData = true;
        ledManager.blink(3, 250);
      }
    }

    if (newIRData) {
      unsigned long mostFrequentCode = 0;
      int maxCount = 0;

      for (const auto& pair : irCodeCounts) {
        if (pair.second > maxCount) {
          mostFrequentCode = pair.first;
          maxCount = pair.second;
        }
      }

      Serial.print("Most frequent IR code: ");
      Serial.println(mostFrequentCode, HEX);

      if (!eepromManager.isValueInEEPROM(mostFrequentCode)) {
        eepromManager.saveToEEPROM(mostFrequentCode);
        Serial.print("Saved most frequent IR code to EEPROM: ");
        Serial.println(mostFrequentCode, HEX);
        //ledManager.blink(3, 250);
      } else {
        Serial.println("Most frequent IR code already in EEPROM or invalid");
      }
    } else {
      Serial.println("No new IR data received");
      ledManager.turnOff(); 
    }

    inAddMode = false;
    digitalWrite(LED_PIN, LOW);
    Serial.println("Exiting ADD mode");
  }

  //Normal Mode
  void handleIRCode(unsigned long irValue) {
    if (irValue == VALID_IR_CODE) {
      uint8_t data[EEPROM_SIZE];
      for (int i = 0; i < EEPROM_SIZE; i++) {
        data[i] = EEPROM.read(i);
      }

      for (int i = 0; i < EEPROM_SIZE; i += sizeof(unsigned long)) {
        unsigned long storedCode;
        EEPROM.get(i, storedCode);

        if (storedCode != 0xFFFFFFFF && storedCode != 0) {
          Serial.print("Sending stored code via BLE: ");
          Serial.println(storedCode, HEX);
          bleManager.notify(storedCode);
          delay(50);
        }
      }
      Serial.println("Completed sending stored codes via BLE.");
    }
  }

  void handleBatteryStatus() {
    int batteryLevel = getBatteryLevel();
    Serial.print("Battery Level: ");
    Serial.println(batteryLevel);

    if (batteryLevel < 20) {
      ledManager.continuousBlink(BLINK_INTERVAL);
    } else {
      ledManager.setBrightness(map(batteryLevel, 0, 100, 0, 255));
      ledManager.turnOn();
    }
  }

  int getBatteryLevel() {
    return analogRead(kRecvPin) * 100 / 1023;
  }
};

Device device(kRecvPin, BUTTON_PIN, EEPROM_SIZE, LED_PIN);

void setup() {
  Serial.begin(115200);
  device.begin();
  Serial.println("Device setup completed");
}

void loop() {
  device.loop();
}
