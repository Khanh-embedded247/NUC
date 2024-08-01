#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <EEPROM.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BUTTON_PIN 15
#define LED_PIN 2
#define ADD_MODE_HOLD_TIME 5000
#define ADD_MODE_RESET 10000
#define EEPROM_SIZE 512
#define MAX_PRESS_DURATION 2000
const uint16_t kRecvPin = 4;
static unsigned long validIRCode = 0x24DB07F8;

IRrecv irrecv(kRecvPin);
decode_results results_IR;
unsigned long lastReceiveTime = 0;
const unsigned long debounceDelay_IR = 1000;
const unsigned long debounceDelay_Button = 50;
volatile bool buttonState = false;
bool inAddMode = false;
volatile bool resetEEPROM = false;
bool newIRData = false;
bool buttonHeld = false;
unsigned long buttonPressTime = 0;
unsigned long addModeStartTime = 0;
unsigned long dataRead;
unsigned long lastDebounceTime = 0;
int lastButtonState = HIGH;
BLEServer *pServer;
BLECharacteristic *pCharacteristic;

void handleButtonPress();//xu ly nut nhan
void saveToEEPROM(unsigned long data);//luu bo nho cung
void resetEEPROMFunction();//reset bo nho cung
void setupBLE();//setup ble server
void handleAddMode();//xu ly che do add
void handleNormalMode();//xu ly che do binh thuong
void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify);//thong bao du lieu nhan dc
int findEmptySlot();//tim vi tri trong de luu nhieu ma bo tb
bool isValueInEEPROM(unsigned long value);//check du lieu bo nho cung
void printAllEEPROMData();//in cac du lieu bo nho cung

void handleButtonPress() {
  unsigned long currentTime = millis();
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonState) {
    lastDebounceTime = currentTime;
  }
  if ((currentTime - lastDebounceTime) > debounceDelay_Button) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        buttonPressTime = currentTime;
        buttonHeld = true;
      } else {
        unsigned long pressDuration = currentTime - buttonPressTime;
        Serial.print("Press Duration: ");
        Serial.println(pressDuration);
        if (pressDuration < MAX_PRESS_DURATION) {
          Serial.println("Button is pressed and released");
          unsigned long blinkStartTime = millis();
          while (millis() - blinkStartTime < 2000) {
            digitalWrite(LED_PIN, HIGH);
            Serial.println("kk");
            delay(250);
            digitalWrite(LED_PIN, LOW);
            delay(250);
          }
          buttonHeld = false;
        } else {
          Serial.println("Button is held");
        }
        if (buttonHeld) {
          if (pressDuration >= ADD_MODE_RESET) {
            resetEEPROM = true;
            Serial.println("Reset EEPROM");
          } else if (pressDuration >= ADD_MODE_HOLD_TIME) {
            inAddMode = true;
            addModeStartTime = currentTime;
            Serial.println("Accessed ADD mode");
            unsigned long blinkStartTime = millis();
            while (millis() - blinkStartTime < 5000) {
              digitalWrite(LED_PIN, HIGH);
              delay(250);
              digitalWrite(LED_PIN, LOW);
              delay(250);
            }
          }
          buttonHeld = false;
        }
        buttonPressTime = 0;
      }
    }
  }
  lastButtonState = reading;
}

int findEmptySlot() {
  for (int i = 0; i < EEPROM_SIZE; i += sizeof(unsigned long)) {
    unsigned long storedCode;
    EEPROM.get(i, storedCode);
    if (storedCode == 0xFFFFFFFF || storedCode == 0) {
      return i;
    }
  }
  return -1;
}

bool isValueInEEPROM(unsigned long value) {
  for (int i = 0; i < EEPROM_SIZE; i += sizeof(unsigned long)) {
    unsigned long storedCode;
    EEPROM.get(i, storedCode);
    if (storedCode == value) {
      return true;
    }
  }
  return false;
}

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

void resetEEPROMFunction() {
  for (int brightness = 0; brightness <= 255; brightness++) {
    analogWrite(LED_PIN, brightness);
    delay(10);
  }
  for (int brightness = 255; brightness >= 0; brightness--) {
    analogWrite(LED_PIN, brightness);
    delay(10);
  }
  for (int i = 0; i < EEPROM_SIZE; i += sizeof(unsigned long)) {
    EEPROM.put(i, 0xFFFFFFFF);
  }
  EEPROM.commit();
  Serial.println("EEPROM has been reset");
  digitalWrite(LED_PIN, LOW);
  buttonState = digitalRead(BUTTON_PIN);
  buttonHeld = false;
  buttonPressTime = 0;
  lastButtonState = buttonState;
  pinMode(LED_PIN, OUTPUT);
  Serial.println("Reset button states after EEPROM reset");
}

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    Serial.println("Device connected");
  }
  void onDisconnect(BLEServer *pServer) {
    Serial.println("Device disconnected");
    BLEDevice::startAdvertising();
  }
};

void setupBLE() {
  BLEDevice::init("Robot H1");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Da publish thong tin ban than");
}

void handleAddMode() {
  unsigned long currentTime = millis();
  while (currentTime - addModeStartTime <= 5000) {
    currentTime = millis();
    if (irrecv.decode(&results_IR)) {
      Serial.print("Nhận được mã IR: ");
      Serial.println(results_IR.value, HEX);
      if (results_IR.value != 0) {
        if (!isValueInEEPROM(results_IR.value)) {
          saveToEEPROM(results_IR.value);
          newIRData = true;
          Serial.print("Mã IR nhận được và lưu: ");
          Serial.println(results_IR.value, HEX);
        } else {
          Serial.println("Mã IR nhận được trùng với mã lưu trong EEPROM, không lưu");
        }
      } else {
        Serial.println("Không nhận được dữ liệu IR");
      }
      irrecv.resume();
    }
    delay(50);
  }
  if (newIRData) {
    Serial.print("Mã IR nhận được cuối cùng: ");
    Serial.println(results_IR.value, HEX);
    for (int i = 0; i < 6; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(250);
      digitalWrite(LED_PIN, LOW);
      delay(250);
    }
  } else {
    Serial.println("Không nhận được dữ liệu mới trong thời gian chờ");
    digitalWrite(LED_PIN, LOW);
    delay(2000);
  }
  newIRData = false;
  inAddMode = false;
  Serial.println("Exited ADD mode");
  printAllEEPROMData();
}
void handleNormalMode() {
  if (irrecv.decode(&results_IR)) {
    Serial.print("Nhận được mã IR: ");
    serialPrintUint64(results_IR.value, HEX);
    Serial.println("");
    if (results_IR.value == validIRCode) {
      uint8_t data[EEPROM_SIZE];
      for (int i = 0; i < EEPROM_SIZE; i++) {
        data[i] = EEPROM.read(i);
      }

      // Gửi từng phần tử trong mảng dữ liệu lần lượt qua BLE nếu phần tử đó có dữ liệu hợp lệ
      Serial.println("Dữ liệu đang gửi qua BLE từng phần tử có dữ liệu hợp lệ:");
      for (int i = 0; i < EEPROM_SIZE; i += sizeof(unsigned long)) {
        unsigned long storedCode;
        EEPROM.get(i, storedCode);

        // Kiểm tra nếu phần tử có dữ liệu hợp lệ (không phải giá trị mặc định 0xFFFFFFFF hoặc 0)
        if (storedCode != 0xFFFFFFFF && storedCode != 0) {
          // In ra từng byte dữ liệu trước khi gửi
          Serial.print("Gửi phần tử thứ ");
          Serial.print(i);
          Serial.print(": ");
          Serial.println(storedCode, HEX);

          // Gửi phần tử qua BLE
          pCharacteristic->setValue((uint8_t*)&storedCode, sizeof(storedCode));
          pCharacteristic->notify();
          delay(50); // Thêm độ trễ nhỏ để đảm bảo gửi tuần tự
        }
      }
      Serial.println("Hoàn thành gửi dữ liệu qua BLE.");
    }
    irrecv.resume();
  }
  delay(50);
}


void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
  Serial.print("Callback cho đặc tính ");
  Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
  Serial.print(" với độ dài dữ liệu ");
  Serial.println(length);
  Serial.print("dữ liệu: ");
  unsigned long receivedValue;
  memcpy(&receivedValue, pData, sizeof(receivedValue));
  Serial.println(receivedValue, HEX);
  unsigned long storedCode;
  EEPROM.get(0, storedCode);
  if (receivedValue == storedCode) {
    Serial.println("Mã nhận được trùng với mã lưu trong EEPROM");
  }
}

void printAllEEPROMData() {
  Serial.println("All EEPROM Data:");
  for (int i = 0; i < EEPROM_SIZE; i += sizeof(unsigned long)) {
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

void setup() {
  Serial.begin(115200);
  irrecv.enableIRIn();
  Serial.println("Emitting a Bluetooth signal");
  setupBLE();
  EEPROM.begin(EEPROM_SIZE);
  bool initialized = false;
  for (int i = 0; i < EEPROM_SIZE; i += sizeof(unsigned long)) {
    unsigned long storedCode;
    EEPROM.get(i, storedCode);
    if (storedCode != 0xFFFFFFFF) {
      initialized = true;
      break;
    }
  }
  if (!initialized) {
    for (int i = 0; i < EEPROM_SIZE; i += sizeof(unsigned long)) {
      EEPROM.put(i, 0xFFFFFFFF);
    }
    EEPROM.commit();
  }
  unsigned long dataRead;
  EEPROM.get(0, dataRead);
  Serial.print("Data read from EEPROM: ");
  Serial.println(dataRead, HEX);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  lastButtonState = digitalRead(BUTTON_PIN);
  buttonState = lastButtonState;
  printAllEEPROMData();
}

void loop() {
  handleButtonPress();
  if (inAddMode) {
    handleAddMode();
  } else {
    handleNormalMode();
  }
  if (resetEEPROM) {
    resetEEPROMFunction();
    resetEEPROM = false;
    printAllEEPROMData();
  }
}
