// Thu vien BLE ESP
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
// Thu vien IR
#include <IRremoteESP8266.h>
 #include <IRrecv.h>
 #include <IRutils.h>
#include <EEPROM.h>

// Tao ma dac tinh rieng nhan tu robot
static unsigned long validIRCode = 0x24DB07F8;
// Chan thu IR
#define RECEIVE_PIN 4
// Tao doi tuong IR
IRrecv irrecv(RECEIVE_PIN);
decode_results results_IR;
// Cac bien xu ly nhan tin hieu IR
unsigned long lastReceiveTime = 0;
const unsigned long debounceDelay_IR = 1000;  // Thời gian chờ debouncing (ms)

// Dinh nghia ma connect BLE
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
// Button
#define BUTTON_PIN 15
#define LED_PIN 2  // Đèn LED báo hiệu
// Thời gian giữ nút nhấn để vào chế độ ADD
#define ADD_MODE_HOLD_TIME 5000
#define ADD_MODE_RESET 10000
// Thời gian debounce button (ms)
const unsigned long debounceDelay_Button = 50;
// Status cua nut nhan
volatile bool buttonState = false;
// Biến lưu trữ trạng thái chế độ ADD
bool inAddMode = false;
// Biến lưu trữ trạng thái reset EEPROM
bool resetEEPROM = false;
// Biến kiểm tra xem có dữ liệu mới hay không
bool newIRData = false;
//Bieen kiem tra xem nut nhan co giu khong
bool buttonHeld = false;
// Thời gian nút nhấn được nhấn
unsigned long buttonPressTime = 0;
// Thời gian bắt đầu chế độ ADD
unsigned long addModeStartTime = 0;
// Địa chỉ EEPROM để lưu dữ liệu
#define EEPROM_ADDRESS 0

unsigned long lastDebounceTime = 0;

BLECharacteristic *pCharacteristic;
int value = 0;

void IRAM_ATTR handleButtonPress() {
  // Lấy thời gian hiện tại
  unsigned long currentTime = millis();

  /*****************Handles switching NORMAL modes *****************/
  if ((currentTime - lastDebounceTime) > debounceDelay_Button) {
    // Đổi trạng thái của buttonState khi nút nhấn được nhấn
    buttonState = true;
    // Cập nhật thời gian ngắt gần nhất
    lastDebounceTime = currentTime;
    // In trạng thái của nút nhấn ra màn hình
    Serial.println("Button is pressed");
  }
  /*****************Handles switching RESET modes *****************/
  if (digitalRead(BUTTON_PIN) == LOW && !buttonHeld) {
    // Nếu nút nhấn đang được giữ
    if (buttonPressTime == 0) {
      buttonPressTime = millis();  //tao tg bawt dau giu nut nhan
    } else if ((millis() - buttonPressTime) >= ADD_MODE_RESET) {
      // Nếu nút nhấn đã được giữ đủ thời gian để reset EEPROM
      resetEEPROM = true;
      Serial.println("Reset EEPROM");
      buttonPressTime = 0;  // Reset thời gian nút nhấn
      buttonHeld = true; // Đánh dấu nút đang được giữ
    }
    /******************Handles switching ADD modes******************/
    else if ((millis() - buttonPressTime) >= ADD_MODE_HOLD_TIME) {
      // Nếu nút nhấn đã được giữ đủ thời gian để vào chế độ ADD
      inAddMode = true;
      addModeStartTime = millis();  //Khoi tao tg bat dau che do ADD
      Serial.println("Accessed ADD mode");
      // Nháy đèn LED liên tục trong 3 giây khi vào chế độ ADD
      unsigned long blinkStartTime = millis();
      while (millis() - blinkStartTime < 3000) {
        digitalWrite(LED_PIN, HIGH);
        delay(250);
        digitalWrite(LED_PIN, LOW);
        delay(250);
      }
      buttonPressTime = 0;  // Reset thời gian nút nhấn
      buttonHeld = true; // Đánh dấu nút đang được giữ
    }
  } else {
    // Nếu nút nhấn không được nhấn, reset thời gian nút nhấn
    buttonPressTime = 0;
    buttonHeld = false; // Đánh dấu nút không còn được giữ
  }
}
//Function save to hard memory
void saveToEEPROM(unsigned long data) {
  EEPROM.put(EEPROM_ADDRESS, data);
  EEPROM.commit();
  Serial.println("Saved to EEPROM");
}
//Function reset hard memory
void resetEEPROMFunction() {
  // LED sáng rồi tắt dần khi reset EEPROM
  for (int brightness = 0; brightness <= 255; brightness++) {
    analogWrite(LED_PIN, brightness);
    delay(10);
  }
  for (int brightness = 255; brightness >= 0; brightness--) {
    analogWrite(LED_PIN, brightness);
    delay(10);
  }

  for (int i = 0; i < 512; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  Serial.println("EEPROM has been reset");

  // Đảm bảo LED tắt sau khi reset EEPROM xong
  digitalWrite(LED_PIN, LOW);
}


void setup() {
  Serial.begin(115200);
  irrecv.enableIRIn();  // Bắt đầu bộ thu IR

  Serial.println("Emitting a Bluetooth signal");

  BLEDevice::init("Robot H1");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  // Tao dac tinh doc, viet, thong bao
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

  // pCharacteristic->setValue("Hello World says Neil");
  // Start service BLE
  pService->start();
  // Tao doi tuong thiet lap va start qua trinh Advertising BLE
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  // Add dich vu vao advertising
  pAdvertising->addServiceUUID(SERVICE_UUID);
  // Kich hoat phan hoi quet, thiet bi nao quet duoc BLE server se co thong tin BLE server
  pAdvertising->setScanResponse(true);
  // Setup thoi gian advertise min --> iOS
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();  // Start advertise
  Serial.println("Da publish thong tin ban than");

  // Khởi tạo EEPROM
  EEPROM.begin(512);

  // Khởi tạo pin button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);  // Khởi tạo LED pin
}

void loop() {
  // value++;
  // // Du lieu gui di
  // pCharacteristic->setValue(value);
  // pCharacteristic->notify();
  // delay(2000);

  handleButtonPress();
  //Call the hard memory reset function
  if (resetEEPROM) {
    resetEEPROMFunction();
    resetEEPROM = false;
  }
//Handle ADD mode
  if (inAddMode) {
    unsigned long currentTime = millis();
    if (currentTime - addModeStartTime <= 5000) {
      // Nếu đang ở chế độ ADD và thời gian chưa vượt quá 5 giây
      if (irrecv.decode(&results_IR)) {
        if (results_IR.value != 0) {
          saveToEEPROM(results_IR.value);
          newIRData = true;
          Serial.println("IR code received and saved");
        }
        irrecv.resume();  // Chuẩn bị cho dữ liệu tiếp theo
      }
    } else {
      // Nếu hết 5 giây, kiểm tra xem có nhận được dữ liệu hay không
      if (newIRData) {
        // Nháy LED 3 giây nếu có dữ liệu
        for (int i = 0; i < 6; i++) {
          digitalWrite(LED_PIN, HIGH);
          delay(250);
          digitalWrite(LED_PIN, LOW);
          delay(250);
        }
      } else {
        // Tắt LED trong 1 giây nếu không có dữ liệu
        digitalWrite(LED_PIN, LOW);
        delay(2000);
      }
      newIRData = false;
      inAddMode = false;
      Serial.println("Exited ADD mode");
    }
  }

  // Kiểm tra và gửi dữ liệu qua BLE khi nhận được mã IR hợp lệ ở chế độ bình thường
  if (!inAddMode && irrecv.decode(&results_IR)) {
    if (results_IR.value == validIRCode) {
      Serial.print("Nhận được mã BLE: ");
      serialPrintUint64(results_IR.value, HEX);  // In mã nhận được dưới dạng HEX
      Serial.println("");
      unsigned long eepromData;
      EEPROM.get(EEPROM_ADDRESS, eepromData);
      pCharacteristic->setValue((uint8_t *)&eepromData, sizeof(eepromData));
      pCharacteristic->notify();
      Serial.println("Dữ liệu đã gửi qua BLE");
    }
    irrecv.resume();  // Chuẩn bị cho dữ liệu tiếp theo
  }
}