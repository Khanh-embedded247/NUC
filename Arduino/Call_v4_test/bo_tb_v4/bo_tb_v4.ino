#include "BLEDevice.h"            // Thư viện cho giao tiếp BLE
#include <DFRobotDFPlayerMini.h>  // Thư viện cho DFPlayer Mini
#include <EEPROM.h>               // Thư viện cho EEPROM
#include <Adafruit_NeoPixel.h>    // Thư viện cho LED RGB
#include <IRremoteESP8266.h>      // Thư viện cho truyền IR
#include <IRsend.h>               // Thư viện cho gửi IR

// Định nghĩa các chân
#define DFPLAYER_TX_PIN 21
#define DFPLAYER_RX_PIN 22
#define BUTTON_PIN 15
#define LED_PIN 16
#define IR_LED_PIN 4
#define EEPROM_ADDRESS 0  // Địa chỉ để lưu mã ngẫu nhiên trong EEPROM
#define MAX_PRESS_DURATION 2000
#define ADD_MODE_HOLD_TIME 5000
#define ADD_MODE_RESET 10000

// UUID của dịch vụ và đặc tính BLE
static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
static BLEUUID charUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");

// Cờ và con trỏ kết nối BLE
static bool doConnect = false;
static bool connected = false;
static BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
static BLEAdvertisedDevice* myDevice = nullptr;
static BLEScan* pBLEScan = nullptr;

// Tạo các đối tượng
DFRobotDFPlayerMini myDFPlayer;
Adafruit_NeoPixel strip(1, LED_PIN, NEO_GRB + NEO_KHZ800);
IRsend irsend(IR_LED_PIN);

unsigned long dataRead;               // Biến để lưu giá trị đọc từ EEPROM
volatile bool buttonPressed = false;  // Biến lưu trạng thái nút nhấn
bool isPlaying = false;               // Biến lưu trạng thái phát nhạc
bool inAddMode = false;               // Biến lưu trạng thái chế độ ADD
bool resetEEPROM = false;             // Biến lưu trạng thái reset EEPROM
bool buttonHeld = false;              // Biến lưu trạng thái nút nhấn giữ
bool shouldPlayMusic = false;         // Cờ để kiểm tra phát nhạc
bool bleSignalReceived = false;       // Đánh dấu tín hiệu BLE đã nhận
bool stopRequested = false;           // Đánh dấu yêu cầu dừng âm thanh

unsigned long buttonPressTime = 0;          // Biến lưu thời gian bắt đầu nhấn nút
const unsigned long debounceDelay = 50;     // Độ trễ chống nhiễu cho nút nhấn
unsigned long lastPlayTime = 0;             // Biến lưu thời gian phát nhạc cuối cùng
const unsigned long stopDuration = 30000;   // Thời gian dừng 30 giây
unsigned long addModeStartTime = 0;         // Biến lưu thời gian bắt đầu chế độ ADD
unsigned long randomCode = 0;               // Biến lưu mã ngẫu nhiên
const unsigned long buttonHoldTime = 2000;  // 2 giây cho giữ nút nhấn
unsigned long lastDebounceTime = 0;         // Biến lưu thời gian cuối cùng chống nhiễu
unsigned long stopStartTime = 0;
unsigned long lastBleReceivedTime = 0;  // Biến lưu thời gian nhận BLE cuối cùng
int receivedValue = 0;                  // Biến  để lưu giá trị BLE nhận được
int lastButtonState = HIGH;             // Biến  trạng thái nút nhấn trước đó
int buttonState = HIGH;                 // Biến  trạng thái hiện tại của nút nhấn
unsigned long storedCode;               // Biến để lưu mã từ EEPROM

// Liệt kê khai báo các hàm function
void DFPlayer_connect();//connect dfplayer
void  handleButtonPress();//xu ly nut nhan
void playMusic();//choi nhac
void stopMusic();//dung nhac
void handleButtonPressWithoutBLE();//xu ly nut nhan khi k co du lieu
void handleButtonPressWithBLE();//xu ly nut nhan khi co du lieu
void sendIRCode(unsigned long code);//gui ma IR taoj ra
void setLEDColor(uint32_t color);//Chonj mau
bool connectToServer();//Ket noi BLE
void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);//ham du lieu nhan duowcj ble
void resetEEPROMFunction();//reset bo nho cung
//void checkBLEData();
void handleAddMode();//che do add
void manageConnection();//reconnect
void checkSignalDuringPlayback();//xet thoi duy tri tin hieu
void handlePostStopActions();//xu ly khi het thoi gian dung nhac


void DFPlayer_connect(){
  Serial2.begin(9600, SERIAL_8N1, DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);
  while (!myDFPlayer.begin(Serial2)) {
    delay(1000);
    if (!myDFPlayer.begin(Serial2)) {
      Serial.println("DFPlayer Mini OFFLINE");
      
    }
  }
  Serial.println("DFPlayer Mini online");
  delay(1000);
  myDFPlayer.volume(30);
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
}
// Callback cho BLE Client
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) override {
    connected = true;  // Đánh dấu đã kết nối thành công
    Serial.println("Kết nối với BLE Server thành công");
  }

  void onDisconnect(BLEClient* pclient) override {
    connected = false;  // Đánh dấu mất kết nối
    Serial.println("Mất kết nối với BLE Server, đang cố gắng kết nối lại...");
    doConnect = true;  // Đặt cờ để thử kết nối lại
  }
};

// Lớp callback khi tìm thấy một thiết bị quảng cáo
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    // Callback cho các thiết bị BLE được tìm thấy
    Serial.print("Thiết bị BLE được tìm thấy: ");
    Serial.println(advertisedDevice.toString().c_str());

    // Kiểm tra xem thiết bị có UUID dịch vụ mong muốn và tên cụ thể hay không
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      if (advertisedDevice.getName() == "Robot H1") {
        pBLEScan->stop();  // Dừng quét
        myDevice = new BLEAdvertisedDevice(advertisedDevice);
        doConnect = true;  // Đặt cờ để kết nối
      }
    }
  }
};

void setup() {
  Serial.begin(115200);  // Khởi tạo Serial với baudrate 115200
  //DFPlayer_connect();
  Serial.println("Bắt đầu ứng dụng Arduino BLE Client...");
  BLEDevice::init("");  // Khởi tạo BLE

  pBLEScan = BLEDevice::getScan();  // Lấy đối tượng BLE scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);    // Đặt khoảng thời gian quét
  pBLEScan->setWindow(449);       // Đặt cửa sổ quét
  pBLEScan->setActiveScan(true);  // Đặt chế độ quét chủ động
  pBLEScan->start(1, false);      // Bắt đầu quét BLE

  pinMode(BUTTON_PIN, INPUT_PULLUP);          // Đặt chế độ cho chân nút nhấn
  lastButtonState = digitalRead(BUTTON_PIN);  // Đọc trạng thái ban đầu của nút nhấn
  buttonState = lastButtonState;              // Khởi tạo trạng thái hiện tại của nút nhấn

  strip.begin();  // Khởi tạo LED RGB
  strip.show();   // Hiển thị LED

  EEPROM.begin(512);                     // Khởi tạo EEPROM với kích thước 512 byte
  EEPROM.get(EEPROM_ADDRESS, dataRead);  // Đọc dữ liệu từ EEPROM
  Serial.print("Data read from EEPROM: ");
  Serial.println(dataRead,HEX);

  irsend.begin();  // Khởi tạo IR send

  // Khởi tạo kết nối với DFPlayer
  Serial2.begin(9600, SERIAL_8N1, DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);
  if (myDFPlayer.begin(Serial2)) {
    Serial.println("DFPlayer Mini đã kết nối thành công");
    myDFPlayer.volume(30);  // Đặt âm lượng
  } else {
    Serial.println("Không thể kết nối DFPlayer Mini");
  }
}

void loop() {
  handleButtonPress();  // Xử lý nút nhấn
  handleResetEEPROM();  // Xử lý reset EEPROM nếu cần
  handleAddMode();      // Xử lý chế độ ADD nếu cần
  manageConnection();   // Quản lý kết nối BLE

  if (shouldPlayMusic) {
    playMusic();
    shouldPlayMusic = false;
  }

  if (isPlaying) {
    checkSignalDuringPlayback();
  }

  handlePostStopActions();
}

void handleResetEEPROM() {
  if (resetEEPROM) {
    resetEEPROMFunction();
    resetEEPROM = false;
  }
}

void handleAddMode() {
  if (inAddMode) {
    unsigned long currentTime = millis();
    if (currentTime - addModeStartTime <= 5000) {  // Nếu TRONG thời gian chế độ ADD
      unsigned long storedCode;
      EEPROM.get(EEPROM_ADDRESS, storedCode);             // Lấy mã đã lưu trong EEPROM
      if (storedCode == 0xFFFFFFFF || storedCode == 0) {  // Kiểm tra xem EEPROM có trống không
        unsigned long code = random(1000, 9999);          // Tạo mã ngẫu nhiên
        saveToEEPROM(code);                               // Lưu mã vào EEPROM
        sendIRCode(code);                                 // Gửi mã IR
      } else {
        Serial.println("Bộ nhớ đã có dữ liệu.");
      }
    } else {
      inAddMode = false;  // Thoát chế độ ADD
      Serial.println("Thoát chế độ ADD");
    }
  }
}

void manageConnection() {
  if (doConnect) {
    if (connectToServer()) {
      Serial.println("Đã kết nối với BLE Server.");
    } else {
      Serial.println("Không thể kết nối với server; đang thử lại...");
    }
    doConnect = false;  // Đặt lại cờ kết nối
  }

  if (!connected) {
    pBLEScan->start(1);  // Nếu không kết nối, tiếp tục quét
  }
}

void saveToEEPROM(unsigned long data) {
  EEPROM.put(EEPROM_ADDRESS, data);  // Lưu mã vào EEPROM tại địa chỉ đầu
  EEPROM.commit();                   // confirm
  Serial.println("Đã lưu vào EEPROM");
}

void handleButtonPress() {
  unsigned long currentTime = millis();   // Lấy thời gian hiện tại
  int reading = digitalRead(BUTTON_PIN);  // Đọc trạng thái hiện tại của nút nhấn

  if (reading != lastButtonState) {  // Kiểm tra nếu trạng thái của nút nhấn đã thay đổi
    lastDebounceTime = currentTime;  // Cập nhật thời gian debounce
  }

  if ((currentTime - lastDebounceTime) > debounceDelay) {  // Tránh nhấn liên tục
    if (reading != buttonState) {                          // Kiểm tra nếu trạng thái nút nhấn đã thay đổi
      buttonState = reading;                               // Cập nhật trạng thái nút nhấn
      if (buttonState == LOW) {                            // Nếu nút nhấn đang được nhấn xuống
        buttonPressTime = currentTime;                     // Ghi lại thời gian khi nút bắt đầu được nhấn
        buttonHeld = true;
      } else {                                                        // Nếu nút nhấn đã được nhả ra
        unsigned long pressDuration = currentTime - buttonPressTime;  // Tính thời gian nút nhấn được giữ
        Serial.print("Press Duration: ");
        Serial.println(pressDuration);
        if (pressDuration < MAX_PRESS_DURATION) {
          buttonPressed = true;
          buttonHeld = false;
          if (bleSignalReceived) {  
            handleButtonPressWithBLE();
          } else {
            handleButtonPressWithoutBLE();
            buttonHeld = true;
          }
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
          }
          buttonHeld = false;
        }
        buttonPressTime = 0;
      }
    }
  }
  lastButtonState = reading;
}

void handleButtonPressWithoutBLE() {
  Serial.println("Nút nhấn khi không có tín hiệu BLE");
}

void handleButtonPressWithBLE() {
  if (isPlaying) {
    stopRequested = true;
    stopMusic();
  }
}

void playMusic() {
  Serial.println("Choi nhac");
  if (!isPlaying) {
    isPlaying = true;
    myDFPlayer.playFolder(2, 7);  // Phát nhạc từ folder 2, bài 7
    lastPlayTime = millis();
  } else {
    if (millis() - lastPlayTime >= 7000) {
      myDFPlayer.playFolder(2, 7);  // Phát lại nhạc từ folder 2, bài 7
      lastPlayTime = millis();
    }
  }
}

void stopMusic() {
  Serial.println("Dừng phát nhạc...");
  isPlaying = false;
  setLEDColor(strip.Color(0, 0, 0));  // Tắt LED
  Serial.println("Đã dừng phát nhạc");
}

void handlePostStopActions() {
  if (stopRequested) {
    Serial.println("Nút nhấn được nhấn, dừng nhạc trong 30 giây...");

    stopStartTime = millis();
    EEPROM.get(EEPROM_ADDRESS, storedCode);  

    while (millis() - stopStartTime < stopDuration) {  // 30s stop music
      handleButtonPress();                             // Xử lý nút nhấn liên tục

      if (digitalRead(BUTTON_PIN) == LOW) {
        Serial.println("Nút nhấn được nhấn lại, cập nhật thời gian dừng.");
        stopStartTime = millis();  // Reset thời gian dừng nếu nút nhấn lại
      }
    }
    Serial.println("Hết thời gian dừng âm thanh ");
    delay(50);                  // Giảm tải CPU
    bleSignalReceived = false;  // Reset trạng thái tín hiệu BLE nhận được
    buttonPressed = false;      // Reset trạng thái nút nhấn
    stopRequested = false;      // Reset trạng thái yêu cầu dừng âm thanh
  }
}

void checkSignalDuringPlayback() {
  unsigned long currentTime = millis();
  if ((currentTime - lastBleReceivedTime) > 5000) {
    stopMusic();  // Dừng nhạc nếu không nhận được tín hiệu BLE trong vòng 5 giây
  }
}

void sendIRCode(unsigned long code) {
  for (int i = 0; i < 5; i++) {
    Serial.print("Gửi mã IR: ");
    Serial.println(code);
    irsend.sendNEC(code, 32);  // Gửi mã IR sử dụng giao thức NEC
    delay(200);
  }
}

void setLEDColor(uint32_t color) {
  strip.setPixelColor(0, color);  // Đặt màu cho LED
  strip.show();                   // Hiển thị màu
}

bool connectToServer() {
  Serial.print("Đang thiết lập kết nối với ");
  Serial.println(myDevice->getAddress().toString().c_str());

  BLEClient* pClient = BLEDevice::createClient();
  Serial.println(" - Đã tạo client");

  pClient->setClientCallbacks(new MyClientCallback());

  if (!pClient->connect(myDevice)) {
    Serial.println(" - Không thể kết nối với server");
    return false;
  }
  Serial.println(" - Đã kết nối với server");

  pClient->setMTU(517);  // Đặt MTU cho kết nối

  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Không tìm thấy UUID dịch vụ: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Đã tìm thấy dịch vụ");

  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Không tìm thấy UUID đặc tính: ");
    Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Đã tìm thấy đặc tính");

  if (pRemoteCharacteristic->canRead()) {
    String value = pRemoteCharacteristic->readValue().c_str();
    Serial.print("Giá trị đặc tính là: ");
    Serial.println(value);
  }

  if (pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(notifyCallback);
  }

  connected = true;
  return true;
}

void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  if (pBLERemoteCharacteristic == nullptr || pData == nullptr) {
    Serial.println("Lỗi: Đặc tính hoặc dữ liệu nhận được là null");
    return;
  }

  Serial.print("dữ liệu nhận: ");

  if (length == sizeof(unsigned long)) {
    memcpy(&receivedValue, pData, sizeof(receivedValue));
    Serial.println(receivedValue, HEX);
    Serial.println("------------");

    EEPROM.get(EEPROM_ADDRESS, storedCode);
    if (receivedValue == storedCode) {
      Serial.println("Mã nhận được trùng với mã lưu trong EEPROM");
      shouldPlayMusic = true;  
      bleSignalReceived = true;
      lastBleReceivedTime = millis();  
    }
  } else {
    Serial.println("Dữ liệu nhận được không hợp lệ");
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

  for (int i = 0; i < 512; i++) {
    EEPROM.write(i, 0);  // Ghi giá trị 0 vào tất cả các vị trí trong EEPROM
  }
  EEPROM.commit();
  Serial.println("EEPROM đã được reset");

  EEPROM.get(EEPROM_ADDRESS, dataRead);
  Serial.print("Data read from EEPROM after reset: ");
  Serial.println(dataRead);

  digitalWrite(LED_PIN, LOW);
}