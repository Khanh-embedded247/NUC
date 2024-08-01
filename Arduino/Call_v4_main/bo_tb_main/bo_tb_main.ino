#include <BLEDevice.h>
#include <DFRobotDFPlayerMini.h>
#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>

constexpr uint8_t DFPLAYER_TX_PIN = 21;
constexpr uint8_t DFPLAYER_RX_PIN = 22;
constexpr uint8_t BUTTON_PIN = 15;
constexpr uint8_t LED_PIN = 16;
constexpr uint8_t IR_LED_PIN = 4;
constexpr uint16_t EEPROM_ADDRESS = 0;
constexpr unsigned long MAX_PRESS_DURATION = 2000;
constexpr unsigned long ADD_MODE_HOLD_TIME = 5000;
constexpr unsigned long ADD_MODE_RESET = 10000;

constexpr char SERVICE_UUID[] = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
constexpr char CHARACTERISTIC_UUID[] = "beb5483e-36e1-4688-b7f5-ea07361b26a8";

class LEDManager {
public:
  LEDManager(uint8_t pin)
    : strip(1, pin, NEO_GRB + NEO_KHZ800) {}

  void begin() {
    strip.begin();
    strip.show();
  }

  void setLEDColor(uint32_t color) {
    strip.setPixelColor(0, color);
    strip.show();
  }

private:
  Adafruit_NeoPixel strip;
};

class DFPlayerManager {
public:
  DFPlayerManager(LEDManager& ledManager)
    : ledManager(ledManager), isPlaying(false), lastPlayTime(0) {}

  void connect() {
    Serial2.begin(9600, SERIAL_8N1, DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);
    if (!myDFPlayer.begin(Serial2)) {
      Serial.println("DFPlayer Mini OFFLINE");
      while (true) {
        delay(1000);
      }
    }
    Serial.println("DFPlayer Mini online");
    delay(1000);
    myDFPlayer.volume(30);
    myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);
    myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);
  }

  void playMusic() {
    Serial.println("choi nhac");
    if (!isPlaying) {
      //check
      if (!myDFPlayer.begin(Serial2)) {
        //Serial.println("Failed to communicate with DFPlayer Mini");
        return;
      }
      isPlaying = true;
      myDFPlayer.playFolder(2, 7);
      lastPlayTime = millis();
    }
    Serial.println("thong bao xong ");
  }

  void stopMusic() {
    Serial.println("Stopping music...");
    isPlaying = false;
    myDFPlayer.stop();
    ledManager.setLEDColor(0);
    Serial.println("Music stopped");
  }

  bool getIsPlaying() const {
    return isPlaying;
  }

private:
  DFRobotDFPlayerMini myDFPlayer;
  LEDManager& ledManager;
  bool isPlaying;
  unsigned long lastPlayTime;
};

class IRManager {
public:
  IRManager(uint8_t pin)
    : irsend(pin) {}

  void begin() {
    irsend.begin();
  }

  void sendIRCode(uint32_t code) {
    Serial.print("Sending IR code: ");
    Serial.println(code, HEX);
    irsend.sendNEC(code, 32);
    delay(1000);
  }

  uint32_t createNecCode(uint8_t address, uint8_t command) {
    uint8_t inv_address = ~address;
    uint8_t inv_command = ~command;
    uint32_t code = 0;

    code |= address;
    code <<= 8;
    code |= inv_address;
    code <<= 8;
    code |= command;
    code <<= 8;
    code |= inv_command;

    return code;
  }

private:
  IRsend irsend;
};

class BLEManager {
public:
  BLEManager(DFPlayerManager& dfPlayer)
    : dfPlayer(dfPlayer), pClient(nullptr), pRemoteService(nullptr), pRemoteCharacteristic(nullptr), connected(false), shouldPlayMusic(false), bleSignalReceived(false), lastBleReceivedTime(0) {}

  void begin() {
    BLEDevice::init("Phenikaa-x");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(this));
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
    pBLEScan->start(1, false);
  }

  void manageConnection() {
    if (doConnect) {
      if (connectToServer()) {
        Serial.println("Connected to BLE Server.");
      } else {
        Serial.println("Failed to connect to server; retrying...");
      }
      doConnect = false;
    }

    if (!connected) {
      pBLEScan->start(1);
    }
  }

  void notifyCallback(BLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    if (pRemoteCharacteristic == nullptr || pData == nullptr) {
      Serial.println("Lỗi: Đặc tính hoặc dữ liệu nhận được là null");
      return;
    }
    Serial.println("--------------------------------");
    Serial.print("dữ liệu nhận: ");

    if (length == sizeof(unsigned long)) {
      unsigned long receivedValue = 0;
      memcpy(&receivedValue, pData, sizeof(receivedValue));
      Serial.println(receivedValue, HEX);


      unsigned long storedCode = 0;
      EEPROM.get(EEPROM_ADDRESS, storedCode);
      Serial.print("Mã lưu trong EEPROM: ");
      Serial.println(storedCode, HEX);

      if (receivedValue == storedCode) {
        Serial.println("Mã nhận được trùng với mã lưu trong EEPROM");
        lastBleReceivedTime = millis();
        shouldPlayMusic = true;
        Serial.println(shouldPlayMusic);
        bleSignalReceived = true;
        Serial.println("*****************************");
      }
    } else {
      Serial.println("Dữ liệu nhận được không hợp lệ");
    }
  }

  bool getBleSignalReceived() const {
    return bleSignalReceived;
  }

  bool getShouldPlayMusic() const {
    return shouldPlayMusic;
  }

  void setShouldPlayMusic(bool value) {
    shouldPlayMusic = value;
  }

  unsigned long getLastBleReceivedTime() const {
    return lastBleReceivedTime;
  }

private:
  DFPlayerManager& dfPlayer;
  BLEClient* pClient;
  BLERemoteService* pRemoteService;
  BLERemoteCharacteristic* pRemoteCharacteristic;
  bool connected;
  BLEAdvertisedDevice* myDevice;
  BLEScan* pBLEScan;
  bool doConnect;
  unsigned long lastBleReceivedTime;
  bool shouldPlayMusic;
  bool bleSignalReceived;

  bool connectToServer() {
    if (myDevice == nullptr) {
      Serial.println("myDevice is null");
      return false;
    }

    Serial.print("Đang thiết lập kết nối với ");
    Serial.println(myDevice->getAddress().toString().c_str());

    pClient = BLEDevice::createClient();
    if (pClient == nullptr) {
      Serial.println("Failed to create BLE client");
      return false;
    }

    Serial.println(" - Đã tạo client");

    pClient->setClientCallbacks(new MyClientCallback(this));

    if (!pClient->connect(myDevice)) {
      Serial.println(" - Không thể kết nối với server");
      return false;
    }
    Serial.println(" - Đã kết nối với server");

    pClient->setMTU(517);

    pRemoteService = pClient->getService(BLEUUID(SERVICE_UUID));
    if (pRemoteService == nullptr) {
      Serial.print("Không tìm thấy UUID dịch vụ: ");
      Serial.println(SERVICE_UUID);
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Đã tìm thấy dịch vụ");

    pRemoteCharacteristic = pRemoteService->getCharacteristic(BLEUUID(CHARACTERISTIC_UUID));
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Không tìm thấy UUID đặc tính: ");
      Serial.println(CHARACTERISTIC_UUID);
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
      pRemoteCharacteristic->registerForNotify([this](BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
        this->notifyCallback(pChar, pData, length, isNotify);
      });
    }

    connected = true;
    return true;
  }

  class MyClientCallback : public BLEClientCallbacks {
  public:
    MyClientCallback(BLEManager* manager)
      : manager(manager) {}

    void onConnect(BLEClient* pclient) override {
      manager->connected = true;
      Serial.println("Connected to BLE Server");
    }

    void onDisconnect(BLEClient* pclient) override {
      manager->connected = false;
      Serial.println("Disconnected from BLE Server, trying to reconnect...");
      manager->doConnect = true;
    }

  private:
    BLEManager* manager;
  };

  class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  public:
    MyAdvertisedDeviceCallbacks(BLEManager* manager)
      : manager(manager) {}

    void onResult(BLEAdvertisedDevice advertisedDevice) override {
      Serial.print("Thiết bị BLE được tìm thấy: ");
      Serial.println(advertisedDevice.toString().c_str());

      if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(BLEUUID(SERVICE_UUID))) {
        if (advertisedDevice.getName() == "Robot H1") {
          BLEDevice::getScan()->stop();
          manager->myDevice = new BLEAdvertisedDevice(advertisedDevice);
          manager->doConnect = true;
        }
      }
    }

  private:
    BLEManager* manager;
  };
};

class ButtonManager {
public:
  ButtonManager(uint8_t pin, DFPlayerManager& dfPlayer, BLEManager& bleManager)
    : pin(pin), dfPlayer(dfPlayer), bleManager(bleManager), lastButtonState(HIGH), buttonState(HIGH), buttonPressTime(0), lastDebounceTime(0), buttonHeld(false), inAddMode(false), addModeStartTime(0), resetEEPROM(false), stopRequested(false), buttonPressed(false) {}

  void begin() {
    pinMode(pin, INPUT_PULLUP);
    lastButtonState = digitalRead(pin);
    buttonState = lastButtonState;
  }

  void handleButtonPress() {
    unsigned long currentTime = millis();
    int reading = digitalRead(pin);

    if (reading != lastButtonState) {
      lastDebounceTime = currentTime;
    }

    if ((currentTime - lastDebounceTime) > debounceDelay) {
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
            buttonPressed = true;
            buttonHeld = false;
            if (bleManager.getBleSignalReceived()) {
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

  bool isInAddMode() const {
    return inAddMode;
  }

  void setInAddMode(bool value) {
    inAddMode = value;
  }

  unsigned long getAddModeStartTime() const {
    return addModeStartTime;
  }

  void setAddModeStartTime(unsigned long time) {
    addModeStartTime = time;
  }

  bool isResetEEPROM() const {
    return resetEEPROM;
  }

  void setResetEEPROM(bool value) {
    resetEEPROM = value;
  }

  bool isStopRequested() const {
    //Serial.println(stopRequested);
    return stopRequested;
  }

  void setStopRequested(bool value) {
    stopRequested = value;
  }

  bool isButtonPressed() const {
    return buttonPressed;
  }

  void setButtonPressed(bool value) {
    buttonPressed = value;
  }

private:
  uint8_t pin;
  int lastButtonState;
  int buttonState;
  unsigned long buttonPressTime;
  unsigned long lastDebounceTime;
  DFPlayerManager& dfPlayer;
  BLEManager& bleManager;
  bool buttonHeld;
  bool inAddMode;
  unsigned long addModeStartTime;
  bool resetEEPROM;
  bool stopRequested;
  bool buttonPressed;
  const unsigned long debounceDelay = 50;

  void handleButtonPressWithBLE() {
    if (dfPlayer.getIsPlaying()) {
      stopRequested = true;
      Serial.println("Button pressed, requesting stop.");
      dfPlayer.stopMusic();
    }
  }

  void handleButtonPressWithoutBLE() {
    Serial.println("Button pressed without BLE signal");
  }
};

class EEPROMManager {
public:
  EEPROMManager(LEDManager& ledManager)
    : ledManager(ledManager) {}

  void begin() {
    EEPROM.begin(512);
    EEPROM.get(EEPROM_ADDRESS, dataRead);
    Serial.print("Data read from EEPROM: ");
    Serial.println(dataRead, HEX);
  }

  void saveToEEPROM(unsigned long data) {
    EEPROM.put(EEPROM_ADDRESS, data);
    EEPROM.commit();
    Serial.println("Saved to EEPROM");
  }

  void resetEEPROM() {
    for (int i = 0; i < 512; i++) {
      EEPROM.write(i, 0);
    }
    EEPROM.commit();
    Serial.println("EEPROM has been reset");

    EEPROM.get(EEPROM_ADDRESS, dataRead);
    Serial.print("Data read from EEPROM after reset: ");
    Serial.println(dataRead);
  }

private:
  unsigned long dataRead;
  LEDManager& ledManager;
};

class DeviceManager {
public:
  DeviceManager()
    : dfPlayer(ledManager), ledManager(LED_PIN), irManager(IR_LED_PIN), bleManager(dfPlayer), buttonManager(BUTTON_PIN, dfPlayer, bleManager), eepromManager(ledManager) {}

  void begin() {
    //dfPlayer.connect();
    ledManager.begin();
    eepromManager.begin();
    bleManager.begin();
    buttonManager.begin();
    irManager.begin();
  }

  void loop() {
    buttonManager.handleButtonPress();
    handleResetEEPROM();
    handleAddMode();
    bleManager.manageConnection();
    Serial.println("kkkkkkkkkkkkk");
    Serial.println(bleManager.getShouldPlayMusic());
    if (bleManager.getShouldPlayMusic()) {
      bleManager.setShouldPlayMusic(false);
      dfPlayer.playMusic();
    }
    handlePostStopActions();
    Serial.println(dfPlayer.getIsPlaying());
    if (dfPlayer.getIsPlaying()) {
      checkSignalDuringPlayback();
    }
    // Serial.println("hhh");
    // Serial.println(buttonManager.isStopRequested());
    
  }

private:
  DFPlayerManager dfPlayer;
  LEDManager ledManager;
  EEPROMManager eepromManager;
  IRManager irManager;
  BLEManager bleManager;
  ButtonManager buttonManager;

  void handleResetEEPROM() {
    if (buttonManager.isResetEEPROM()) {
      eepromManager.resetEEPROM();
      buttonManager.setResetEEPROM(false);
    }
  }

  void handleAddMode() {
    if (buttonManager.isInAddMode()) {
      unsigned long currentTime = millis();
      if (currentTime - buttonManager.getAddModeStartTime() <= 5000) {
        unsigned long storedCode = 0;
        EEPROM.get(EEPROM_ADDRESS, storedCode);
        if (storedCode == 0xFFFFFFFF || storedCode == 0) {
          uint8_t address = random(0x00, 0xFF);
          uint8_t command = random(0x00, 0xFF);
          uint32_t necCode = irManager.createNecCode(address, command);
          Serial.print("Mã NEC tạo ra : ");
          Serial.println(necCode, HEX);
          eepromManager.saveToEEPROM(necCode);
          irManager.sendIRCode(necCode);
        } else {
          Serial.println("EEPROM already contains data. Not generating new code.");
        }
      } else {
        buttonManager.setInAddMode(false);
        Serial.println("Exiting ADD mode");
      }
    }
  }

  void checkSignalDuringPlayback() {
    unsigned long currentTime = millis();
    if ((currentTime - bleManager.getLastBleReceivedTime()) > 5000) {
      dfPlayer.stopMusic();
      Serial.println("Exiting thong bao mode");
    }
  }

  void handlePostStopActions() {
    Serial.println(buttonManager.isStopRequested());
    if (buttonManager.isStopRequested()) {
      Serial.println("Nút nhấn được nhấn, dừng nhạc trong 30 giây...");

      unsigned long stopStartTime = millis();
      unsigned long storedCode = 0;
      EEPROM.get(EEPROM_ADDRESS, storedCode);

      while (millis() - stopStartTime < 30000) {
        buttonManager.handleButtonPress();

        if (digitalRead(BUTTON_PIN) == LOW) {
          Serial.println("Button pressed again, updating stop time.");
          stopStartTime = millis();
        }
      }
      Serial.println("End of stop duration");
      delay(50);
      buttonManager.setButtonPressed(false);
      buttonManager.setStopRequested(false);
    }
  }
};

// Global instance of DeviceManager
DeviceManager deviceManager;

void setup() {
  Serial.begin(115200);
  deviceManager.begin();
}

void loop() {
  deviceManager.loop();
}
