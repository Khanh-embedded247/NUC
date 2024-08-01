#include <WebServer.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <esp_sleep.h>
#include "DFRobotDFPlayerMini.h"
#include <mbedtls/md.h>
#include <mbedtls/sha256.h>
#include <mbedtls/base64.h>
#include <time.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
DFRobotDFPlayerMini DFPlayer_247;

#define DFPLAYER_TX_PIN 21
#define DFPLAYER_RX_PIN 22
#define AREA "12a_22_5_e583"
//#define AREA "12a_v1_508f"
#define StopButton 15
#define TIME_STOP 10000
const int buttonPin = 15;             // Chân GPIO mà nút nhấn được kết nối
volatile bool buttonPressed = false;  // Biến cờ để theo dõi trạng thái của nút nhấn
unsigned long lastDebounceTime = 0;   // Thời gian cuối cùng nút nhấn được nhấn
unsigned long debounceDelay = 100;    // Thời gian debounce (miliseconds)
//unsigned long lastButtonCheckTime = 0;   // Thời gian cuối cùng kiểm tra trạng thái nút nhấn
//unsigned long buttonCheckInterval = 10;  // Khoảng thời gian giữa các lần kiểm tra trạng thái nút nhấn
unsigned long buttonPressStartTime = 0;

bool keu = false;
bool nhan = false;
bool connect_wifi = false;
bool connect_dfplayer = false;
bool check_network = false;
bool executedAfter10Seconds = false;
//bool playedPKSound = false;
bool first_nofi = false;
bool soundPlayed = false;
bool first_disconnect = true;
bool connecting_wifi = false;
bool audioPlaying = false;

unsigned long startTime;
unsigned long lastSoundTime = 0;  // Thời gian cuối cùng kêu
unsigned long programStartTime = 0;
//KHAI BAO WIFI

// const char* ssid = "Dang Thi Hoai";
// const char* password = "Etzetkhong9";
const char* ssid = "abc";
const char* password = "keke123@";

//Khoi tao realtime-gmt
const long gmtOffset_sec = 0;         // London nằm ở múi giờ 0
const int daylightOffset_sec = 3600;  // London áp dụng daylight saving time

String currentSiteId;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", gmtOffset_sec);
String my_bind_code = "";
String _signature = "";
String _date = "";
String _token = "";
String _mapID = "";
String _siteIds;


String language = "Language(zh-rCN:Chinese  en-rUS:English   ko-rKR:Korean      JA-rJP:Japanese zh-rTW:Traditional)";

const char* AK = "03DA2635A";
const char* SK = "e6e2bb261cc04440bb19dc33f34eed49";
// const char* AK = "03DA2635A";
// const char* SK = "e6e2bb261cc04440bb19dc33f34eed49";
const char* HTTP_METHOD = "GET";
const char* REQUEST_URI = "/user";

//const size_t maxJsonSize = 256;
WebServer server(9000);
StaticJsonDocument<1024> jsonDocument;
//Khoi tao dung luong chua du lieu
char buffer[1024];
JsonObject receivedData;

void IRAM_ATTR handleInterrupt() {
  unsigned long currentTime = millis();
  // Chỉ xử lý nếu đã đủ thời gian debounce
  if (currentTime - lastDebounceTime > debounceDelay) {
    buttonPressed = true;
    nhan = true;
    buttonPressStartTime = currentTime;  // Lưu thời gian khi nút được nhấn
    lastDebounceTime = currentTime;
  }
}

bool initializationDone = false;
void setup() {
  // Button
  pinMode(StopButton, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(StopButton), handleInterrupt, FALLING);
  startTime = -TIME_STOP - 1;
  buttonPressed = false;

  Serial.begin(115200);
  programStartTime = millis();
  DFPlayer_connect();

  WiFi_connect_wifi();
  // Đồng bộ thời gian từ máy chủ NTP
  timeClient.begin();
  timeClient.setTimeOffset(gmtOffset_sec);
  if (!initializationDone) {
    // Gọi các hàm chỉ một lần
    initializationDone = true;
    _signature = print_signature();
    _date = getFormattedDate();
    my_bind_code = perform_call_bind_code();
    _token = get_token();
    _mapID = get_mapID();
    _siteIds = get_siteIds();
  }

  setupApi();
  // esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_ON);
  // esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
}

void handlePost() {
}
void WiFi_connect_wifi() {
  WiFi.begin(ssid, password);
  unsigned long current_status = millis();
  while (millis() - programStartTime < 7000) {
    if (WiFi.status() != WL_CONNECTED) {
      connect_wifi = false;
      Serial.println("WiFi connect false");
      WiFi.reconnect();
      delay(2000);
    } else {
      connect_wifi = true;
      Serial.println("WiFi connect success");
      // DFPlayer_247.play(3);
      delay(1000);
      Serial.println(WiFi.localIP());
      return;
    }
  }
}

void DFPlayer_connect() {
    Serial2.begin(9600, SERIAL_8N1, DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);
    if (!DFPlayer_247.begin(Serial2)) {
        delay(1000);  // Đợi một chút lâu hơn
        if (!DFPlayer_247.begin(Serial2)) {
            Serial.println("DFPlayer Mini OFFLINE");
            while (true);  // Loop here to indicate failure
        }
    }
    connect_dfplayer = true;
    Serial.println("DFPlayer Mini online");
    delay(1000);  // Delay thêm sau khi kết nối thành công
    DFPlayer_247.volume(30);
}

String getFormattedDate() {
  // Lấy thời gian từ máy chủ NTP
  timeClient.update();

  // Lấy thời gian hiện tại
  time_t rawTime = timeClient.getEpochTime();

  // Chuyển đổi thời gian sang cấu trúc tm
  struct tm* timeinfo;
  timeinfo = gmtime(&rawTime);

  // Khai báo một mảng để lưu trữ chuỗi định dạng
  char formattedDate[32];

  // Định dạng chuỗi theo yêu cầu Mon, 13 May 2024 11:09:06 GMT
  strftime(formattedDate, sizeof(formattedDate), "%a, %d %b %Y %H:%M:%S GMT", timeinfo);

  // Trả về chuỗi đã định dạng
  return String(formattedDate);
}

String genSignature(const char* method, const char* requestURI, const char* ak, const char* sk) {
  String signData = String(method) + " " + String(requestURI) + "\\n" + _date;

  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
  mbedtls_md_hmac_starts(&ctx, (const unsigned char*)sk, strlen(sk));
  mbedtls_md_hmac_update(&ctx, (const unsigned char*)signData.c_str(), signData.length());
  unsigned char signatureBytes[32];  // SHA256 has 32-byte digest size
  mbedtls_md_hmac_finish(&ctx, signatureBytes);
  mbedtls_md_free(&ctx);

  size_t outputLength;
  char encodedSignature[64];  // Base64 encoding of SHA256 digest needs at most 64 bytes
  mbedtls_base64_encode((unsigned char*)encodedSignature, sizeof(encodedSignature), &outputLength, signatureBytes, 32);

  return String(encodedSignature);
}

String print_signature() {
  String formattedDate = getFormattedDate();
  String signature = genSignature(HTTP_METHOD, REQUEST_URI, AK, SK);
  String Authorization = "SEGWAY " + String(AK) + ":" + signature;
  Serial.println(formattedDate);
  Serial.println(Authorization);
  return Authorization;
}

bool call_bind_code = false;
String receivedData_bindCode;
String receivedData_token;
//tim bindcode
String perform_call_bind_code() {
  String bind_code;
  HTTPClient http;
  http.begin("https://restaurant-oregon.loomo.com/api/s1/s/call/bind/code?count=1");

  http.addHeader("Date", _date);
  http.addHeader("Authorization", _signature);

  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    DeserializationError error = deserializeJson(jsonDocument, payload);
    if (!error) {
      call_bind_code = true;
      bind_code = jsonDocument["data"][0].as<String>();
      Serial.print("bind_code: ");
      Serial.println(bind_code);
    } else {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
    }
  } else {
    Serial.print("HTTP request failed with error code: ");
    Serial.println(httpCode);
  }
  http.end();
  return bind_code;
}

bool get_token_success = false;
String get_token() {
  if (call_bind_code == true) {
    call_bind_code = false;
    HTTPClient http;
    String bindCode = "https://restaurant-oregon.loomo.com/api/s1/s/call/shop/bind?bindCode=" + my_bind_code;

    http.begin(bindCode);
    http.addHeader("Date", _date);
    http.addHeader("Authorization", _signature);
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      DeserializationError error = deserializeJson(jsonDocument, payload);
      if (!error) {
        get_token_success = true;
        JsonObject received_token = jsonDocument["data"];
        String token = received_token["token"].as<String>();
        receivedData_token = token;
        Serial.print("token: ");
        Serial.println(receivedData_token);
      } else {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        server.send(400, "application/json", "{}");
      }
    } else {
      Serial.print("HTTP request failed with error code: ");
      Serial.println(httpCode);
      server.send(500, "application/json", "{}");
    }
    http.end();
    return receivedData_token;
  }
}
bool get_mapId_success = false;
String mapID;

String get_mapID() {
  if (get_token_success == true) {
    get_token_success = false;
    HTTPClient http;
    String bindCode = "https://restaurant-oregon.loomo.com/api/s1/s/call/shop/maps";

    http.begin(bindCode);
    http.addHeader("token", _token);
    http.addHeader("language", language);
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      DeserializationError error = deserializeJson(jsonDocument, payload);
      if (!error) {
        get_mapId_success = true;
        JsonArray data = jsonDocument["data"].as<JsonArray>();
        for (JsonObject map : data) {
          if (map["mapName"].as<String>() == AREA) {
            String mapID = map["mapId"].as<String>();
            Serial.print("mapID: ");
            Serial.println(mapID);
            http.end();
            return mapID;
          }
        }
        Serial.println("Không tìm thấy mapName phù hợp.");
        server.send(400, "application/json", "{}");
        return "";
      } else {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        server.send(400, "application/json", "{}");
        return "";
      }
    } else {
      Serial.print("HTTP request failed with error code: ");
      Serial.println(httpCode);
      server.send(500, "application/json", "{}");
      return "";
    }
    http.end();
  }
  return mapID;
}

bool get_sitesId_success = false;
bool go_go = false;
String get_siteIds() {
  String siteIdList = "";
  if (get_mapId_success == true) {
    HTTPClient http;
    String bindCode = "https://restaurant-oregon.loomo.com/s1/api/c/call/map/sites?mapId=" + _mapID;

    http.begin(bindCode);
    http.addHeader("token", _token);
    http.addHeader("Authorization", _signature);
    http.addHeader("Date", _date);
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      DeserializationError error = deserializeJson(jsonDocument, payload);
      if (!error) {
        get_sitesId_success = true;
        JsonArray sitesArray = jsonDocument["data"].as<JsonArray>();

        // Duyệt qua từng phần tử trong mảng data
        for (JsonObject site : sitesArray) {
          String siteId = site["siteId"].as<String>();
          // Lưu danh sách siteId vào siteIdList
          siteIdList += siteId + ",";
        }
      } else {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        server.send(400, "application/json", "{}");
        return "";
      }
    } else {
      Serial.print("HTTP request failed with error code: ");
      Serial.println(httpCode);
      server.send(500, "application/json", "{}");
      return "";
    }

    http.end();
  }
  go_go = true;
  Serial.println("SiteId List: ");
  Serial.println(siteIdList);  // In ra danh sách các siteId
  return siteIdList;
}

bool pkCallPlaying = false;
bool go_to_1 = false;
bool pre = false;
void getRobotState(String currentSiteId) {
  // Serial.println("-----------getRobotState-----------");

  if (go_go == true) {
    static bool printedPaths = false;  // Biến tĩnh để kiểm tra xem các đường dẫn đã được in ra chưa
    static String siteIdArray[8];      // Biến tĩnh để lưu trữ danh sách siteId
    static int siteCount = 0;          // Biến tĩnh để lưu trữ số lượng siteId
    static String siteIds = get_siteIds();
    //static bool finished_1_to_2 = false;  // Biến tĩnh để lưu trữ trạng thái finish của b2
    //static bool isB1Calling = false;      // Biến tĩnh để lưu trữ trạng thái kêu của b2
    // Serial.println("Site IDs: ");
    // Serial.println(siteIds);
    // Biến đổi danh sách siteIds thành một mảng


    if (!printedPaths) {
      int index = 0;
      int lastIndex = 0;
      while ((index = siteIds.indexOf(',', index)) != -1 && siteCount < 8) {
        siteIdArray[siteCount] = siteIds.substring(lastIndex, index);
        lastIndex = index + 1;  // Di chuyển lastIndex đến vị trí ký tự sau dấu phẩy
        siteCount++;
        index++;
      }
      // Xử lý phần tử cuối cùng trong trường hợp không có dấu phẩy ở cuối
      if (siteCount < 5) {
        siteIdArray[siteCount] = siteIds.substring(lastIndex);
        siteCount++;
      }
      // Duyệt qua từng phần tử trong mảng và tạo đường dẫn hoàn chỉnh
      for (int i = 0; i < siteCount; i++) {
        String currentSiteId = siteIdArray[i];
        String bindCode = "https://restaurant-oregon.loomo.com/s1/api/c/call/task/detail?siteId=" + currentSiteId;

        // In ra đường dẫn đã tạo để kiểm tra
        Serial.print("Đường dẫn ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(bindCode);
      }
      printedPaths = true;
    }
    // get all urrl form siteIds
    for (int i = 0; i < siteCount; i++) {
      String currentSiteId = siteIdArray[i];
      String bindCode = "https://restaurant-oregon.loomo.com/s1/api/c/call/task/detail?siteId=" + currentSiteId;

      // Thực hiện kết nối HTTP
      HTTPClient http;
      http.begin(bindCode);
      //http.addHeader("token", "b97793b0528f4194beabd2a5836cb85b");
      http.addHeader("token", "4aa81170b95040308dc3f49454c8605b");
      //http.addHeader("token", "3e6e4eb1e1b04370a0fccd86c4a3ef93");
      http.addHeader("language", "en-US");

      int httpCode = http.GET();

      if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        DeserializationError error = deserializeJson(jsonDocument, payload);
        if (!error) {
          JsonObject data = jsonDocument["data"];
          int resultCode = jsonDocument["resultCode"].as<int>();
          String siteName = data["siteName"].as<String>();
          int finished = data["finished"].as<int>();
          int status = data["status"].as<int>();

          //Nếu giao task thành công và đang trên đường tới PK
          if (status == 2) {
            Serial.print("First : ");
            if (siteName.equals("Pickup") && finished == 1) {
              pre = true;
            }
          }
          Serial.println("Giao task success:");
          Serial.print(pre);

          //Lúc gọi robot tới PK thif keeu
          if (siteName.equals("Pickup") && finished == 2 && pre == true) {
            Serial.println("Arrived");
            keu = true;
            audioPlaying = true;
            // go_to_1 = true;
          }
          //Luc dang keu PK ma task khac di chuyn thi im
          // if (audioPlaying && go_to_1 == true) {
          if (audioPlaying == true && keu == true) {
            if (resultCode == 9000) {
              if (siteName != "Pickup" || finished != 2) {
                Serial.println("Im");
                DFPlayer_247.stop();
                keu = false;
                audioPlaying = false;
                // go_to_1 = true;
                pre = false;
              }
            }
          }
          Serial.println("Am thanh dang keu : ");
          Serial.print(audioPlaying);
          //Serial.println(go_to_1);
          if (keu) {
            Serial.println("Đang kêu");
            handlePKState();
          }
        } else {
          Serial.print("deserializeJson() failed: ");
          Serial.println(error.c_str());
          server.send(400, "application/json", "{}");
          return;  // Trả về nếu deserializeJson() thất bại
        }
      } else {
        Serial.print("HTTP request failed with error code: ");
        Serial.println(httpCode);
        server.send(500, "application/json", "{}");
        return;  // Trả về nếu HTTP request thất bại
      }

      http.end();  // Kết thúc kết nối HTTP sau khi hoàn thành công việc
    }
  }
}

void handlePKState() {
  for (int i = 0; i < 3; i++) {
    DFPlayer_247.playFolder(2, 8);  ////ROBOT VE PHONG KHAM
    delay(4000);
  }
  Serial.println("ROBOT DA DEN PK");
}

void createJson(const char* name, bool mapName) {
  jsonDocument.clear();
  jsonDocument[name] = mapName;
  serializeJson(jsonDocument, buffer, sizeof(buffer));
}

void setupApi() {
  server.on("/api/s1/s/call/task/detail", HTTP_POST, handlePost);
  server.begin();
}

void loop() {

  start_up();
  // if (connect_dfplayer == false) {
  //   DFPlayer_connect();
  // }
  // if (connect_wifi == false) {
  //   WiFi_connect_wifi();
  // }
  //First time: connect success
  if (soundPlayed == true && first_disconnect == false) {

    if (WiFi.status() != WL_CONNECTED) {  //then disconnect
      if (check_network == false) {
        DFPlayer_247.playFolder(2, 3);
        delay(3000);
        check_network = true;
      }
      Serial.print("try connect_wifi ... ");
      WiFi.begin(ssid, password);
      delay(2000);
    } else {
      if (check_network == true) {
        DFPlayer_247.playFolder(2, 4);
        delay(3000);
        Serial.print("connect wifi success ");
        check_network = false;
      }

      server.handleClient();

      getRobotState(currentSiteId);
      stopSound();
    }
  }
  //Firts time:DIsconnect
  if (soundPlayed == true && first_disconnect == true) {
    Serial.print("Trying to connect to WiFi... ");
    WiFi.begin(ssid, password);
    delay(2000);
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi connected successfully.");
      if (!connecting_wifi) {
        DFPlayer_247.playFolder(2, 4);  // Play success sound
        connecting_wifi = true;
      }
      server.handleClient();
      getRobotState(currentSiteId);
      stopSound();
    }

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi connection lost. Reconnecting...");
      if (connecting_wifi) {
        DFPlayer_247.playFolder(2, 3);  // Play reconnecting sound
        connecting_wifi = false;
      }
      WiFi.begin(ssid, password);
      delay(2000);
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi reconnected successfully.");
        DFPlayer_247.playFolder(2, 4);  // Play success sound after reconnection
        connecting_wifi = true;
      }
    }
  }
}
void start_up() {
  if (executedAfter10Seconds == false) {
    unsigned long currentMillis = millis();
    if (millis() - programStartTime >= 7000) {
      if (!soundPlayed) {
        if (connect_wifi == true && connect_dfplayer == true) {
          DFPlayer_247.playFolder(2, 5);  // sua
          delay(2000);
          first_disconnect = false;
        } else {
          DFPlayer_247.playFolder(2, 6);
          delay(3000);
          first_disconnect = true;
          connect_wifi = false;
        }
        soundPlayed = true;
      }
      first_nofi = true;
      executedAfter10Seconds = true;
    }
  }
}
void stopSound() {
  if (buttonPressed) {
    if (digitalRead(buttonPin) == LOW) {
      Serial.println("Button pressed!");
      DFPlayer_247.stop();
      buttonPressed = false;
    }
  }
}