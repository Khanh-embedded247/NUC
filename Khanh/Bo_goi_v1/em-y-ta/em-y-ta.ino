#include <WebServer.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <esp_sleep.h>
#include "DFRobotDFPlayerMini.h"

DFRobotDFPlayerMini DFPlayer_247;

#define RX_PIN 22
#define TX_PIN 21
#define USER_ID "tittit"
#define StopButton 15

IPAddress staticIP( 100,100,0,247);
IPAddress gateway(192, 168, 1, 1);   // Replace this with your gateway IP Addess
IPAddress subnet(255, 255, 0, 0);  // Replace this with your Subnet Mask
IPAddress dns(192, 168, 1, 1); 
//state robot
bool Y4 = false;
bool PK = false;
bool VC = false;
bool TK = false;
bool stopSoundFlag = false;
bool bell = false;

const char* ssid = "abc";
const char* password = "keke123@";

const size_t maxJsonSize = 256;
WebServer server(80);
StaticJsonDocument<256> jsonDocument;
char buffer[512];

void setup() {
  pinMode(StopButton, INPUT);  
  // attachInterrupt(digitalPinToInterrupt(StopButton), stopSound, FALLING);
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); 
  Serial.begin(115200);

  Serial.println();
  Serial.println("DFRobot DFPlayer Mini in Hospital");
  Serial.println("Initializing DFPlayer ... (May take 3~5 seconds)");

  //Use hardwareSerial communicate with DFPlayer
  if (!DFPlayer_247.begin(Serial2)) {
    Serial.println("Unable to begin:");
    Serial.println("1.Please recheck the connection!");
    Serial.println("2.Please insert the SD card");
    while (true);
      
  }
  Serial.println("DFPlayer Mini online");
  delay(1000);
   // Đặt âm lượng
  DFPlayer_247.volume(30); // Đặt âm lượng ở mức 20 (từ 0 đến 30)

  //Connect wifi
   if (WiFi.config(staticIP, gateway, subnet, dns, dns) == false) {
   Serial.println("Configuration failed.");
 }
  WiFi.begin(ssid, password);
  WiFi.mode(WIFI_STA);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Đang kết nối WiFi...");
  }
  Serial.println("Đã kết nối WiFi");
  Serial.println(WiFi.localIP());
  setupApi();
  //Safe energy
  esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_ON);
  esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
}

void handlePost() {
  // plain=raw data (JSON) trong POST
  if (server.hasArg("plain") == false) {
  // Serial.println("Lỗi: Không có dữ liệu JSON.");
    server.send(400, "application/json", "{}");
    return;
  }
  //Nếu lớn hơn bộ chứa api
  String body = server.arg("plain");
  if (body.length() > maxJsonSize) {
    server.send(400, "application/json", "{}");
    return;
  }
  body.toCharArray(buffer, sizeof(buffer));

  Serial.println("Thông tin JSON:");
  Serial.println(body);
  DeserializationError error = deserializeJson(jsonDocument, buffer);
  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    server.send(400, "application/json", "{}");
    return;
  }

  String name = jsonDocument["robot_name"].as<String>();
  int state = jsonDocument["state"].as<int>();
  String name_pose = jsonDocument["name_pose"].as<String>();

  //state TRUOWNG HOP
  if (name.equals(USER_ID) & state == 10) {
    if (name_pose.equals("Y4")) {
      Y4 = true;
      DFPlayer_247.play(1);  //ROBOT YEU CAU LAY DO
      stopSoundFlag = false;
      while (Y4) {
        if (stopSoundFlag) {
          DFPlayer_247.stop();
          Y4 = false;
          break;
        }
        if (!name_pose.equals("Y4")) {
          Y4 = false;
          return;
        }
      }
    } 
    else if (name_pose.equals("PK")) {
      PK = true;
      for (int i = 0; i < 3; i++) {
        DFPlayer_247.play(2); // Robot ddax quay veef phongf khams
        delay(500); // 
      }
      
      while (PK) {
        if (stopSoundFlag) {
          DFPlayer_247.stop();
          PK = false;
          break;
        }
        if (!name_pose.equals("PK")) {
          PK = false;
          return;
        }
      }
    }
  // Phản hồi với JSON trạng thái
    createJson(USER_ID, bell);
    server.send(200, "application/json", buffer);
  } else {
    DFPlayer_247.stop();
    Serial.println("Lỗi: Trường  không hợp lệ.");
    server.send(400, "application/json", "{}");
  }
}

void createJson(const char* name, bool state) {
  jsonDocument.clear();
  jsonDocument[name] = state;
  serializeJson(jsonDocument, buffer, sizeof(buffer));
}

void getstate() {
  createJson(USER_ID, bell);
  server.send(200, "application/json", buffer);
}

void setupApi() {
  server.on("/phenet_tittit", HTTP_POST, handlePost);
  server.on("/state", HTTP_GET, getstate);
  server.begin();
}

void loop() {
  server.handleClient();
  stopSound();
}

void stopSound() {
  if(digitalRead(StopButton)==HIGH)
  Serial.println("Đã nhấn nút dừng.");
  // Khi nhấn nút dừng, đặt cờ dừng âm thanh thành true
  stopSoundFlag = true;
}
/*
 //----Mp3 play----
  myDFPlayer.next();  //Play next mp3
  myDFPlayer.previous();  //Play previous mp3
  myDFPlayer.play(1);  //Play the first mp3
  myDFPlayer.loop(1);  //Loop the first mp3
  myDFPlayer.pause();  //pause the mp3
  myDFPlayer.start();  //start the mp3 from the pause
  myDFPlayer.playFolder(15, 4);  //play specific mp3 in SD:/15/004.mp3; Folder Name(1~99); File Name(1~255)
  myDFPlayer.enableLoopAll(); //loop all mp3 files.
  myDFPlayer.disableLoopAll(); //stop loop all mp3 files.
  myDFPlayer.playMp3Folder(4); //play specific mp3 in SD:/MP3/0004.mp3; File Name(0~65535)
  myDFPlayer.advertise(3); //advertise specific mp3 in SD:/ADVERT/0003.mp3; File Name(0~65535)
  myDFPlayer.stopAdvertise(); //stop advertise
  myDFPlayer.playLargeFolder(2, 999); //play specific mp3 in SD:/02/004.mp3; Folder Name(1~10); File Name(1~1000)
  myDFPlayer.loopFolder(5); //loop all mp3 files in folder SD:/05.
  myDFPlayer.randomAll(); //Random play all the mp3.
  myDFPlayer.enableLoop(); //enable loop.
  myDFPlayer.disableLoop(); //disable loop.
 
*/
