#include <IRremoteESP8266.h>
#include <IRsend.h>

const uint16_t kIrLed = 15; // Chân nối tới cathode của LED IR

IRsend irsend(kIrLed);

unsigned long data[] = {0x24DB07F8};
int dataSize = sizeof(data) / sizeof(data[0]);
int currentIndex = 0;

void setup() {
  Serial.begin(115200);
  irsend.begin();
}

void loop() {
  irsend.sendNEC(data[currentIndex], 32); // Gửi dữ liệu IR dùng chuẩn NEC
  Serial.print("Phát dữ liệu: ");
  Serial.println(data[currentIndex], HEX); // In ra mã đang phát
  currentIndex = (currentIndex + 1) % dataSize; // Chuyển đến mã tiếp theo trong mảng
  
  delay(1000); // Chờ 1 giây trước khi phát mã tiếp theo
}



