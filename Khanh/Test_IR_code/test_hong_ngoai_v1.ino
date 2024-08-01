#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>

unsigned long data[] = {0x1AE52BD4, 0x1AE53CC3, 0x1AE54DB2, 0x1AE55EA1, 0x1AE56F90, 0x2AD51BE4, 0x2AD52CD3, 0x2AD53DC2, 0x2AD54EB1};
const uint16_t kRecvPin = 4; // Chân thu IR
IRrecv irrecv(kRecvPin);
decode_results results;
unsigned long lastValidCode = 0;
unsigned long lastReceiveTime = 0;
const unsigned long debounceDelay = 1000; // Thời gian chờ debouncing (ms)

void setup() {
  Serial.begin(115200);
  irrecv.enableIRIn(); // Bắt đầu bộ thu IR
}

bool isCodeInArray(unsigned long code) {
  for (int i = 0; i < sizeof(data) / sizeof(data[0]); i++) {
    if (data[i] == code) {
      return true;
    }
  }
  return false;
}

int calculateSignalStrength(decode_results *results) {
  // Đo cường độ tín hiệu IR dựa trên tổng thời gian của các xung
  int totalDuration = 0;
  for (int i = 0; i < results->rawlen; i++) {
    totalDuration += results->rawbuf[i];
  }
  return totalDuration;
}

void loop() {
  if (irrecv.decode(&results)) {
    unsigned long currentTime = millis();
    if (currentTime - lastReceiveTime > debounceDelay) {
      if (isCodeInArray(results.value)) {
        if (results.value != lastValidCode) {
          Serial.print("Mã nhận được: ");
          serialPrintUint64(results.value, HEX); // In mã nhận được dưới dạng HEX
          Serial.println("");

          int signalStrength = calculateSignalStrength(&results);
          Serial.print("Cường độ tín hiệu: ");
          Serial.println(signalStrength);

          lastValidCode = results.value;
        }
      } else {
        Serial.println("Mã không hợp lệ.");
      }
      lastReceiveTime = currentTime;
    }
    irrecv.resume(); // Chuẩn bị cho dữ liệu tiếp theo
  }
}
