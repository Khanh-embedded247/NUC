#include <Arduino.h>
#include <mbedtls/md.h>
#include <mbedtls/sha256.h>
#include <mbedtls/base64.h>
#include <time.h>

const char* AK = "03DA2635A";
const char* SK = "e6e2bb261cc04440bb19dc33f34eed49";
const char* HTTP_METHOD = "GET";
const char* REQUEST_URI = "/api/s1/s/call/bind/code";

String getFormattedDate() {
  char formattedDate[32];
  time_t now;
  struct tm *timeinfo;
  now = time(nullptr);
  timeinfo = gmtime(&now);
  strftime(formattedDate, sizeof(formattedDate), "%a, %d %b %Y %H:%M:%S GMT", timeinfo);
  return String(formattedDate);
}

String genSignature(const char* method, const char* requestURI, const char* ak, const char* sk) {
  String signData = String(method) + " " + String(requestURI) + "\\n" + getFormattedDate();
  
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
  mbedtls_md_hmac_starts(&ctx, (const unsigned char*)sk, strlen(sk));
  mbedtls_md_hmac_update(&ctx, (const unsigned char*)signData.c_str(), signData.length());
  unsigned char signatureBytes[32]; // SHA256 has 32-byte digest size
  mbedtls_md_hmac_finish(&ctx, signatureBytes);
  mbedtls_md_free(&ctx);
  
  size_t outputLength;
  char encodedSignature[64]; // Base64 encoding of SHA256 digest needs at most 64 bytes
  mbedtls_base64_encode((unsigned char*)encodedSignature, sizeof(encodedSignature), &outputLength, signatureBytes, 32);
  
  return String(encodedSignature);
}

void setup() {
  Serial.begin(9600);
  
  String formattedDate = getFormattedDate();
  String signature = genSignature(HTTP_METHOD, REQUEST_URI, AK, SK);
  String Authorization = "SEGWAY " + String(AK) + ":" + signature;
  
  Serial.println("formattedDate: " + formattedDate);
  Serial.println("Authorization: " + Authorization);
}

void loop() {
  // Nothing to do here
}
