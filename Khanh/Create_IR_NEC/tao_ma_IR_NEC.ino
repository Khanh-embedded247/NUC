#include <Arduino.h>

// Hàm này tạo mã NEC từ địa chỉ và lệnh, cũng tính toán địa chỉ đảo và lệnh đảo.
uint32_t createNEcCode(uint8_t address, uint8_t command) {
    uint8_t inv_address = ~address;  // Đảo mọi bit của địa chỉ
    uint8_t inv_command = ~command;  // Đảo mọi bit của lệnh
    uint32_t code = 0;  // Khởi tạo biến mã

    code |= address;    // Gán địa chỉ vào 8 bit thấp nhất của mã
    code <<= 8;         // Dịch tất cả các bit sang trái 8 vị trí
    code |= inv_address; // Gán địa chỉ đảo vào 8 bit tiếp theo
    code <<= 8;         // Dịch lại 8 bit
    code |= command;    // Gán lệnh vào 8 bit tiếp theo
    code <<= 8;         // Dịch lại 8 bit
    code |= inv_command; // Gán lệnh đảo vào 8 bit còn lại

    return code;  // Trả về mã 32 bit hoàn chỉnh
}

// Kiểm tra xem chuỗi có phải là một số hệ thập lục phân hợp lệ
bool isValidHex(const String& str) {
    for (char c : str) {
        if (!isxdigit(c))  // Kiểm tra xem từng ký tự có phải là chữ số hệ hex không
            return false;  // Trả về false nếu có bất kỳ ký tự nào không hợp lệ
    }
    return true;  // Tất cả ký tự đều hợp lệ
}

void setup() {
    Serial.begin(9600);  // Khởi động giao tiếp Serial với tốc độ 9600 bps
    while (!Serial) continue;  // Đợi cho đến khi kết nối Serial sẵn sàng
    Serial.println("Enter Address and Command in format: Address,Command (both in HEX format)");
}

void loop() {
    if (Serial.available() > 0) {  // Kiểm tra xem có dữ liệu từ Serial không
        String input = Serial.readStringUntil('\n');  // Đọc chuỗi cho đến khi gặp ký tự xuống dòng
        int commaIndex = input.indexOf(',');  // Tìm vị trí của dấu phẩy
        if (commaIndex == -1 || commaIndex == 0 || commaIndex == input.length() - 1) {
            Serial.println("Invalid format. Please use Address,Command format.");
            return;  // Trả về lỗi nếu định dạng không đúng
        }

        String addressStr = input.substring(0, commaIndex);  // Lấy phần địa chỉ từ chuỗi
        String commandStr = input.substring(commaIndex + 1);  // Lấy phần lệnh từ chuỗi

        if (!isValidHex(addressStr) || !isValidHex(commandStr)) {  // Kiểm tra tính hợp lệ của hex
            Serial.println("Invalid hex input. Please enter valid hexadecimal numbers.");
            return;  // Nếu không hợp lệ, thông báo lỗi
        }

        uint8_t address = (uint8_t)strtol(addressStr.c_str(), NULL, 16);  // Chuyển đổi địa chỉ từ hex sang số nguyên
        uint8_t command = (uint8_t)strtol(commandStr.c_str(), NULL, 16);  // Chuyển đổi lệnh từ hex sang số nguyên

        uint32_t necCode = createNEcCode(address, command);  // Tạo mã NEC từ địa chỉ và lệnh
        Serial.print("Generated NEC IR Code: 0x");
        if(necCode < 0x10000000) Serial.print("0");  // Đảm bảo hiển thị đủ 8 chữ số hex
        Serial.println(necCode, HEX);  // In mã NEC
    }
}
