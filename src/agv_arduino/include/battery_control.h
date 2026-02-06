#include "agv_arduino_define.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <SimpleKalmanFilter.h>
#define RS485_Serial Serial2

#define DEBUG_BMS
// #undef DEBUG_BMS

#ifdef DEBUG_BMS
  #define BMS_DEBUG(x)          Serial.print(x)
  #define BMS_DEBUGLN(x)        Serial.println(x)
  #define BMS_DEBUGHEX(x)       Serial.print(x, HEX)
  #define BMS_DEBUG2(x,y)       Serial.print(x); Serial.print(y)
  #define BMS_DEBUG2LN(x,y)     Serial.print(x); Serial.println(y)
#else
  #define BMS_DEBUG(x)
  #define BMS_DEBUGLN(x)
  #define BMS_DEBUGHEX(x)
  #define BMS_DEBUG2(x,y)
  #define BMS_DEBUG2LN(x,y)
#endif

class BatteryControl {
private:
  bool inChargingRange_;
  bool chargeToMaxLimit = false;

  float maxChargeLimit = 100.0f;
  float minChargeLimit = 95.0f;
  unsigned long t_1 = 0;
  unsigned long r_1 = 0;
  float vol_average = 0.0f;
  float ampe_average = 0.0f;
  float percent_return = 0.0f;
  bool reset_100 = false;
  int false_couting = 0;

  // Biến cho state machine
  typedef enum {
    SEND_CMD,
    WAIT_RESPONSE,
    READ_DATA,
    HANDLE_ERROR,
    RESET_FULL_CHARGE
  } STATE;

  STATE state = SEND_CMD;

  // Buffer nhận dữ liệu
  #define MODBUS_BUF_SIZE 13
  uint8_t modbus_buf[MODBUS_BUF_SIZE]{};
  uint8_t modbus_idx = 0;
  uint32_t last_byte_time = 0;
  #define BYTE_TIMEOUT 50  // ms

  // === ĐẾM SỐ LẦN GỌI HÀM ĐỂ TIMEOUT ===
  uint8_t call_counter = 0;
  const uint8_t TIMEOUT_CALL_LIMIT = 4;
  uint8_t timeout_count = 0;
  const uint8_t MAX_RETRY = 3;

  // Hàm chuyển byte → số
  long convert_byte_to_dec(const uint8_t z[]) {
    return (long)z[0] << 8 | z[1];
  }

public:
  BatteryControl() {
    t_1 = millis();
    state = SEND_CMD;
  }

  void dataHandle();
  float readCurrentCharging() { return ampe_average; }
  float readBatteryVoltage() { return vol_average; }
  float readBatteryCapacity() { return percent_return; }
  bool inChargingRange() { return inChargingRange_; }
};

// ==================================================================
// ====================== HÀM CHÍNH =================
// ==================================================================
void BatteryControl::dataHandle() {
  unsigned long currentTime = millis();
  // Serial.println("==============debug==================");
  switch (state) {

    // ==============================================================
    case SEND_CMD:
      while (RS485_Serial.available()) RS485_Serial.read();
      modbus_idx = 0;
      memset(modbus_buf, 0, MODBUS_BUF_SIZE);

      static const uint8_t Vol_Command[13] = {
        0xA5, 0x40, 0x90, 0x08, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x7D
      };
      RS485_Serial.write(Vol_Command, 13);

      // BMS_DEBUGLN("[BMS] → Gửi lệnh đọc BMS");

      call_counter = 0;           // Reset đếm khi gửi lệnh mới
      state = WAIT_RESPONSE;
      break;

    // ==============================================================
    case WAIT_RESPONSE:
      // Tăng đếm mỗi lần gọi
      call_counter++;
      // Đọc dữ liệu nếu có
      while (RS485_Serial.available() && modbus_idx < MODBUS_BUF_SIZE) {
        uint8_t rx = RS485_Serial.read();
        last_byte_time = currentTime;

        if (modbus_idx == 0) {
          if (rx == 0xA5) {
            modbus_buf[modbus_idx++] = rx;
            // BMS_DEBUGLN("[BMS] Tìm thấy header 0xA5");
          }
        } else {
          modbus_buf[modbus_idx++] = rx;

          if (modbus_idx >= 13) {
            // BMS_DEBUGLN("[BMS] Đã nhận đủ 13 byte → xử lý dữ liệu");
            state = READ_DATA;
            break;
          }
        }
      }

      // Timeout giữa các byte (frame bị đứt)
      if (modbus_idx > 0 && modbus_idx < 13 && (currentTime - last_byte_time > BYTE_TIMEOUT)) {
        // BMS_DEBUGLN("[BMS] Frame bị đứt → bỏ");
        modbus_idx = 0;
      }

      // Timeout bằng số lần gọi
      if (call_counter >= TIMEOUT_CALL_LIMIT) {
        timeout_count++;
        // BMS_DEBUG2LN("[BMS] Timeout sau ", TIMEOUT_CALL_LIMIT);
        // BMS_DEBUGLN(" lần gọi hàm → thử lại");
        if (timeout_count >= MAX_RETRY) {
          state = HANDLE_ERROR;
        } else {
          state = SEND_CMD;
        }
      }
      break;

    // ==============================================================
    case READ_DATA:
      // reset cho lần sau
      timeout_count = 0;
      call_counter = 0;
      false_couting = 0;

      // Giải mã dữ liệu
      vol_average     = convert_byte_to_dec(&modbus_buf[4])  / 10.0f;
      ampe_average    = convert_byte_to_dec(&modbus_buf[8])  / 10.0f - 3000.0f;
      percent_return  = convert_byte_to_dec(&modbus_buf[10]) / 10.0f;

      // BMS_DEBUG2LN("         Điện áp:     ", vol_average);
      // BMS_DEBUG2LN("         Dòng điện:   ", ampe_average);
      // BMS_DEBUG2LN("         Dung lượng:  ", percent_return);

      // Logic pin đầy
      BATTERY_FULL = (percent_return >= 100.0f);
      // if (BATTERY_FULL) BMS_DEBUGLN("         PIN ĐÃ ĐẦY 100%");

      // Logic sạc
      if (percent_return >= maxChargeLimit) {
        inChargingRange_ = false; chargeToMaxLimit = false;
      } else if (percent_return < minChargeLimit) {
        inChargingRange_ = true; chargeToMaxLimit = true;
      } else if (chargeToMaxLimit) {
        inChargingRange_ = true;
      }

      if (percent_return < 95.0f) reset_100 = true;

      // Reset 100% khi đầy + không sạc 5 phút
      r_1 = currentTime - t_1;
      if (vol_average >= 28.1f && ampe_average >= -0.1f && ampe_average <= 0.1f) {
        if (reset_100 && r_1 > 300000UL) {
          state = RESET_FULL_CHARGE;
        } else {
          t_1 = currentTime;
        }
      } else {
        state = SEND_CMD;  // bình thường thì gửi tiếp
      }
      break;

    // ==============================================================
    case HANDLE_ERROR:
      // BMS_DEBUGLN("[BMS] LỖI KẾT NỐI BMS → báo -99");
      vol_average = 11.11f;
      ampe_average = -99.0f;
      percent_return = 11.11f;
      false_couting++;
      if (false_couting >= 5) {
        // BMS_DEBUGLN("[BMS] Reset RS485...");
        false_couting = 0;
        RS485_Serial.end();
        delay(100);
        RS485_Serial.begin(9600);
      }
      state = SEND_CMD;
      break;

    // ==============================================================
    case RESET_FULL_CHARGE:
      {
        static const uint8_t reset_cmd[13] = {
          0xA5, 0x40, 0x21, 0x08, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x03, 0xE8, 0xF9
        };
        RS485_Serial.write(reset_cmd, 13);
        // BMS_DEBUGLN("[BMS] ĐÃ GỬI LỆNH RESET 100%");
        reset_100 = false;
      }
      state = SEND_CMD;
      break;
  }

  // === IN RA SERIAL ===
  Serial.print(F("=== DATA BATTERY: "));
  Serial.print(vol_average, 2); Serial.print(F("V | "));
  Serial.print(ampe_average, 3); Serial.print(F("A | "));
  Serial.print(percent_return, 1); Serial.print(F("%"));

  // if (unchanged_start_time > 0) {
  //   Serial.print(F(" | TRIPLE-STUCK: "));
  //   Serial.print((millis() - unchanged_start_time)/1000); Serial.print(F("s"));
  // }
  Serial.println();

}







// #include "agv_arduino_define.h"
// #include <Arduino.h>
// #include <EEPROM.h>
// #include <SimpleKalmanFilter.h>
// #define RS485_Serial Serial2

// class BatteryControl {
// private:
//   bool inChargingRange_;
//   bool chargeToMaxLimit;

//   float maxChargeLimit;
//   float minChargeLimit;
//   float maxBatteryPercent;
//   unsigned long t_1;
//   unsigned long r_1;
//   float ampe;
//   float vol;
//   int state;
//   int INIT;
//   int CHECK_FIRST_TIME;
//   int UPDATE;
//   float stable_ample;
//   float stable_ample_when_charge;
//   uint32_t time_detect_ample_is_full_charge;
//   bool is_charge_when_first_init;
//   float addr_batt_percent;
//   float addr_batt_ampe;
//   float addr_batt_vol;
//   float last_batt_percent;
//   float last_batt_ampe;
//   float last_batt_vol;
//   uint32_t last_time_save_data;
//   uint32_t time_update_battery_average;
//   float vol_average;
//   float ampe_average;
//   int counter_update_battery_average;
//   float time_waiting_when_init;
//   int bytesRead;
//   int false_couting;
//   bool read;
//   bool reset_100;

//   // byte Vol_Command[13] ;
//   byte Data_Modbus[13];
//   byte battery_percent_command[13];

//   // byte Vol_Command[13] =
//   // {0xA5,0x40,0x92,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7F}; //chua
//   // su dung byte Current_charge[13] =
//   // {0xA5,0x40,0x93,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80};  //chua
//   // su dung
//   byte vol_read[2];
//   byte ampe_read[2];
//   byte percent_read[2];
//   float percent_return;
//   byte z[2];

//   // Read Current
//   long convert_byte_to_dec(byte z[]) {
//     long value = 0;
//     value += (long)z[0] << 8;
//     value += (long)z[1];
//     return value;
//   }

// public:
//   BatteryControl();
//   void dataHandle();
//   float readCurrentCharging();
//   float readBatteryVoltage();
//   float readBatteryCapacity();
//   bool inChargingRange();
// };

// BatteryControl::BatteryControl() {
//   inChargingRange_ = false;
//   chargeToMaxLimit = false;

//   maxChargeLimit = 100.0;
//   minChargeLimit = 100.0;
//   maxBatteryPercent = 100.0;

//   ampe = 0.0;
//   vol = 0.0;
//   state = 0;
//   INIT = 0;
//   CHECK_FIRST_TIME = 1;
//   UPDATE = 2;
//   stable_ample = 4;
//   stable_ample_when_charge = 1.5;
//   time_detect_ample_is_full_charge = 0;
//   is_charge_when_first_init = 0;
//   addr_batt_percent = 0;
//   addr_batt_ampe = 5;
//   addr_batt_vol = 10;
//   last_batt_percent = 0;
//   last_batt_ampe = 0;
//   last_batt_vol = 0;
//   last_time_save_data = 0;
//   time_update_battery_average = 0;
//   vol_average = 0;
//   ampe_average = 0;
//   counter_update_battery_average = 0;
//   time_waiting_when_init = 5000;
//   percent_return = 0;
//   bytesRead = 0;
//   false_couting = 0;
//   read = 0;
//   t_1 = millis();
//   reset_100 = 0;
// }

// void BatteryControl::dataHandle() {
//   static unsigned long lastSendTime = 0;
//   const unsigned long SEND_INTERVAL = 500;

//   byte Vol_Command[13] = {0xA5, 0x40, 0x90, 0x08, 0x00, 0x00, 0x00,
//                           0x00, 0x00, 0x00, 0x00, 0x00, 0x7D};

//   unsigned long currentTime = millis();

//   // Logic toggle
//   if (read == 0) {
//     // Gửi lệnh khi read = 0
//     if (currentTime - lastSendTime >= SEND_INTERVAL) {
//       if (RS485_Serial.availableForWrite() >= 13) {
//         while (RS485_Serial.available()) {
//           RS485_Serial.read();
//         }

//         RS485_Serial.write(Vol_Command, 13);
//         lastSendTime = currentTime;
//         bytesRead = 0;
//       }
//     }
//     read = 1; // Chuyển sang chế độ đọc
//   } else {
//     // Đọc dữ liệu khi read = 1
//     // TÌM HEADER TRƯỚC KHI ĐỌC DỮ LIỆU
//     while (RS485_Serial.available() > 0) {
//       // Nếu chưa tìm thấy header
//       if (bytesRead == 0) {
//         byte firstByte = RS485_Serial.read();
//         if (firstByte == 0xA5) {
//           // Tìm thấy header
//           Data_Modbus[0] = firstByte;
//           bytesRead = 1;
//         }
//         // Nếu không phải 0xA5, bỏ qua byte này
//       }
//       // Đã có header, đọc tiếp 12 byte còn lại
//       else if (bytesRead < 13) {
//         Data_Modbus[bytesRead] = RS485_Serial.read();
//         bytesRead++;

//         // Đã đọc đủ 13 byte
//         if (bytesRead == 13) {
//           break; // Thoát khỏi while loop
//         }
//       }
//     }

//     // Xử lý dữ liệu khi đã đọc đủ 13 byte
//     if (bytesRead == 13) {
//       // Bây giờ chắc chắn Data_Modbus[0] == 0xA5
//       false_couting = 0;

//       ampe_read[0] = Data_Modbus[8];
//       ampe_read[1] = Data_Modbus[9];
//       ampe_average =
//           static_cast<float>(convert_byte_to_dec(ampe_read)) / 10 - 3000;

//       vol_read[0] = Data_Modbus[4];
//       vol_read[1] = Data_Modbus[5];
//       vol_average = static_cast<float>(convert_byte_to_dec(vol_read)) / 10;

//       percent_read[0] = Data_Modbus[10];
//       percent_read[1] = Data_Modbus[11];
//       percent_return =
//           static_cast<float>(convert_byte_to_dec(percent_read)) / 10;

//       // Set BATTERY_FULL when percent_return >= 100
//       if (percent_return >= maxBatteryPercent) {
//         BATTERY_FULL = true;
//       } else {
//         BATTERY_FULL = false;
//       }

//       if (percent_return >= maxChargeLimit) {
//         inChargingRange_ = false;
//         chargeToMaxLimit = false;
//       } else if (percent_return < minChargeLimit) {
//         inChargingRange_ = true;
//         chargeToMaxLimit = true;
//       } else if (percent_return >= minChargeLimit &&
//                  percent_return < maxChargeLimit && chargeToMaxLimit) {
//         inChargingRange_ = true;
//       }

//       bytesRead = 0; // Reset để đọc gói tiếp theo
//       read = 0;      // Quay lại chế độ gửi lệnh
//     } else if (currentTime - lastSendTime > 200) {
//       // Timeout - không nhận được đủ dữ liệu
//       false_couting++;
//       bytesRead = 0;
//       read = 0; // Quay lại chế độ gửi lệnh
//     }
//   }

//   // Báo lỗi khi thất bại nhiều lần
//   if (false_couting >= 5) {
//     ampe_average = -99;

//     if (false_couting >= 10) {
//       false_couting = 0;
//       RS485_Serial.end();
//       delay(100);
//       RS485_Serial.begin(9600);
//       lastSendTime = 0; // Force immediate resend
//       read = 0;         // Reset về chế độ gửi lệnh
//     }
//   }

//   if (percent_return < 95) {
//     reset_100 = 1;
//   }

//   r_1 = millis() - t_1;

//   if (vol_average >= 28.1 && (ampe_average >= -0.1 && ampe_average <= 0.1)) {
//     if (reset_100 == 1) {
//       if (r_1 > 300000) {
//         byte reset_100_command[13] = {0xA5, 0x40, 0x21, 0x08, 0x00, 0x00, 0x00,
//                                       0x00, 0x00, 0x00, 0x03, 0xE8, 0xF9};
//         if (RS485_Serial.availableForWrite() >= 13) {
//           RS485_Serial.write(reset_100_command, 13);
//           reset_100 = 0;
//         }
//       }
//     }
//   } else {
//     t_1 = millis();
//   }
// }

// float BatteryControl::readCurrentCharging() { return ampe_average; }

// // Read Voltage
// float BatteryControl::readBatteryVoltage() { return vol_average; }

// // Manager battery capacity
// float BatteryControl::readBatteryCapacity() { return percent_return; }

// bool BatteryControl::inChargingRange() { return inChargingRange_; }
