# README

## Git copy file with history to another repository

```bash
mkdir agv_arduino/patch
cd to/origin/repo
git format-patch -o ~/catkin_ws/src/agv_arduino/patch --root ./agv_arduino/upload_arduino.py
cd to/new/repo
git am --3way patch/*.patch && rm patch/*.patch
```




# Quản lý phiên bản firmware trên Arduino 

- Xem được thông tin git của chương trình đang dùng trên Arduino như:
Thông tin chương trình chính:

    COMMIT_HASH

    COMMIT_MESSAGE

    COMMIT_AUTHOR

    COMMIT_DATE

    BRANCH_NAME

Thông tin thư viện:

    LIBS_DEPEND 
    ARDUINO_COMMON_LIBS

Ví dụ:

    ========== GIT INFO ==========
    COMMIT_HASH: fe89723c
    COMMIT_MESSAGE: Test version management program arduino
    COMMIT_AUTHOR: hoangdc
    COMMIT_DATE: 2025-04-03 09:31:11
    BRANCH_NAME: version_manage
    ==============================
    ========== LIB INFO ==========
    LIBS_DEPEND: denyssene/SimpleKalmanFilter@^0.1.0 - adafruit/Adafruit NeoPixel@^1.11.0 - ricaun/ArduinoUniqueID@^1.1.0 - fastled/FastLED@3.6.0 - 
    ARDUINO_COMMON_LIBS:  No.1 - commit_hash: 080c9696 - commit_message: Add change battery pin tada - commit_author: dinhson-nguyen - commit_date: 2024-06-10 14:44:11


### 1. Cài đặt
- Installing PlatformIO IDE Extension on VS Code: 
    https://randomnerdtutorials.com/vs-code-platformio-ide-esp32-esp8266-arduino/

- Thêm biến mối trường cho platformIO:

    ```echo 'export PATH=$HOME/.platformio/penv/bin:$PATH' >> ~/.bashrc```

    ```source ~/.bashrc```

- Kiểm tra thông tin PlatformIO trên hệ thống:

    ``` pio system info```
    
    như hình bên dưới:

    ![alt text](<Screenshot from 2025-04-03 13-29-09.png>)

- Cài thư viện git trong python:

    ```pip install gitpython```

### 2. Xem thông tin chương trình
- Commit chương trình lên GIT

- Sau đó build và nạp chương trình xuống Arduino.

- Bật PlatformIO: Serial Monitor để xem thông tin:

    ![alt text](<Screenshot from 2025-04-03 13-39-44.png>)

### 3. Lưu ý

