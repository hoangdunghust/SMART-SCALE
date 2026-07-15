# Smart Scale System

## Giới thiệu

Smart Scale System là hệ thống cân điện tử thông minh được xây dựng trên nền tảng vi điều khiển **STM32F429ZI**. Hệ thống sử dụng thẻ **RFID RC522** để nhận dạng người dùng, cảm biến **Load Cell kết hợp HX711** để đo khối lượng, đồng hồ thời gian thực **RTC của STM32** để ghi nhận thời gian lưu lịch sử và truyền dữ liệu về máy tính dưới dạng **JSON** thông qua giao tiếp UART.

Ngoài việc hiển thị kết quả cân trên LED 7 đoạn, hệ thống còn hiển thị thông tin trên màn hình OLED SSD1306 và lưu lịch sử **03 lần cân gần nhất** của từng thẻ RFID.

---

# Chức năng chính

- Đọc UID của thẻ RFID RC522.
- Nhận dạng người dùng thông qua UID.
- Đo khối lượng bằng Load Cell + HX711.
- Tự động hiệu chuẩn điểm 0 (Tare) khi khởi động.
- Hiển thị khối lượng trên LED 7 đoạn.
- Hiển thị cân nặng và UID trên màn hình OLED SSD1306.
- Quét RFID liên tục để phát hiện thẻ ngay lập tức.
- Cập nhật dữ liệu cân mỗi 5 giây.
- Lưu lịch sử 03 lần cân gần nhất của từng thẻ RFID.
- Truyền dữ liệu JSON về máy tính thông qua UART.
- Khi không có thẻ RFID, UID được đặt về `00000000`.

---

# Kiến trúc hệ thống

```text
                  RFID Card
                      │
                      ▼
               RC522 RFID Reader
                      │ SPI
                      ▼
           STM32F429ZI Development Board
     ┌──────────┬────────────┬────────────┬─────────────┐
     │          │            │            │             │
     ▼          ▼            ▼            ▼             ▼
   HX711     RTC STM32     OLED SSD1306   UART      Dual 7-Segment LED
     │          │            │            │
     ▼          ▼            ▼            ▼
 Load Cell   Time Base     Display         PC

                  
             
```

---

# Phần cứng sử dụng

| Thiết bị | Chức năng |
|----------|-----------|
| STM32F429ZI | Vi điều khiển trung tâm |
| RC522 RFID | Đọc UID thẻ |
| Mifare Card | Thẻ định danh |
| HX711 | Bộ khuếch đại và ADC cho Load Cell |
| Load Cell | Đo khối lượng |
| OLED SSD1306 | Hiển thị cân nặng và UID |
| LED 7 đoạn 5161BS | Hiển thị khối lượng |
| UART | Giao tiếp với máy tính |

---

# Công nghệ sử dụng

- STM32CubeIDE
- STM32 HAL Library
- Embedded C
- UART Communication
- SPI Communication
- I2C Communication
- GPIO Control

---

# Luồng hoạt động

## Bước 1

Khởi động hệ thống.

## Bước 2

Khởi tạo:

- GPIO
- SPI
- I2C
- UART
- RTC
- RC522
- HX711
- OLED SSD1306

## Bước 3

Thực hiện hiệu chuẩn điểm 0 (Tare) cho Load Cell.

## Bước 4

Quét thẻ RFID liên tục.

- Nếu phát hiện thẻ:
  - Đọc UID.
  - Hiển thị UID trên OLED.
  - Lưu UID hiện tại.

## Bước 5

Cứ mỗi 5 giây:

- Đọc khối lượng từ HX711.
- Cập nhật LED 7 đoạn.
- Cập nhật OLED.
- Gửi dữ liệu JSON qua UART.

## Bước 6

Nếu có thẻ RFID:

- Lưu lịch sử cân của UID.
- Chỉ lưu tối đa 03 lần cân gần nhất.

## Bước 7

Nếu bỏ thẻ ra khỏi đầu đọc trong hơn 2 giây:

- Xóa UID.
- OLED hiển thị "No Card".

---

# Cấu trúc chương trình

```text
main.c
│
├── HX711 Driver
│     ├── HX711_Read()
│     ├── HX711_Tare()
│     └── HX711_GetWeight()
│
├── RC522 Driver
│     ├── RC522_Init()
│     ├── RC522_WriteReg()
│     ├── RC522_ReadReg()
│     ├── RC522_ToCard()
│     └── RC522_CheckCard()
│
├── OLED Driver
│     └── Display_Data_On_OLED()
│
├── LED Display Driver
│     ├── Update_LED_Buffer()
│     └── LED_Scan_Routine()
│
├── History Manager
│     └── Update_And_Log_History()
│
└── UART Communication
      └── Send_Data_To_PC()
```

---

# Giao tiếp phần cứng

## SPI

- RC522 RFID

## I2C

- OLED SSD1306

## GPIO

- HX711
- LED 7 đoạn
- RFID Control

## UART

- Truyền dữ liệu JSON tới máy tính

---

# Định dạng dữ liệu

Ví dụ dữ liệu gửi tới máy tính:

```json
{
    "uid":"736A5B11",
    "weight":65.40
}
```

Nếu chưa có thẻ RFID:

```json
{
    "uid":"00000000",
    "weight":65.40
}
```

---

# Lưu lịch sử cân

Mỗi UID được lưu tối đa **03 lần cân gần nhất**.

Ví dụ hiển thị trên UART:

```text
--- History (UID:736A5B11) ---

1: 64.80 kg | 09:30:12 - 15/07/2026
2: 65.10 kg | 10:15:25 - 15/07/2026
3: 65.40 kg | 11:42:03 - 15/07/2026
```

Khi số lần cân vượt quá 3, bản ghi cũ nhất sẽ tự động bị ghi đè.

---

# Ứng dụng

- Hệ thống cân thông minh
- Phòng Gym
- Phòng khám
- Quản lý bệnh nhân
- Theo dõi sức khỏe
- Hệ thống IoT

---

# Hướng phát triển

- Kết nối WiFi bằng ESP32.
- Tích hợp module đồng hồ thời gian thực (RTC DS3231) để bổ sung thông tin thời gian vào dữ liệu JSON.
- Thiết kế Web Dashboard.
- Đồng bộ dữ liệu với Cloud.
