# Smart Scale System

## Giới thiệu

Smart Scale System là hệ thống cân điện tử thông minh được xây dựng trên nền tảng vi điều khiển STM32F429ZI. Hệ thống cho phép nhận dạng người dùng bằng thẻ RFID, đo khối lượng bằng Load Cell, ghi nhận thời gian đo thông qua đồng hồ thời gian thực DS3231 và gửi dữ liệu về máy tính dưới dạng JSON thông qua giao tiếp UART.

Dự án được thiết kế nhằm mô phỏng một hệ thống cân điện tử sử dụng trong phòng khám, phòng gym hoặc các hệ thống quản lý sức khỏe, nơi mỗi kết quả cân được gắn với một người dùng xác định.

---

## Chức năng chính

- Đọc UID của thẻ RFID RC522.
- Nhận dạng người dùng thông qua UID của thẻ.
- Đo khối lượng bằng cảm biến Load Cell kết hợp HX711.
- Hiệu chuẩn điểm 0 (Tare) khi khởi động.
- Tự động phát hiện giá trị cân ổn định.
- Hiển thị khối lượng trên LED 7 đoạn.
- Đọc thời gian thực từ DS3231.
- Đóng gói dữ liệu theo định dạng JSON.
- Truyền dữ liệu đến máy tính qua UART.
- Hiển thị trạng thái hoạt động bằng LED trên board.

---

## Kiến trúc hệ thống

```
                RFID Card
                    │
                    ▼
              RC522 RFID Reader
                    │ SPI
                    ▼
          STM32F429ZI Development Board
      ┌──────────┬─────────────┬────────────┐
      │          │             │            │
      ▼          ▼             ▼            ▼
   HX711      DS3231       LED Display    UART
      │          │             │            │
      ▼          ▼             ▼            ▼
 Load Cell   Real Time     2x 7 Segment     PC
```

---

## Phần cứng sử dụng

| Thiết bị | Chức năng |
|----------|-----------|
| STM32F429ZI | Vi điều khiển trung tâm |
| RC522 RFID | Đọc UID thẻ |
| Mifare Card | Thẻ định danh |
| HX711 | Chuyển đổi ADC cho Load Cell |
| Load Cell | Đo khối lượng |
| DS3231 | Đồng hồ thời gian thực |
| LED 7 đoạn 5161BS | Hiển thị khối lượng |
| UART | Gửi dữ liệu về máy tính |

---

## Công nghệ sử dụng

- STM32CubeIDE
- STM32 HAL Library
- Embedded C
- UART Communication
- SPI Communication
- I2C Communication
- GPIO Control

---

## Luồng hoạt động

### Bước 1

Khởi động hệ thống.

### Bước 2

Khởi tạo:

- GPIO
- SPI
- I2C
- UART
- RC522
- HX711
- DS3231

### Bước 3

Chờ người dùng đưa thẻ RFID.

### Bước 4

Đọc UID của thẻ.

### Bước 5

Đọc thời gian hiện tại từ DS3231.

### Bước 6

Đo khối lượng liên tục.

### Bước 7

Khi giá trị cân ổn định:

- Lưu giá trị cuối cùng.
- Đóng gói dữ liệu.

Ví dụ:

```json
{
  "uid":"736A5B11",
  "weight":65.40,
  "timestamp":"2026-07-01 16:55:32"
}
```

### Bước 8

Gửi dữ liệu qua UART đến máy tính.

### Bước 9

Quay lại chế độ chờ.

---

## Cấu trúc chương trình

```
main.c
│
├── HX711 Driver
│     ├── HX711_Read()
│     ├── HX711_Tare()
│     └── HX711_GetWeight()
│
├── RC522 Driver
│     ├── RC522_Init()
│     ├── RC522_Request()
│     ├── RC522_Anticoll()
│     ├── RC522_SelectTag()
│     └── RC522_CheckCard()
│
├── DS3231 Driver
│     └── DS3231_ReadTime()
│
├── LED Display Driver
│     ├── Update_LED_Buffer()
│     └── LED_Scan_Routine()
│
└── UART Communication
      ├── Send_Card_ID_To_PC()
      └── Send_Data_To_PC()
```

---

## Giao tiếp phần cứng

### SPI

- RC522

### I2C

- DS3231

### GPIO

- HX711
- LED 7 đoạn
- LED trạng thái

### UART

- Gửi dữ liệu JSON về máy tính

---

## Định dạng dữ liệu

Ví dụ dữ liệu gửi đến máy tính:

```json
{
  "uid":"736A5B11",
  "weight":65.40,
  "timestamp":"2026-07-01 16:55:32"
}
```

---

## Ứng dụng

- Hệ thống cân thông minh
- Quản lý phòng Gym
- Quản lý bệnh nhân
- Quản lý nhân viên
- Theo dõi sức khỏe
- Hệ thống IoT

---

## Hướng phát triển

- Kết nối WiFi bằng ESP32.
- Gửi dữ liệu lên MQTT Broker.
- Lưu dữ liệu lên Firebase hoặc Supabase.
- Thiết kế giao diện Web Dashboard.
- Hiển thị lịch sử cân.
- Xây dựng ứng dụng Android.
- Thêm cảm biến chiều cao để tính BMI.
- Đồng bộ dữ liệu với Cloud.

---

## Tác giả

**Hoàng Dũng**

Chuyên ngành: Công nghệ thông tin

Dự án môn học: Hệ thống nhúng (Embedded Systems)