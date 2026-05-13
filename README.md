# 🖊️ Pen Plotter CNC — WiFi-Controlled 2-Axis Drawing Machine

Máy vẽ CNC 2 trục điều khiển qua WiFi, sử dụng kiến trúc dual-MCU: **ESP32** làm host/web server và **Raspberry Pi Pico** làm motion controller thời gian thực.

---

## Kiến trúc hệ thống

```
[Browser] ──WebSocket/HTTP──▶ [ESP32 Host v2.1]
                                      │
                              UART Serial2 (115200 baud)
                                      │
                               [Pico CNC Controller]
                                      │
                    ┌─────────────────┴─────────────────┐
               [X Stepper]                         [Y Stepper]
```

| MCU | Vai trò |
|---|---|
| ESP32 | WiFi, web dashboard, SPIFFS lưu G-code, FreeRTOS job/cmd queue |
| Raspberry Pi Pico | Parse G-code, Bresenham + trapezoid accel, arc interpolation |

---

## Tính năng

- **Web dashboard** host trực tiếp trên ESP32 — truy cập qua `http://[IP_ESP32]/`, không cần server ngoài
- **Upload file G-code** từ browser, tự động chạy sau khi upload
- **Jog thủ công** 4 hướng, bước 0.1 / 1 / 10 / 50 mm, hold-to-move
- **G-code trực tiếp** nhập từ textarea trên web
- **Tọa độ X/Y realtime** cập nhật qua WebSocket
- **Emergency STOP** — phản hồi ngay trong vòng lặp step của Pico
- **Arc interpolation** G2/G3 mượt — CONTINUOUS mode, không giật giữa các segment
- **Full circle** G2/G3 khi điểm đầu = điểm cuối (360°)

---

## Phần cứng

### Kết nối ESP32 ↔ Pico (UART)

| ESP32 | Pico |
|---|---|
| GPIO16 (RX2) | GPIO0 (TX — Serial1) |
| GPIO17 (TX2) | GPIO1 (RX — Serial1) |
| GND | GND (**bắt buộc nối chung**) |

### Kết nối Stepper (Pico)

| Pin | Chức năng |
|---|---|
| GPIO 2 | X STEP |
| GPIO 3 | X DIR |
| GPIO 4 | Y STEP |
| GPIO 5 | Y DIR |
| GPIO 6 | ENABLE (LOW = bật) |
| GPIO 7 | SPINDLE / Servo bút |

---

## Thông số máy mặc định

```
STEPS_PER_MM_X  = 80.0
STEPS_PER_MM_Y  = 79.0
MAX_SPEED       = 150 mm/s
ACCELERATION    = 400 mm/s²
ARC_TOLERANCE   = 0.01 mm  (chord error)
```

> Đo lại `STEPS_PER_MM` và kiểm tra `INVERT_X/Y` trong `pico_cnc2.ino` trước khi chạy lần đầu.

---

## Cài đặt & Cấu hình

### 1. ESP32 (`esp32_host_v2_1.ino`)

Đổi WiFi credentials:

```cpp
const char* WIFI_SSID     = "your_wifi";
const char* WIFI_PASSWORD = "your_password";
```

Thư viện cần cài (Arduino IDE / PlatformIO):
- `ESPAsyncWebServer`
- `AsyncTCP`
- SPIFFS, FreeRTOS (built-in ESP32 core)

### 2. Pico (`pico_cnc2.ino`)

Chỉnh thông số máy nếu cần:

```cpp
#define STEPS_PER_MM_X   80.0f
#define STEPS_PER_MM_Y   79.0f
#define INVERT_X         true    // đổi nếu motor chạy ngược chiều
#define INVERT_Y         true
```

Core cần cài: **Arduino-Pico** (Earle Philhower)

---

## HTTP API

| Method | Endpoint | Mô tả |
|---|---|---|
| GET | `/` | Web UI |
| POST | `/gcode` | Upload G-code text → lưu SPIFFS → chạy |
| POST | `/cmd` | Lệnh đơn (jog, home, inline G-code) |
| POST | `/stop` | Dừng khẩn cấp |
| POST | `/clear` | Xóa file G-code khỏi SPIFFS |
| GET | `/info` | JSON: firmware, file size, heap, uptime |

### WebSocket `/ws`

ESP32 broadcast JSON sau mỗi thay đổi trạng thái:

```json
{
  "status": "Running 42%",
  "x": 100.25,
  "y": 50.10,
  "ready": false,
  "ver": "v2.1"
}
```

---

## G-code được hỗ trợ

| Lệnh | Mô tả |
|---|---|
| `G0 X Y` | Rapid move |
| `G1 X Y F` | Linear move |
| `G2 X Y I J F` | Arc CW |
| `G3 X Y I J F` | Arc CCW |
| `G28` | Home (về 0, 0) |
| `G90` / `G91` | Absolute / Relative mode |
| `G92 X Y` | Set zero tại vị trí hiện tại |
| `G4 P` | Dwell (ms) |
| `G20` / `G21` | Inch / mm mode |
| `M3 S` | Spindle ON |
| `M5` | Spindle OFF |
| `M17` / `M18` | Enable / Disable motors |
| `M30` | End of program |

---

## Cấu trúc repo

```
pen-plotter-cnc/
├── esp32_host_v2_1.ino   — ESP32: WiFi host, web server, FreeRTOS, SPIFFS
├── pico_cnc2.ino          — Pico: G-code parser, motion controller
└── README.md
```

---
