# Smart Transportation Tracking System

**Edge-Based Event Detection & Selective Cloud Communication**

**Course:** Embedded System Development (CSC4700), Universiti Putra Malaysia
**Assignment:** 2
**Keywords:** Edge computing, ESP32, driver behavior, event-driven telemetry, ThingsBoard

---

## 1. Project Overview

This project is a **smart transportation monitoring system** that detects unsafe driving behavior **directly at the edge** instead of streaming all sensor data to the cloud.

The system:

* Runs **sensor fusion and event detection locally** on an ESP32
* Gives the driver an **immediate local alert** (LED strip and buzzer)
* Uploads **only validated unsafe driving events** to a ThingsBoard dashboard

Keeping the decision logic at the edge reduces latency, network use and energy use, and the detection does not depend on the cloud being reachable.

---

## 2. System Architecture

```
 Arduino Nano 33 BLE Rev2            ESP32 Dev Module                 ThingsBoard Cloud
 (raw IMU, 50 Hz)         UART       (edge brain)            HTTP     (dashboard)
 ┌──────────────────┐   ───────►   ┌──────────────────────┐  ─────►  ┌──────────────┐
 │ accel + gyro     │              │ Madgwick fusion      │  events   │ event log    │
 │ text frames      │              │ state machine        │  only     │ severity     │
 └──────────────────┘              │ event detection      │           │ speed, map   │
 GY-NEO6MV2 GPS ───────────────►   │ local alerts         │           └──────────────┘
                       UART        └──────┬───────┬───────┘
                                          │       │
                                    NeoPixel     Buzzer
```

### 2.1 Sensing layer

* **GPS module (GY-NEO6MV2):** latitude, longitude, speed, heading and UTC time
* **IMU (Arduino Nano 33 BLE Rev2):** streams raw accelerometer (g) and gyroscope (deg/s) readings at 50 Hz over UART. The Nano does no filtering or decision-making.

### 2.2 Edge intelligence layer (ESP32)

* Madgwick sensor fusion for roll, pitch and yaw
* Gravity-compensated linear acceleration, jerk and yaw rate
* State-machine gating (IDLE / ARMED) to ignore noise
* Event detection and severity classification
* Local visual and audible alerts

### 2.3 Cloud visualization layer

* **ThingsBoard Cloud** dashboard that receives event-driven telemetry only: event type, severity, speed, GPS position and timestamps.

---

## 3. Hardware

| Component                | Role                            |
| ------------------------ | ------------------------------- |
| ESP32 Dev Module         | Edge processor and decision logic |
| Arduino Nano 33 BLE Rev2 | Raw IMU sensing                 |
| GY-NEO6MV2 GPS           | Position, speed, UTC time       |
| NeoPixel LED strip (8 LEDs) | Visual alerts                |
| Piezo buzzer             | Audible alerts                  |

### Wiring (pins as defined in `edge_esp32.ino`)

| Connection                              | ESP32 pin |
| --------------------------------------- | --------- |
| GPS TX → ESP32 RX (`GPS_RX`)            | GPIO 16   |
| GPS RX ← ESP32 TX (`GPS_TX`, optional)  | GPIO 17   |
| Nano TX (`Serial1`, D1) → ESP32 RX (`IMU_RX`) | GPIO 27 |
| Nano RX ← ESP32 TX (`IMU_TX`, optional) | GPIO 26   |
| NeoPixel data (`NEOPIXEL_PIN`)          | GPIO 4    |
| Buzzer (`BUZZER_PIN`)                   | GPIO 25   |

Connect the grounds of the ESP32, the Nano and the GPS module together. The Nano 33 BLE and the ESP32 both use 3.3 V logic.

---

## 4. Edge Processing Pipeline

### 4.1 Sensor fusion and features

* The Madgwick filter fuses accelerometer and gyroscope data into roll, pitch and yaw.
* Gravity is estimated from roll and pitch and subtracted to give **linear acceleration** (`ax_lin`).
* **Jerk** is the change in acceleration magnitude between samples; **yaw rate** is the gyroscope z-axis (`gz`).

### 4.2 State machine gating (IDLE ↔ ARMED)

* IDLE → ARMED when the acceleration magnitude deviates from 1 g by more than 0.18 g, or jerk exceeds 0.12 g.
* ARMED → IDLE when the deviation stays below 0.10 g for 1.2 s and the system has been armed for at least 0.8 s.
* Events are only detected while ARMED, which prevents noise-induced false detections.

---

## 5. Unsafe Driving Event Detection

Thresholds are prototype defaults in `edge_esp32.ino` and depend on how the IMU is mounted.

| Event              | Rule                                                              |
| ------------------ | ----------------------------------------------------------------- |
| Collision          | \|A\| > 2.10 g and jerk > 0.45 g on 2 consecutive checks (always High) |
| Harsh braking      | `ax_lin` < −0.55 g                                                 |
| Rapid acceleration | `ax_lin` > +0.55 g                                                 |
| Sharp turn         | \|`gz`\| > 160 deg/s                                               |

* Collision has the highest priority. A 500 ms cooldown separates events.
* **GPS validation:** a collision is **rejected** when a GPS fix reports a speed below 2 km/h (`STATIONARY_GPS`). The other event types are not filtered by GPS speed.
* Rejected events are printed on the serial monitor with the reason but are not alerted or uploaded.

### Severity

Braking, acceleration and turn events are graded by how far the value exceeds its threshold: **Low** below 1.5×, **Medium** from 1.5×, **High** from 2×. Collisions are always High. Severity controls the local alert intensity and is sent to the cloud with every uploaded event.

---

## 6. Local Alerting

Alerts fire immediately at the edge, before any upload is attempted:

| Severity | NeoPixel                          | Buzzer                          |
| -------- | --------------------------------- | ------------------------------- |
| Low      | LEDs 0–2 solid green for 0.8 s    | none                            |
| Medium   | LEDs 0–5 solid yellow for 1.5 s   | one short beep                  |
| High     | all 8 LEDs blinking red for 2.5 s | repeated beeps with the blinks  |

---

## 7. Cloud Communication

* An HTTP POST of JSON to the ThingsBoard Cloud telemetry endpoint, using the device access token. It is sent **only when a validated unsafe event occurs**, with up to 3 attempts.
* Payload fields: `ts` (GPS UTC time in epoch ms), `event_type`, `severity`, `gps_ts_ms`, `upload_ts_ms`, `speed`, `latitude`, `longitude`.
* The dashboard is for post-event analysis (event log, severity, speed, map), not for real-time control.

---

## 8. Performance Evaluation Summary

Figures below were measured in our experiments; the full methodology and results are in the submitted report.

* **Latency:** typically about 1–2 s from edge detection to cloud ingestion, dominated by network transmission rather than edge computation.
* **Energy efficiency:** Wi-Fi transmit duty cycle of about 12.4 %, an estimated reduction of about 88 % compared with continuous telemetry. The firmware prints a 60 s `ENERGY STATS` summary (upload count, total upload time, duty cycle) on the serial monitor.
* **Reliability:** no event flooding under noisy IMU input, false collisions rejected at low speed, and stable long-term execution.

---

## 9. Repository Structure

```
├── edge_esp32/
│   └── edge_esp32.ino              # ESP32: fusion, detection, alerts, ThingsBoard upload
├── nano_raw_imu_sender/
│   └── nano_raw_imu_sender.ino     # Nano 33 BLE: raw IMU stream over UART
├── LICENSE
└── README.md
```

Each sketch is in its own folder because the Arduino IDE treats every `.ino` file in a folder as one sketch.

---

## 10. Setup

**Libraries** (Arduino Library Manager):

* ESP32 sketch: *TinyGPSPlus*, *Madgwick* (`MadgwickAHRS.h`), *Adafruit NeoPixel*, plus the ESP32 board package (provides `WiFi` and `HTTPClient`)
* Nano sketch: *Arduino_BMI270_BMM150*, plus the Arduino Mbed OS Nano boards package

**Steps:**

1. Wire the hardware as in section 3.
2. Upload `nano_raw_imu_sender/nano_raw_imu_sender.ino` to the Nano 33 BLE.
3. In `edge_esp32/edge_esp32.ino`, fill in `WIFI_SSID`, `WIFI_PASS` and `TB_DEVICE_TOKEN` (a ThingsBoard Cloud device access token). **Do not commit these values.**
4. Upload `edge_esp32/edge_esp32.ino` to the ESP32 board (ESP32 Dev Module).
5. Open the serial monitor at **115200 baud** to see the 5 Hz status line, `EDGE_EVENT_JSON` lines for each detected event, and the energy statistics.

---

## 11. Limitations & Future Work

* Thresholds are fixed and need manual tuning for each mounting position.
* Uploads and Wi-Fi reconnection are blocking: while an HTTP request or a reconnect attempt (up to about 3 s) is in progress, IMU processing and the blinking pattern pause. Detection and the first alert do not depend on the cloud.
* Without a GPS fix the telemetry timestamp `ts` is 0, and the upload timestamp is 0 until NTP time has synced.
* Only collisions are checked against GPS speed.
* Possible improvements: adaptive thresholds, non-blocking or MQTT communication, offline buffering, learning-based behavior models, CAN bus integration.

---

## 12. Conclusion

Detecting unsafe driving at the edge is a practical approach for transportation safety monitoring. Moving the intelligence next to the sensors gives low latency, low network energy use and alerts that work even when the connection is poor.

---

## 13. Licence

[MIT](LICENSE)
