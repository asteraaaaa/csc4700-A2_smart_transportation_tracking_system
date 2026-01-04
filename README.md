# Smart Transportation Tracking System

**Edge-Based Event Detection & Selective Cloud Communication**

**Course:** Embedded System Development (CSC4700)
**Institution:** Universiti Putra Malaysia
**Assignment:** 2
**Keywords:** Edge computing, ESP32, driver behavior, event-driven telemetry, ThingsBoard

---

## 1. Project Overview

This project implements a **smart transportation monitoring system** that performs **safety-critical driving behavior detection directly at the edge**, rather than relying on continuous cloud processing.

Unlike cloud-centric architectures that stream all sensor data, this system:

* Executes **sensor fusion and event detection locally**
* Provides **immediate driver alerts**
* Uploads **only validated unsafe driving events** to the cloud

This design reduces **latency**, **energy consumption**, and **network dependence**, making it suitable for real-time vehicular safety applications.

---

## 2. System Architecture

The system follows a **three-layer edge-first architecture**:

### 2.1 Sensing Layer

* **GPS Module (GY-NEO6MV2)**
  Provides latitude, longitude, speed, heading, and UTC timestamp
* **IMU (Arduino Nano 33 BLE Rev2)**
  Streams raw accelerometer and gyroscope data via UART
  *(No filtering or decision-making at this node)*

### 2.2 Edge Intelligence Layer (ESP32)

* Real-time sensor fusion (Madgwick filter)
* Gravity-compensated motion analysis
* State-machine gated event detection
* Severity classification
* Local visual and auditory alerts

All safety-critical logic runs **entirely on the ESP32**, independent of cloud availability.

### 2.3 Cloud Visualization Layer

* **ThingsBoard Dashboard**
* Receives **event-driven telemetry only**
* Displays:

  * Unsafe driving event logs
  * Event severity
  * Speed at event time
  * GPS-based map visualization

---

## 3. Hardware Components

| Component                | Role                            |
| ------------------------ | ------------------------------- |
| ESP32 Dev Module         | Edge processor & decision logic |
| Arduino Nano 33 BLE Rev2 | Raw IMU sensing                 |
| GY-NEO6MV2 GPS           | Position, speed, UTC time       |
| NeoPixel LED Strip       | Visual alerts                   |
| Piezo Buzzer             | Auditory alerts                 |

---

## 4. Edge Processing Pipeline

### 4.1 Sensor Fusion

* Madgwick algorithm fuses accelerometer and gyroscope data
* Outputs roll, pitch, and yaw in real time

### 4.2 Feature Extraction

* Gravity-compensated linear acceleration
* Jerk (rate of change of acceleration)
* Yaw rate

### 4.3 State Machine Gating

* **IDLE → ARMED → IDLE**
* Prevents noise-induced false detections
* Enables detection only during meaningful motion

---

## 5. Unsafe Driving Event Detection

Detected events include:

* **Harsh Braking**
* **Rapid Acceleration**
* **Sharp Turning**
* **Collision**

Detection is:

* Rule-based and deterministic
* Validated using **GPS context**
* Filtered to reject events at near-zero speed

### Severity Levels

Each event is classified as:

* Low
* Medium
* High

Severity determines alert intensity and cloud reporting priority.

---

## 6. Local Alerting

Alerts are triggered **immediately at the edge**:

* **NeoPixel LED**

  * Green → Low severity
  * Yellow → Medium severity
  * Red blinking → High severity
* **Piezo Buzzer**

  * Short beep → Medium severity
  * Repeated tone → High severity

Alerts remain functional even during **Wi-Fi or cloud outages**.

---

## 7. Cloud Communication

### 7.1 Event-Driven Telemetry

* HTTP JSON uploads
* Triggered **only when unsafe events occur**
* Payload includes:

  * Event type & severity
  * GPS timestamp
  * Cloud upload timestamp
  * Speed
  * Location

### 7.2 Dashboard Purpose

* Post-event analysis
* Spatial and temporal inspection
* Not used for real-time control

---

## 8. Performance Evaluation Summary

### 8.1 Latency

* End-to-end (edge detection → cloud ingestion)
* Typical latency: **~1–2 seconds**
* Dominated by network transmission, not edge computation

### 8.2 Energy Efficiency

* Wi-Fi transmission duty cycle: **~12.4%**
* Estimated **~88% reduction** compared to continuous telemetry
* Energy usage scales with event frequency, not sensor rate

### 8.3 Reliability

* No event flooding under noisy IMU input
* False collisions rejected at low speed
* Edge alerts remain operational without Wi-Fi
* Stable long-term execution observed

---

## 9. Key Contributions

* Demonstrates **edge-first safety architecture**
* Achieves **deterministic local decision-making**
* Reduces bandwidth and energy usage
* Maintains functionality under network disruption
* Uses transparent, explainable detection logic

---

## 10. Limitations & Future Work

* Threshold-based detection requires manual tuning
* Limited adaptability to driver behavior variability
* Future enhancements:

  * Adaptive thresholds
  * MQTT communication
  * Offline buffering
  * Learning-based behavior models
  * CAN bus integration

---

## 11. Conclusion

This project validates that **edge computing is a practical and effective approach** for safety-oriented transportation monitoring.
By moving intelligence closer to the sensors, the system achieves **low latency**, **energy efficiency**, and **robust operation**, even in intermittently connected environments.

---

## 12. Reference Implementation

See full technical report and experimental results in the submitted PDF.

* Extract an **“Experimental Methodology” markdown** for graders
* Align it line-by-line with your **Arduino code comments**
