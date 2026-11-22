#include "Arduino_BMI270_BMM150.h"

/* ================= STREAM CONFIG ================= */
// Stable, fast, UART-safe
#define STREAM_HZ 50
#define STREAM_PERIOD_MS (1000 / STREAM_HZ)

/* ================= IMU DATA ================= */
float ax, ay, az;   // g
float gx, gy, gz;   // deg/s

unsigned long lastSend = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);   // UART to ESP32

  if (!IMU.begin()) {
    Serial.println("[ERROR] IMU init failed");
    while (1);
  }

  Serial.println("=== NANO RAW IMU SENDER (FORCED READ) ===");
  Serial.print("STREAM_HZ = ");
  Serial.println(STREAM_HZ);
}

void loop() {
  unsigned long now = millis();
  if (now - lastSend < STREAM_PERIOD_MS) return;
  lastSend = now;

  // ================= RAW FORCED READ =================
  // NO availability checks
  // NO returns
  // This guarantees continuous output
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);

  // ================= UART FRAME =================
  // IMU|ax=..|ay=..|az=..|gx=..|gy=..|gz=..
  Serial1.print("IMU|ax=");
  Serial1.print(ax, 5);
  Serial1.print("|ay=");
  Serial1.print(ay, 5);
  Serial1.print("|az=");
  Serial1.print(az, 5);
  Serial1.print("|gx=");
  Serial1.print(gx, 3);
  Serial1.print("|gy=");
  Serial1.print(gy, 3);
  Serial1.print("|gz=");
  Serial1.print(gz, 3);
  Serial1.println();

  // ================= OPTIONAL USB DEBUG =================
  // Uncomment ONLY if needed (USB printing can slow things)
  /*
  Serial.print("ax=");
  Serial.print(ax, 3);
  Serial.print(" ay=");
  Serial.print(ay, 3);
  Serial.print(" az=");
  Serial.print(az, 3);
  Serial.print(" gx=");
  Serial.print(gx, 1);
  Serial.print(" gy=");
  Serial.print(gy, 1);
  Serial.print(" gz=");
  Serial.println(gz, 1);
  */
}
