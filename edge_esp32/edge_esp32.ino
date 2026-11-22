#ifdef ARDUINO_ARCH_ESP32

#include <TinyGPS++.h>
#include <MadgwickAHRS.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <time.h>

/* ================= UART PINS ================= */
#define GPS_RX 16
#define GPS_TX 17   // optional

#define IMU_RX 27
#define IMU_TX 26   // optional

/* ================= ALERT PINS ================= */
#define BUZZER_PIN    25
#define NEOPIXEL_PIN  4
#define NEOPIXEL_CNT  8

Adafruit_NeoPixel pixel(NEOPIXEL_CNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

/* ================= WIFI / THINGSBOARD ================= */
const char* WIFI_SSID = "";
const char* WIFI_PASS = "";

const char* TB_HOST = "https://thingsboard.cloud";
const char* TB_DEVICE_TOKEN = "";

static const uint8_t MAX_HTTP_RETRIES = 3;
static const uint32_t WIFI_RETRY_MS = 5000;


/* ================= SERIALS ================= */
HardwareSerial gpsSerial(2);
HardwareSerial imuSerial(1);
TinyGPSPlus gps;

/* ================= FUSION ================= */
// IMPORTANT: Should match Nano stream rate reasonably.
static const float SAMPLE_RATE_HZ = 50.0f;
Madgwick filter;

/* ================= RAW IMU (from Nano) ================= */
float ax=0, ay=0, az=1;   // g
float gx=0, gy=0, gz=0;   // deg/s

/* ================= Derived features ================= */
float roll=0, pitch=0, yaw=0;
float accelMag=1.0f;  // |A| in g
float deltaG=0.0f;    // | |A| - 1g |
float jerkG=0.0f;     // | |A| - last|A| |

// Gravity-compensated linear acceleration (in g)
float gX=0, gY=0, gZ=1;
float ax_lin=0, ay_lin=0, az_lin=0;

/* ================= GPS snapshot ================= */
bool   gpsHasFix=false;
double lastLat=0, lastLon=0;
float  lastSpeedKmh=0, lastCourseDeg=0;
char   gpsTimestampUtc[25] = "----";  // "YYYY-MM-DD HH:MM:SS"

/* ================= Timing ================= */
String imuLine;
uint32_t lastImuRxMs = 0;
uint32_t lastSnapshotMs = 0;

/* ================= Phase 4: State machine ================= */
enum SystemState { IDLE, ARMED };
SystemState state = IDLE;
uint32_t armedSinceMs = 0;
uint32_t stableSinceMs = 0;
uint32_t lastEventMs = 0;

/* ================= Phase 4: Filtering knobs ================= */
static const uint32_t IMU_FRESH_MS = 200;      // if no samples within this, IMU is stale
static const uint16_t SNAPSHOT_HZ  = 5;
static const uint32_t SNAPSHOT_PERIOD_MS = 1000UL / SNAPSHOT_HZ;

static const uint32_t EVENT_COOLDOWN_MS = 500;

// Arm/disarm based on motion significance (NOT legal driving logic)
static const float ARM_DELTA_G = 0.18f;
static const float ARM_JERK_G  = 0.12f;
static const float DISARM_DELTA_G = 0.10f;
static const uint32_t MIN_ARM_TIME_MS = 800;
static const uint32_t DISARM_STABLE_MS = 1200;

// Event thresholds (prototype defaults; tune by mounting)
// NOTE: braking/accel thresholds must be applied to ax_lin (gravity compensated)
static const float HARSH_BRAKE_AX_G     = -0.55f;
static const float RAPID_ACCEL_AX_G     =  0.55f;
static const float SHARP_TURN_GZ_DPS    = 160.0f;

// Collision: magnitude + jerk + consecutive hits
static const float COLLISION_G          = 2.10f;
static const float COLLISION_JERK_G     = 0.45f;
static const uint8_t COLLISION_HITS     = 2;
static uint8_t collisionCount = 0;

// Noise gate: treat very low GPS speed as stationary (for rejection decisions)
static const float STATIONARY_KMH = 2.0f;

// ===== Energy efficiency instrumentation =====
static uint32_t tbUploadCount = 0;
static uint32_t tbUploadTimeMs = 0;

static uint32_t energyWindowStartMs = 0;
static const uint32_t ENERGY_STATS_WINDOW_MS = 60000; // 60s

/* ================= Phase 4: Canonical Edge Event ================= */
enum EventType {
  EV_NONE = 0,
  EV_COLLISION,
  EV_HARSH_BRAKING,
  EV_RAPID_ACCELERATION,
  EV_SHARP_TURN
};

enum Severity {
  SEV_LOW = 0,
  SEV_MEDIUM,
  SEV_HIGH
};

struct EdgeEvent {
  bool     valid;
  EventType type;
  Severity sev;

  // Context
  uint32_t t_ms;             // edge timestamp (millis)
  bool     has_gps;
  double   lat;
  double   lon;
  float    speed_kmh;
  float    course_deg;
  char     ts_utc[25];       // GPS UTC time if available

  // Evidence (helps debugging + later analytics)
  float ax_g, ay_g, az_g;           // raw accel
  float ax_lin_g, ay_lin_g, az_lin_g; // linear accel
  float gx_dps, gy_dps, gz_dps;
  float accel_mag_g;
  float jerk_g;

  // Filtering status for transparency
  bool rejected;
  const char* reject_reason;
};

/* ================= Phase 5: Alert timing ================= */
uint32_t alertUntilMs = 0;
uint32_t lastBlinkMs = 0;
bool ledOn = false;

/* ================= Helpers ================= */
static inline float f_abs(float v){ return v < 0 ? -v : v; }

static const char* eventTypeToStr(EventType t) {
  switch (t) {
    case EV_COLLISION: return "COLLISION";
    case EV_HARSH_BRAKING: return "HARSH_BRAKING";
    case EV_RAPID_ACCELERATION: return "RAPID_ACCELERATION";
    case EV_SHARP_TURN: return "SHARP_TURN";
    default: return "NONE";
  }
}

static const char* severityToStr(Severity s) {
  switch (s) {
    case SEV_HIGH: return "HIGH";
    case SEV_MEDIUM: return "MEDIUM";
    default: return "LOW";
  }
}

static Severity severityByRatio(float ratio) {
  // ratio = how far beyond threshold (1.0 == at threshold)
  if (ratio >= 2.0f) return SEV_HIGH;
  if (ratio >= 1.5f) return SEV_MEDIUM;
  return SEV_LOW;
}

static float parseFloatField(const String& s, const char* key) {
  int i = s.indexOf(key);
  if (i < 0) return NAN;
  int start = i + (int)strlen(key);
  int end = s.indexOf('|', start);
  String v = (end > 0) ? s.substring(start, end) : s.substring(start);
  return v.toFloat();
}

/* ================= Forward declarations ================= */
void readGps();
void readImu();
void processImuLine(const String& line);

void updateFusionAndDerived();
void updateStateMachine();
EdgeEvent detectEvent();
void printSnapshot(bool force=false);
void printEventJson(const EdgeEvent& ev);

// Phase 5
void handleAlerts(const EdgeEvent& ev);
void updateHighSeverityAlert();

/* ================= Setup ================= */
void setup() {
  Serial.begin(115200);

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  imuSerial.begin(115200, SERIAL_8N1, IMU_RX, IMU_TX);

  imuLine.reserve(180);

  filter.begin(SAMPLE_RATE_HZ);

  // Phase 5 init
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pixel.begin();
  pixel.clear();
  pixel.show();

  Serial.println("=== ESP32 EDGE BRAIN (PHASE 4+5) ===");
  Serial.print("SAMPLE_RATE_HZ="); Serial.println(SAMPLE_RATE_HZ);
  Serial.print("SNAPSHOT_HZ="); Serial.println(SNAPSHOT_HZ);
  Serial.println("Policy: raw streams suppressed; only unsafe events emitted as JSON");
  Serial.println("[PHASE 5] Buzzer + NeoPixel initialized");
}

/* ================= Loop ================= */
void loop() {
  printEnergyStatsTick();

  readGps();
  readImu();

  const uint32_t now = millis();
  const bool imuFresh = (now - lastImuRxMs) <= IMU_FRESH_MS;

  if (imuFresh) {
    updateFusionAndDerived();
    updateStateMachine();

    EdgeEvent ev = detectEvent();
    if (ev.valid) {
      printEventJson(ev);

      if (!ev.rejected) {
        handleAlerts(ev);

        ensureWiFi();

        String payload = buildTelemetryJson(ev);
        for (uint8_t i = 0; i < MAX_HTTP_RETRIES; i++) {
          if (sendToThingsBoard(payload)) break;
          delay(300);
        }
      }
    }


    if ((now - lastSnapshotMs) >= SNAPSHOT_PERIOD_MS) {
      lastSnapshotMs = now;
      printSnapshot(false);
    }

    // Phase 5: keep HIGH severity blinking alive
    if (now < alertUntilMs) {
      updateHighSeverityAlert();
    }

  } else {
    static uint32_t lastStaleNote = 0;
    if (now - lastStaleNote > 2000) {
      lastStaleNote = now;
      Serial.println("[WARN] IMU stale (no samples recently)");
    }

    // Clear alerts if IMU goes stale
    if (now > alertUntilMs) {
      pixel.clear();
      pixel.show();
      noTone(BUZZER_PIN);
      digitalWrite(BUZZER_PIN, LOW);
    }
  }
}

/* ================= GPS ================= */
void readGps() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid()) {
    gpsHasFix = true;
    lastLat = gps.location.lat();
    lastLon = gps.location.lng();

    if (gps.speed.isValid())  lastSpeedKmh  = gps.speed.kmph();
    if (gps.course.isValid()) lastCourseDeg = gps.course.deg();

    if (gps.date.isValid() && gps.time.isValid()) {
      snprintf(gpsTimestampUtc, sizeof(gpsTimestampUtc),
               "%04d-%02d-%02d %02d:%02d:%02d",
               gps.date.year(), gps.date.month(), gps.date.day(),
               gps.time.hour(), gps.time.minute(), gps.time.second());
    }
  }
}

/* ================= IMU UART ================= */
void readImu() {
  while (imuSerial.available()) {
    char c = (char)imuSerial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      imuLine.trim();
      if (imuLine.length() > 0) processImuLine(imuLine);
      imuLine = "";
    } else {
      if (imuLine.length() < 170) imuLine += c;
      else imuLine = ""; // reset on overflow
    }
  }
}

void processImuLine(const String& line) {
  if (!line.startsWith("IMU|")) return;

  if (line.indexOf("ax=") >= 0) ax = parseFloatField(line, "ax=");
  if (line.indexOf("ay=") >= 0) ay = parseFloatField(line, "ay=");
  if (line.indexOf("az=") >= 0) az = parseFloatField(line, "az=");
  if (line.indexOf("gx=") >= 0) gx = parseFloatField(line, "gx=");
  if (line.indexOf("gy=") >= 0) gy = parseFloatField(line, "gy=");
  if (line.indexOf("gz=") >= 0) gz = parseFloatField(line, "gz=");

  lastImuRxMs = millis();
}

/* ================= Fusion + derived ================= */
void updateFusionAndDerived() {
  // Madgwick expects gyro in rad/s; accel in g for updateIMU
  filter.updateIMU(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD, ax, ay, az);

  roll  = filter.getRoll();
  pitch = filter.getPitch();
  yaw   = filter.getYaw();

  accelMag = sqrtf(ax*ax + ay*ay + az*az);
  deltaG   = f_abs(accelMag - 1.0f);

  static float lastA = 1.0f;
  jerkG = f_abs(accelMag - lastA);
  lastA = accelMag;

  // Gravity vector estimation from fused roll/pitch (degrees -> radians)
  const float r = roll  * DEG_TO_RAD;
  const float p = pitch * DEG_TO_RAD;

  gX = sinf(p);
  gY = -sinf(r) * cosf(p);
  gZ = cosf(r) * cosf(p);

  // Linear acceleration (gravity compensated)
  ax_lin = ax - gX;
  ay_lin = ay - gY;
  az_lin = az - gZ;
}

/* ================= State machine (gating) ================= */
void updateStateMachine() {
  const uint32_t now = millis();

  if (state == IDLE && (deltaG > ARM_DELTA_G || jerkG > ARM_JERK_G)) {
    state = ARMED;
    armedSinceMs = now;
    stableSinceMs = 0;
  }

  if (state == ARMED) {
    if (deltaG < DISARM_DELTA_G) {
      if (stableSinceMs == 0) stableSinceMs = now;
    } else {
      stableSinceMs = 0;
    }

    if (stableSinceMs &&
        (now - stableSinceMs) > DISARM_STABLE_MS &&
        (now - armedSinceMs) > MIN_ARM_TIME_MS) {
      state = IDLE;
      stableSinceMs = 0;
    }
  }
}

/* ================= Event detection + filtering ================= */
EdgeEvent detectEvent() {
  EdgeEvent ev{};
  ev.valid = false;
  ev.rejected = false;
  ev.reject_reason = "";

  const uint32_t now = millis();

  if (state != ARMED) {
    collisionCount = 0;
    return ev;
  }

  if (now - lastEventMs < EVENT_COOLDOWN_MS) return ev;

  // ---- Collision (highest priority) ----
  if (accelMag > COLLISION_G && jerkG > COLLISION_JERK_G) collisionCount++;
  else collisionCount = 0;

  if (collisionCount >= COLLISION_HITS) {
    ev.valid = true;
    ev.type = EV_COLLISION;
    ev.sev = SEV_HIGH;
    lastEventMs = now;
    collisionCount = 0;

    if (gpsHasFix && lastSpeedKmh < STATIONARY_KMH) {
      ev.rejected = true;
      ev.reject_reason = "STATIONARY_GPS";
    }
  }

  if (ev.valid) {
    ev.t_ms = now;
    ev.has_gps = gpsHasFix;
    ev.lat = lastLat;
    ev.lon = lastLon;
    ev.speed_kmh = lastSpeedKmh;
    ev.course_deg = lastCourseDeg;
    strncpy(ev.ts_utc, gpsTimestampUtc, sizeof(ev.ts_utc));
    ev.ts_utc[sizeof(ev.ts_utc)-1] = '\0';

    ev.ax_g = ax; ev.ay_g = ay; ev.az_g = az;
    ev.ax_lin_g = ax_lin; ev.ay_lin_g = ay_lin; ev.az_lin_g = az_lin;
    ev.gx_dps = gx; ev.gy_dps = gy; ev.gz_dps = gz;
    ev.accel_mag_g = accelMag;
    ev.jerk_g = jerkG;
    return ev;
  }

  // ---- Harsh braking / rapid acceleration (use ax_lin) ----
  if (ax_lin < HARSH_BRAKE_AX_G) {
    float ratio = f_abs(ax_lin) / f_abs(HARSH_BRAKE_AX_G);
    ev.valid = true;
    ev.type = EV_HARSH_BRAKING;
    ev.sev = severityByRatio(ratio);
    lastEventMs = now;
  }
  else if (ax_lin > RAPID_ACCEL_AX_G) {
    float ratio = ax_lin / RAPID_ACCEL_AX_G;
    ev.valid = true;
    ev.type = EV_RAPID_ACCELERATION;
    ev.sev = severityByRatio(ratio);
    lastEventMs = now;
  }
  // ---- Sharp turn from yaw rate ----
  else if (f_abs(gz) > SHARP_TURN_GZ_DPS) {
    float ratio = f_abs(gz) / SHARP_TURN_GZ_DPS;
    ev.valid = true;
    ev.type = EV_SHARP_TURN;
    ev.sev = severityByRatio(ratio);
    lastEventMs = now;
  }

  if (!ev.valid) return ev;

  ev.t_ms = now;
  ev.has_gps = gpsHasFix;
  ev.lat = lastLat;
  ev.lon = lastLon;
  ev.speed_kmh = lastSpeedKmh;
  ev.course_deg = lastCourseDeg;
  strncpy(ev.ts_utc, gpsTimestampUtc, sizeof(ev.ts_utc));
  ev.ts_utc[sizeof(ev.ts_utc)-1] = '\0';

  ev.ax_g = ax; ev.ay_g = ay; ev.az_g = az;
  ev.ax_lin_g = ax_lin; ev.ay_lin_g = ay_lin; ev.az_lin_g = az_lin;
  ev.gx_dps = gx; ev.gy_dps = gy; ev.gz_dps = gz;
  ev.accel_mag_g = accelMag;
  ev.jerk_g = jerkG;

  return ev;
}

/* ================= Canonical JSON event summary ================= */
void printEventJson(const EdgeEvent& ev) {
  Serial.print("EDGE_EVENT_JSON ");
  Serial.print("{\"event_type\":\""); Serial.print(eventTypeToStr(ev.type));
  Serial.print("\",\"severity\":\"");  Serial.print(severityToStr(ev.sev));
  Serial.print("\",\"t_ms\":");        Serial.print(ev.t_ms);

  Serial.print(",\"gps_time_utc\":\""); Serial.print(ev.has_gps ? ev.ts_utc : "NO_GPS");
  Serial.print("\"");

  Serial.print(",\"latitude\":");      Serial.print(ev.has_gps ? ev.lat : 0.0, 6);
  Serial.print(",\"longitude\":");     Serial.print(ev.has_gps ? ev.lon : 0.0, 6);
  Serial.print(",\"speed_kmh\":");     Serial.print(ev.has_gps ? ev.speed_kmh : -1.0f, 1);
  Serial.print(",\"course_deg\":");    Serial.print(ev.has_gps ? ev.course_deg : -1.0f, 1);

  // Evidence (keep for analysis)
  Serial.print(",\"accel_mag_g\":");   Serial.print(ev.accel_mag_g, 2);
  Serial.print(",\"jerk_g\":");        Serial.print(ev.jerk_g, 2);
  Serial.print(",\"ax_lin_g\":");      Serial.print(ev.ax_lin_g, 3);
  Serial.print(",\"gz_dps\":");        Serial.print(ev.gz_dps, 1);

  Serial.print(",\"rejected\":");      Serial.print(ev.rejected ? "true" : "false");
  Serial.print(",\"reject_reason\":\""); Serial.print(ev.rejected ? ev.reject_reason : "");
  Serial.print("\"");

  Serial.print(",\"state\":\""); Serial.print(state == IDLE ? "IDLE" : "ARMED"); Serial.print("\"");

  Serial.println("}");
}

/* ================= Phase 5: Alert mapping =================
   LOW    -> Green solid, no beep
   MEDIUM -> Yellow solid, short beep
   HIGH   -> Red blinking + repeated beeps (non-blocking)
*/
void handleAlerts(const EdgeEvent& ev) {
  const uint32_t now = millis();

  // Reset
  pixel.clear();
  pixel.show();
  noTone(BUZZER_PIN);
  digitalWrite(BUZZER_PIN, LOW);
  ledOn = false;

  // LOW severity → LEDs 0–2, green, no sound
  if (ev.sev == SEV_LOW) {
    for (int i = 0; i <= 2; i++) {
      pixel.setPixelColor(i, pixel.Color(0, 60, 0)); // green
    }
    pixel.show();
    alertUntilMs = now + 800;
    return;
  }

  // MEDIUM severity → LEDs 0–5, yellow, short beep
  if (ev.sev == SEV_MEDIUM) {
    for (int i = 0; i <= 5; i++) {
      pixel.setPixelColor(i, pixel.Color(60, 60, 0)); // yellow
    }
    pixel.show();
    tone(BUZZER_PIN, 2000, 150); // short beep
    alertUntilMs = now + 1500;
    return;
  }

  // HIGH severity → LEDs 0–7, red blinking + repeated beeps
  if (ev.sev == SEV_HIGH) {
    alertUntilMs = now + 2500;
    lastBlinkMs = 0; // force immediate blink
    return;
  }
}


void updateHighSeverityAlert() {
  const uint32_t now = millis();

  if (now > alertUntilMs) {
    pixel.clear();
    pixel.show();
    noTone(BUZZER_PIN);
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

  // Blink every 200 ms
  if (now - lastBlinkMs >= 200) {
    lastBlinkMs = now;
    ledOn = !ledOn;

    if (ledOn) {
      for (int i = 0; i < NEOPIXEL_CNT; i++) {
        pixel.setPixelColor(i, pixel.Color(90, 0, 0)); // red
      }
      pixel.show();
      tone(BUZZER_PIN, 2500, 120); // sharp warning beep
    } else {
      pixel.clear();
      pixel.show();
      noTone(BUZZER_PIN);
    }
  }
}


/* ================= Debug snapshot ================= */
void printSnapshot(bool force) {
  const uint32_t now = millis();
  const bool imuFresh = (now - lastImuRxMs) <= IMU_FRESH_MS;
  if (!force && !imuFresh) return;

  Serial.print(gpsHasFix ? gpsTimestampUtc : "NO_GPS_TIME");

  Serial.print(" | [GPS] ");
  if (!gpsHasFix) {
    Serial.print("waiting_fix");
  } else {
    Serial.print("Lat:"); Serial.print(lastLat, 6);
    Serial.print(" Lon:"); Serial.print(lastLon, 6);
    Serial.print(" Spd:"); Serial.print(lastSpeedKmh, 1);
    Serial.print("km/h Crs:"); Serial.print(lastCourseDeg, 1);
    Serial.print("deg");
  }

  Serial.print(" | [IMU] R:"); Serial.print(roll, 1);
  Serial.print(" P:");        Serial.print(pitch, 1);
  Serial.print(" Y:");        Serial.print(yaw, 1);

  Serial.print(" | |A|:");    Serial.print(accelMag, 2);
  Serial.print("g jerk:");    Serial.print(jerkG, 2);
  Serial.print(" ax_lin:");   Serial.print(ax_lin, 3);
  Serial.print(" gz:");       Serial.print(gz, 1);

  Serial.print(" | STATE:");
  Serial.println(state == IDLE ? "IDLE" : "ARMED");
}

void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  static uint32_t lastAttempt = 0;
  if (millis() - lastAttempt < WIFI_RETRY_MS) return;
  lastAttempt = millis();

  Serial.print("[WiFi] Connecting...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 3000) {
    delay(200);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" connected");
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  } else {
    Serial.println(" failed");
  }
}

bool sendToThingsBoard(const String& jsonPayload) {
  if (WiFi.status() != WL_CONNECTED) return false;

  uint32_t tStart = millis();

  HTTPClient http;
  String url = String(TB_HOST) + "/api/v1/" + TB_DEVICE_TOKEN + "/telemetry";
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  int code = http.POST(jsonPayload);
  http.end();

  uint32_t tEnd = millis();

  // Record stats (always)
  tbUploadCount++;
  tbUploadTimeMs += (tEnd - tStart);

  if (code == 200) {
    Serial.println("[TB] Upload OK");
    return true;
  } else {
    Serial.print("[TB] Upload failed, HTTP ");
    Serial.println(code);
    return false;
  }
}

void printEnergyStatsTick() {
  uint32_t now = millis();

  if (energyWindowStartMs == 0) {
    energyWindowStartMs = now;
    return;
  }

  if (now - energyWindowStartMs >= ENERGY_STATS_WINDOW_MS) {
    float duty = (tbUploadTimeMs * 100.0f) / ENERGY_STATS_WINDOW_MS;

    Serial.println("=== ENERGY STATS (last 60s) ===");
    Serial.print("Uploads count      : ");
    Serial.println(tbUploadCount);
    Serial.print("Total upload time  : ");
    Serial.print(tbUploadTimeMs);
    Serial.println(" ms");
    Serial.print("WiFi TX duty cycle : ");
    Serial.print(duty, 3);
    Serial.println(" %");
    Serial.println("==============================");

    // Reset window
    tbUploadCount = 0;
    tbUploadTimeMs = 0;
    energyWindowStartMs = now;
  }
}


String buildTelemetryJson(const EdgeEvent& ev) {
  uint64_t gpsTs = gpsUtcToEpochMs();
  uint64_t uploadTs = uploadEpochMs();

  String json = "{";
  json += "\"ts\":" + String(gpsTs) + ",";
  json += "\"values\":{";
  json += "\"event_type\":\"" + String(eventTypeToStr(ev.type)) + "\",";
  json += "\"severity\":\"" + String(severityToStr(ev.sev)) + "\",";
  json += "\"gps_ts_ms\":" + String(gpsTs) + ",";
  json += "\"upload_ts_ms\":" + String(uploadTs) + ",";
  json += "\"speed\":" + String(ev.speed_kmh, 1) + ",";
  json += "\"latitude\":" + String(ev.lat, 6) + ",";
  json += "\"longitude\":" + String(ev.lon, 6);
  json += "}}";
  return json;
}


// ESP32 upload time (cloud-facing clock)
uint64_t uploadEpochMs() {
  return (uint64_t)time(nullptr) * 1000ULL;
}

// Convert GPS UTC date/time to Unix epoch milliseconds
// Convert GPS UTC date/time to Unix epoch milliseconds (ESP32-safe)
uint64_t gpsUtcToEpochMs() {
  if (!gps.date.isValid() || !gps.time.isValid()) return 0;

  int year   = gps.date.year();
  int month  = gps.date.month();
  int day    = gps.date.day();
  int hour   = gps.time.hour();
  int minute = gps.time.minute();
  int second = gps.time.second();

  static const int daysInMonth[] =
    {31,28,31,30,31,30,31,31,30,31,30,31};

  uint32_t days = 0;

  // Years since 1970
  for (int y = 1970; y < year; y++) {
    days += (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) ? 366 : 365;
  }

  // Months this year
  for (int m = 1; m < month; m++) {
    days += daysInMonth[m - 1];
    if (m == 2 && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0))) {
      days += 1; // leap year February
    }
  }

  days += (day - 1);

  uint64_t totalSeconds =
      (uint64_t)days * 86400ULL +
      (uint64_t)hour * 3600ULL +
      (uint64_t)minute * 60ULL +
      (uint64_t)second;

  return totalSeconds * 1000ULL;
}

#endif // ARDUINO_ARCH_ESP32
