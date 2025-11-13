/*
  PaceBot – ESP32-S3 RC + QTR + Wi-Fi Provisioning + WebSocket Telemetry + MPU-6500 + Encoder (AS5048A PWM)
  Runtime Config editable over HTTP (/config)

  Endpoints:
    GET  /config
    POST /config
    POST /config/reset
    GET  /wifi/scan
    POST /wifi/connect {"ssid":"..","password":".."}
    GET  /wifi/status
    POST /wifi/forget

  Notes:
    - Config lives in NVS (Preferences "pacebot", key "cfg") as JSON.
    - Control loop reads config live (no reboot needed).
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <math.h>

/* ===================== PINS ===================== */
// QTRX #1 (one 7-sensor board)
#define EMITTER1_PIN 1
int qtr1Pins[7] = {4, 5, 6, 7, 15, 16, 17};

// QTRX #2 (second 7-sensor board)
#define EMITTER2_PIN 2
int qtr2Pins[7] = {18, 8, 3, 9, 10, 11, 12};

// Steering + Throttle
const int STEER_PIN    = 36;   // servo signal
const int THROTTLE_PIN = 35;   // ESC signal

// Rotary encoder (AS5048A in PWM mode)
static const int ENC_PWM_PIN = 21;
static const unsigned long ENC_TO_US = 30000;  // pulseIn timeout (us)

/* Wheel geometry */
static const float TYRE_DIAMETER_MM = 95.0f;
static const float WHEEL_RADIUS_M   = (TYRE_DIAMETER_MM * 0.001f) * 0.5f;

#define SWAP_BLOCKS        0
#define REVERSE_QTR1_ORDER 1
#define REVERSE_QTR2_ORDER 1

/* ===================== Wi-Fi/WS CONFIG ===================== */
static const uint16_t WS_PORT   = 81;
static const char*    MDNS_NAME = "pacebot";
static const char*    AP_PASS   = "setup1234";

/* ===================== Globals ===================== */
WebServer        http(80);
WebSocketsServer ws(WS_PORT);
Preferences      prefs;

enum Mode { IDLE, FOLLOW_TRACK, PACE_ATHLETE, MAP_TRACK, TELEOP };
Mode mode = IDLE;

// RC
Servo steer, throttle;
const int NUM_SENSORS = 14;
int sensorPins[NUM_SENSORS];
uint16_t calMin[NUM_SENSORS], calMax[NUM_SENSORS];
int lastSteerUS = 1500;
int lastThrottleUS = 1500;

uint32_t lastTelem = 0;
uint32_t bootMs    = 0;

/* ===== Teleop state ===== */
int teleopSteerUS     = 1500;
int teleopThrottleUS  = 1500;
uint32_t lastTeleopMs = 0;
static const uint32_t TELEOP_TIMEOUT_MS = 300;  // neutral if stale

/* ===================== Encoder types & helpers ===================== */
struct EncSample {
  uint32_t high_us;
  uint32_t low_us;
  uint32_t period_us;
  float    duty;        // 0..1
  float    angle_deg;   // 0..360
  uint16_t angle_14bit; // 0..16383
};

static bool encReadOnce(EncSample &s, int pin) {
  uint32_t high_us = pulseIn(pin, HIGH, ENC_TO_US);
  uint32_t low_us  = pulseIn(pin, LOW,  ENC_TO_US);
  if (high_us == 0 || low_us == 0) return false;

  s.high_us   = high_us;
  s.low_us    = low_us;
  s.period_us = high_us + low_us;
  s.duty      = (float)high_us / (float)(s.period_us);
  s.angle_deg = s.duty * 360.0f;
  uint32_t counts = (uint32_t)lroundf(s.duty * 16383.0f);
  if (counts > 16383) counts = 16383;
  s.angle_14bit = (uint16_t)counts;
  return true;
}

static bool encReadAveraged(EncSample &out, int pin, int N) {
  int got = 0;
  double high_sum=0, low_sum=0, period_sum=0, duty_sum=0, angle_sum=0;
  for (int i=0;i<N;i++) {
    EncSample s;
    if (encReadOnce(s, pin)) {
      got++;
      high_sum   += s.high_us;
      low_sum    += s.low_us;
      period_sum += s.period_us;
      duty_sum   += s.duty;
      angle_sum  += s.angle_deg;
    } else {
      delayMicroseconds(300);
    }
  }
  if (!got) return false;
  out.high_us   = (uint32_t)lround(high_sum/got);
  out.low_us    = (uint32_t)lround(low_sum/got);
  out.period_us = (uint32_t)lround(period_sum/got);
  out.duty      = (float)(duty_sum/got);
  out.angle_deg = (float)(angle_sum/got);
  uint32_t counts = (uint32_t)lround(out.duty * 16383.0);
  if (counts > 16383) counts = 16383;
  out.angle_14bit = (uint16_t)counts;
  return true;
}

// Encoder integration state
static float    enc_last_angle_deg = NAN;
static uint32_t enc_last_ts_ms     = 0;
static double   distance_m_accum   = 0.0;
static float    speed_mps_curr     = 0.0;

static void updateEncoder() {
  EncSample s;
  if (!encReadAveraged(s, ENC_PWM_PIN, 5)) {
    // If we lose signal, decay the reported speed
    speed_mps_curr *= 0.9f;
    return;
  }
  uint32_t now_ms = millis();

  if (isnan(enc_last_angle_deg)) {
    enc_last_angle_deg = s.angle_deg;
    enc_last_ts_ms     = now_ms;
    return;
  }

  float ddeg = s.angle_deg - enc_last_angle_deg;
  // unwrap across 0/360 to the shortest path
  if (ddeg >  180.0f) ddeg -= 360.0f;
  if (ddeg < -180.0f) ddeg += 360.0f;

  float dt_s = (now_ms - enc_last_ts_ms) * 0.001f;
  if (dt_s <= 0.0f) dt_s = 1e-3f;

  // angular speed (rad/s)
  float omega = (ddeg * (float)M_PI / 180.0f) / dt_s;

  // linear speed and incremental distance
  speed_mps_curr   = omega * WHEEL_RADIUS_M;
  distance_m_accum += fabsf(ddeg) * ((float)M_PI / 180.0f) * WHEEL_RADIUS_M;

  enc_last_angle_deg = s.angle_deg;
  enc_last_ts_ms     = now_ms;
}

/* ===================== IMU (MPU-6500 @ SDA=13,SCL=14) ===================== */
#define SDA_PIN   13
#define SCL_PIN   14
#define I2C_HZ    400000
#define IMU_ADDR  0x68

#define REG_WHO_AM_I     0x75
#define REG_PWR_MGMT_1   0x6B
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B

const float ACCEL_SENS_LSB_PER_G = 4096.0f; // ±8g
const float GYRO_SENS_LSB_PER_DPS = 65.5f;  // ±500 dps
const float G = 9.80665f;
const float DEG2RAD = 3.14159265358979323846f / 180.0f;
const float TEMP_SENS   = 333.87f; // LSB/°C
const float TEMP_OFFSET = 21.0f;   // °C

static bool i2cWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}
static bool i2cRead(uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false; // repeated start
  size_t got = Wire.requestFrom((int)IMU_ADDR, (int)len);
  if (got != len) return false;
  for (size_t i = 0; i < len; ++i) buf[i] = Wire.read();
  return true;
}
static void i2cScan() {
  Serial.println("I2C scan...");
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; ++a) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("  Found device at 0x");
      if (a < 16) Serial.print('0');
      Serial.println(a, HEX);
      found++;
    }
    delay(2);
  }
  if (!found) Serial.println("  No I2C devices found.");
  Serial.println();
}
static inline int16_t be16(const uint8_t *p) { return (int16_t)((p[0] << 8) | p[1]); }

static float ax_ms2=0, ay_ms2=0, az_ms2=0;
static float gx_rad=0, gy_rad=0, gz_rad=0;
static float imu_temp_c=0;
static float imu_yaw_deg=0;
static uint32_t lastIMUms=0;

static bool imuBegin6500() {
  Wire.begin(SDA_PIN, SCL_PIN, I2C_HZ);
  delay(10);

  Serial.println("ESP32-S3 + MPU-6500 bring-up (SDA=13, SCL=14)");
  i2cScan();

  uint8_t who = 0xFF;
  if (!i2cRead(REG_WHO_AM_I, &who, 1)) {
    Serial.println("Failed to read WHO_AM_I. Check wiring.");
    return false;
  }
  Serial.print("WHO_AM_I = 0x"); Serial.println(who, HEX);
  if (who != 0x70) {
    Serial.println("[Unverified] Not 0x70; may not be MPU-6500. Proceeding anyway.");
  }

  if (!i2cWriteReg(REG_PWR_MGMT_1, 0x01)) {
    Serial.println("Failed to write PWR_MGMT_1");
    return false;
  }
  delay(50);

  i2cWriteReg(REG_SMPLRT_DIV, 0x00);
  i2cWriteReg(REG_CONFIG,      0x03);
  i2cWriteReg(REG_GYRO_CONFIG, 0x08);
  i2cWriteReg(REG_ACCEL_CONFIG,0x10);

  lastIMUms = millis();
  imu_yaw_deg = 0;
  Serial.println("IMU initialized.");
  return true;
}
static bool imuRead6500() {
  uint8_t buf[14];
  if (!i2cRead(REG_ACCEL_XOUT_H, buf, sizeof(buf))) return false;

  int16_t ax_raw = be16(&buf[0]);
  int16_t ay_raw = be16(&buf[2]);
  int16_t az_raw = be16(&buf[4]);
  int16_t t_raw  = be16(&buf[6]);
  int16_t gx_raw = be16(&buf[8]);
  int16_t gy_raw = be16(&buf[10]);
  int16_t gz_raw = be16(&buf[12]);

  float ax_g = (float)ax_raw / ACCEL_SENS_LSB_PER_G;
  float ay_g = (float)ay_raw / ACCEL_SENS_LSB_PER_G;
  float az_g = (float)az_raw / ACCEL_SENS_LSB_PER_G;
  ax_ms2 = ax_g * G; ay_ms2 = ay_g * G; az_ms2 = az_g * G;

  float gx_dps = (float)gx_raw / GYRO_SENS_LSB_PER_DPS;
  float gy_dps = (float)gy_raw / GYRO_SENS_LSB_PER_DPS;
  float gz_dps = (float)gz_raw / GYRO_SENS_LSB_PER_DPS;
  gx_rad = gx_dps * DEG2RAD; gy_rad = gy_dps * DEG2RAD; gz_rad = gz_dps * DEG2RAD;

  imu_temp_c = (t_raw / TEMP_SENS) + TEMP_OFFSET;

  uint32_t now = millis();
  float dt = (now - lastIMUms) * 0.001f;
  lastIMUms = now;
  imu_yaw_deg += (gz_dps) * dt;
  while (imu_yaw_deg >= 360.0f) imu_yaw_deg -= 360.0f;
  while (imu_yaw_deg <    0.0f) imu_yaw_deg += 360.0f;

  return true;
}

/* ===================== Runtime Config ===================== */
struct Config {
  bool  white_line;
  float Kp;
  int   error_slowdown_thresh;
  int   lost_confidence_sum;

  int   us_min, us_neu, us_max;
  int   steer_min_us, steer_max_us;
  int   throttle_min, throttle_base, throttle_max;

  unsigned long esc_arm_ms;
  uint16_t qtr_timeout_us;
  unsigned long calibrate_ms;


  float qtr_gamma;
  int   qtr_smooth_n;
  int   min_line_contrast;
  int   deadband_error;

  void setDefaults() {
    white_line = true;
    Kp = 0.12f;
    error_slowdown_thresh = 2500;
    lost_confidence_sum   = 1200;

    us_min = 1000; us_neu = 1500; us_max = 2000;
    steer_min_us = 1200; steer_max_us = 1800;
    throttle_min = 1500; throttle_base = 1600; throttle_max = 1750;

    esc_arm_ms = 3000;
    qtr_timeout_us = 3000;
    calibrate_ms = 3000;

    qtr_gamma         = 0.75f;
    qtr_smooth_n      = 3;
    min_line_contrast = 120;
    deadband_error    = 300;
  }

  bool load(Preferences& p) {
    String s = p.getString("cfg", "");
    if (!s.length()) return false;
    DynamicJsonDocument d(2048);
    if (deserializeJson(d, s)) return false;
    fromJson(d.as<JsonObject>());
    return true;
  }
  void save(Preferences& p) const {
    DynamicJsonDocument d(2048);
    toJson(d.to<JsonObject>());
    String s; serializeJson(d, s);
    p.putString("cfg", s);
  }
  void toJson(JsonObject o) const {
    o["white_line"]=white_line; o["Kp"]=Kp;
    o["error_slowdown_thresh"]=error_slowdown_thresh;
    o["lost_confidence_sum"]=lost_confidence_sum;

    o["us_min"]=us_min; o["us_neu"]=us_neu; o["us_max"]=us_max;
    o["steer_min_us"]=steer_min_us; o["steer_max_us"]=steer_max_us;
    o["throttle_min"]=throttle_min; o["throttle_base"]=throttle_base; o["throttle_max"]=throttle_max;

    o["esc_arm_ms"]=esc_arm_ms; o["qtr_timeout_us"]=qtr_timeout_us; o["calibrate_ms"]=calibrate_ms;

    o["qtr_gamma"]=qtr_gamma;
    o["qtr_smooth_n"]=qtr_smooth_n;
    o["min_line_contrast"]=min_line_contrast;
    o["deadband_error"]=deadband_error;
  }
  void fromJson(JsonObject o) {
    if (o.containsKey("white_line")) white_line = o["white_line"];
    if (o.containsKey("Kp")) Kp = o["Kp"];
    if (o.containsKey("error_slowdown_thresh")) error_slowdown_thresh = o["error_slowdown_thresh"];
    if (o.containsKey("lost_confidence_sum"))   lost_confidence_sum   = o["lost_confidence_sum"];

    if (o.containsKey("us_min")) us_min = o["us_min"];
    if (o.containsKey("us_neu")) us_neu = o["us_neu"];
    if (o.containsKey("us_max")) us_max = o["us_max"];
    if (o.containsKey("steer_min_us")) steer_min_us = o["steer_min_us"];
    if (o.containsKey("steer_max_us")) steer_max_us = o["steer_max_us"];
    if (o.containsKey("throttle_min"))  throttle_min  = o["throttle_min"];
    if (o.containsKey("throttle_base")) throttle_base = o["throttle_base"];
    if (o.containsKey("throttle_max"))  throttle_max  = o["throttle_max"];

    if (o.containsKey("esc_arm_ms"))     esc_arm_ms     = (unsigned long)o["esc_arm_ms"].as<uint32_t>();
    if (o.containsKey("qtr_timeout_us")) qtr_timeout_us = (uint16_t)o["qtr_timeout_us"].as<uint32_t>();
    if (o.containsKey("calibrate_ms"))   calibrate_ms   = (unsigned long)o["calibrate_ms"].as<uint32_t>();

    if (o.containsKey("qtr_gamma"))        qtr_gamma        = o["qtr_gamma"];
    if (o.containsKey("qtr_smooth_n"))     qtr_smooth_n     = o["qtr_smooth_n"];
    if (o.containsKey("min_line_contrast"))min_line_contrast= o["min_line_contrast"];
    if (o.containsKey("deadband_error"))   deadband_error   = o["deadband_error"];

    clampAndFix();
  }
  void clampAndFix() {
    if (Kp < 0.0f) Kp = 0.0f;
    if (error_slowdown_thresh < 0) error_slowdown_thresh = 0;
    if (lost_confidence_sum < 0)   lost_confidence_sum = 0;

    if (us_min < 800) us_min = 800;
    if (us_max > 2200) us_max = 2200;
    if (us_neu < us_min) us_neu = us_min;
    if (us_neu > us_max) us_neu = us_max;

    if (steer_min_us < us_min) steer_min_us = us_min;
    if (steer_max_us > us_max) steer_max_us = us_max;
    if (steer_min_us > steer_max_us) steer_min_us = steer_max_us;

    if (throttle_min < us_min) throttle_min = us_min;
    if (throttle_max > us_max) throttle_max = us_max;
    if (throttle_base < throttle_min) throttle_base = throttle_min;
    if (throttle_base > throttle_max) throttle_base = throttle_max;

    if (esc_arm_ms < 0) esc_arm_ms = 0;
    if (qtr_timeout_us < 500) qtr_timeout_us = 500;
    if (calibrate_ms < 500) calibrate_ms = 500;

    if (qtr_gamma < 0.3f) qtr_gamma = 0.3f;
    if (qtr_gamma > 2.5f) qtr_gamma = 2.5f;
    if (qtr_smooth_n < 1) qtr_smooth_n = 1;
    if (qtr_smooth_n > 5) qtr_smooth_n = 5;
    if ((qtr_smooth_n % 2)==0) qtr_smooth_n += 1; // force odd: 1,3,5
    if (min_line_contrast < 20) min_line_contrast = 20;
    if (min_line_contrast > 500) min_line_contrast = 500;
    if (deadband_error < 0) deadband_error = 0;
    if (deadband_error > 1500) deadband_error = 1500;
  }
} cfg;

/* ===================== Small helpers ===================== */
inline int clampUS(int us, int lo, int hi){
  if (us < lo) return lo; if (us > hi) return hi; return us;
}
String chipSuffix() {
  char buf[5];
  snprintf(buf, sizeof(buf), "%04X", (uint16_t)(ESP.getEfuseMac() & 0xFFFF));
  return String(buf);
}
void addCORS() {
  http.sendHeader("Access-Control-Allow-Origin", "*");
  http.sendHeader("Access-Control-Allow-Headers", "Content-Type");
  http.sendHeader("Access-Control-Allow-Methods", "GET,POST,OPTIONS");
}
void sendJSON(const String& s) { addCORS(); http.send(200, "application/json", s); }
void sendErr(int code, const String& msg) { addCORS(); http.send(code, "application/json", "{\"error\":\""+msg+"\"}"); }

/* ===================== QTR Helpers ===================== */
uint16_t readQTR_RC_1pin(int pin, uint16_t timeoutUs) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  pinMode(pin, INPUT);
  uint32_t start = micros();
  while (digitalRead(pin) == HIGH) {
    if ((uint32_t)(micros() - start) >= timeoutUs) return timeoutUs;
  }
  return (uint16_t)(micros() - start);
}
inline uint16_t normalize(uint16_t raw, uint16_t rmin, uint16_t rmax){
  if (raw < rmin) raw = rmin; if (raw > rmax) raw = rmax;
  return (uint16_t)(((uint32_t)(raw - rmin) * 1000UL) / (uint32_t)(rmax - rmin));
}
void qtrCalibrate(unsigned long ms){
  Serial.println("[QTR] Calibrating... move across line & background");
  for (int i=0;i<NUM_SENSORS;i++){ calMin[i]=0xFFFF; calMax[i]=0; }
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    for (int i=0;i<NUM_SENSORS;i++){
      uint16_t v = readQTR_RC_1pin(sensorPins[i], cfg.qtr_timeout_us);
      if (v < calMin[i]) calMin[i] = v;
      if (v > calMax[i]) calMax[i] = v;
    }
    delay(5);
  }
  for (int i=0;i<NUM_SENSORS;i++){
    if (calMax[i] <= calMin[i]) calMax[i] = calMin[i] + 1;
  }
  Serial.println("[QTR] Calibration done");
}
void setSteerFromError(long error){
  // deadband to reduce twitch
  if (abs(error) <= cfg.deadband_error) error = 0;
  int offset = (int)(cfg.Kp * error);   // µs offset around neutral
  int target = clampUS(cfg.us_neu + offset, cfg.steer_min_us, cfg.steer_max_us);
  steer.writeMicroseconds(target);
  lastSteerUS = target;
}
void setThrottleByErrorAndConf(long absErr, int confidence){
  int us = cfg.throttle_base;
  if (confidence < cfg.lost_confidence_sum) {
    us = cfg.throttle_min;
  } else if (absErr > cfg.error_slowdown_thresh) {
    int delta = map(absErr, cfg.error_slowdown_thresh, 6500, 0, (cfg.throttle_base - cfg.throttle_min));
    us = cfg.throttle_base - delta;
  }
  us = clampUS(us, cfg.throttle_min, cfg.throttle_max);
  throttle.writeMicroseconds(us);
  lastThrottleUS = us;
}

/* ===== Teleop mapper ===== */
void setTeleopFromXY(float x, float y) {
  // clamp normalized inputs
  if (x < -1) x = -1; if (x >  1) x = 1;
  if (y < -1) y = -1; if (y >  1) y = 1;

  // Steering around neutral within
  int steerSpanL = cfg.us_neu - cfg.steer_min_us;
  int steerSpanR = cfg.steer_max_us - cfg.us_neu;
  int steerDelta = (x < 0) ? (int)lroundf(x * steerSpanL) : (int)lroundf(x * steerSpanR);
  int steerTarget = clampUS(cfg.us_neu + steerDelta, cfg.steer_min_us, cfg.steer_max_us);

  // Throttle: y>0 => forward up to throttle_max; y<0 => reverse down to throttle_min
  int thrForward = cfg.throttle_max - cfg.us_neu;
  int thrReverse = cfg.us_neu - cfg.throttle_min;
  int thrDelta   = (y >= 0) ? (int)lroundf(y * thrForward) : (int)lroundf(y * -thrReverse);
  int throttleTarget = clampUS(cfg.us_neu + thrDelta, cfg.throttle_min, cfg.throttle_max);

  teleopSteerUS = steerTarget;
  teleopThrottleUS = throttleTarget;
  lastTeleopMs = millis();
}

/* ===================== ADDITIONS FOR QTR ===================== */
// Drive emitters explicitly
static inline void qtrEmitters(bool leftOn, bool rightOn){
  digitalWrite(EMITTER1_PIN, leftOn  ? HIGH : LOW);
  digitalWrite(EMITTER2_PIN, rightOn ? HIGH : LOW);
  delayMicroseconds(50); // settle LEDs
}

// Read ambient-subtracted value with a chosen emitter state
static uint16_t readQTR_RC_delta(int pin, uint16_t timeoutUs, bool leftOn, bool rightOn){
  qtrEmitters(false, false);
  uint16_t amb = readQTR_RC_1pin(pin, timeoutUs);
  qtrEmitters(leftOn, rightOn);
  uint16_t on  = readQTR_RC_1pin(pin, timeoutUs);
  return (on > amb) ? (uint16_t)(on - amb) : 0;
}

// Temporal median (N=3) ring buffer
static const int TEMP_N = 3;
static uint16_t temp_hist[NUM_SENSORS][TEMP_N];
static uint8_t  temp_idx = 0;

static inline uint16_t median3(uint16_t a, uint16_t b, uint16_t c){
  if (a > b) { uint16_t t=a; a=b; b=t; }
  if (b > c) { uint16_t t=b; b=c; c=t; }
  if (a > b) { uint16_t t=a; a=b; b=t; }
  return b;
}
static void temporalPush(const uint16_t v[NUM_SENSORS]){
  for(int i=0;i<NUM_SENSORS;i++) temp_hist[i][temp_idx] = v[i];
  temp_idx = (temp_idx + 1) % TEMP_N;
}
static void temporalMedian(uint16_t out[NUM_SENSORS]){
  for(int i=0;i<NUM_SENSORS;i++)
    out[i] = median3(temp_hist[i][0], temp_hist[i][1], temp_hist[i][2]);
}

// Contrast = mean(top3) - mean(bottom3)
static int contrastTop3MinusBottom3(const uint16_t v[NUM_SENSORS]){
  uint16_t min1=1000,min2=1000,min3=1000, max1=0,max2=0,max3=0;
  for(int i=0;i<NUM_SENSORS;i++){
    uint16_t x=v[i];
    if (x>max1){ max3=max2; max2=max1; max1=x; }
    else if (x>max2){ max3=max2; max2=x; }
    else if (x>max3){ max3=x; }
    if (x<min1){ min3=min2; min2=min1; min1=x; }
    else if (x<min2){ min3=min2; min2=x; }
    else if (x<min3){ min3=x; }
  }
  int topMean = (max1+max2+max3)/3;
  int botMean = (min1+min2+min3)/3;
  return topMean - botMean;
}

// Robust centroid over bright band only
static long robustCentroid(const uint16_t v[NUM_SENSORS], int band, bool &hasLine){
  uint16_t vmax = 0;
  for (int i=0;i<NUM_SENSORS;i++) if (v[i] > vmax) vmax = v[i];
  int thr = (int)vmax - band; if (thr < 0) thr = 0;

  long num = 0, den = 0;
  for (int i=0;i<NUM_SENSORS;i++){
    if (v[i] >= thr){
      int pos = i*1000 - 6500;
      num += (long)pos * (long)v[i];
      den += v[i];
    }
  }
  if (den == 0) { hasLine=false; return 0; }
  hasLine=true;
  return num / den;
}

static void buildSensorOrder() {
#if SWAP_BLOCKS
#endif
  sensorPins[0]  = 17;
  sensorPins[1]  = 16;
  sensorPins[2]  = 15;
  sensorPins[3]  = 7;
  sensorPins[4]  = 6;
  sensorPins[5]  = 5;
  sensorPins[6]  = 4;
  sensorPins[7]  = 12;
  sensorPins[8]  = 11;
  sensorPins[9]  = 10;
  sensorPins[10] = 9;
  sensorPins[11] = 3;
  sensorPins[12] = 8;
  sensorPins[13] = 18;
}

/* ===================== Wi-Fi ===================== */
void startAP() {
  WiFi.mode(WIFI_AP_STA);
  String ssid = "PaceBot-" + chipSuffix();
  bool ok = WiFi.softAP(ssid.c_str(), AP_PASS, 6);
  Serial.printf("[AP] SSID:%s  PASS:%s  IP:%s  %s\n",
                ssid.c_str(), AP_PASS, WiFi.softAPIP().toString().c_str(), ok ? "OK" : "FAIL");
}
bool startSTA(const String& ssid, const String& pass) {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(MDNS_NAME);
  WiFi.begin(ssid.c_str(), pass);
  Serial.printf("[STA] Connecting to %s ...\n", ssid.c_str());
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(250); Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[STA] OK  IP:%s  RSSI:%d\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
    if (MDNS.begin(MDNS_NAME)) {
      MDNS.addService("http", "tcp", 80);
      MDNS.addService("ws",   "tcp", WS_PORT);
      Serial.printf("[mDNS] http://%s.local  ws://%s.local:%u\n", MDNS_NAME, MDNS_NAME, WS_PORT);
    }
    return true;
  }
  Serial.println("[STA] FAIL");
  return false;
}

/* ===================== HTTP ===================== */
void handleRoot() {
  addCORS();
  String html =
    "<h3>PaceBot</h3>"
    "<p>Use /wifi/scan and /wifi/connect to provision. Use /config to tune.</p>"
    "<pre>Mode: " +
    String((WiFi.isConnected() ? "STA " + WiFi.localIP().toString() : "AP " + WiFi.softAPIP().toString())) +
    "</pre>";
  http.send(200, "text/html", html);
}
void handleScan() {
  if (WiFi.getMode() != WIFI_AP_STA) WiFi.mode(WIFI_AP_STA);
  int n = WiFi.scanNetworks(false, false);
  DynamicJsonDocument doc(1536);
  JsonArray arr = doc.to<JsonArray>();
  for (int i = 0; i < n; i++) {
    JsonObject o = arr.createNestedObject();
    o["ssid"] = WiFi.SSID(i);
    o["rssi"] = WiFi.RSSI(i);
    o["enc"]  = WiFi.encryptionType(i);
  }
  String out; serializeJson(arr, out);
  sendJSON(out);
}
void handleConnectWiFi() {
  if (http.method() == HTTP_OPTIONS) { addCORS(); http.send(204); return; }
  if (!http.hasArg("plain")) return sendErr(400, "no body");
  DynamicJsonDocument doc(256);
  if (deserializeJson(doc, http.arg("plain"))) return sendErr(400, "bad json");
  String ssid = doc["ssid"] | "";
  String pass = doc["password"] | "";
  if (ssid == "") return sendErr(400, "ssid required");
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  bool ok = startSTA(ssid, pass);
  DynamicJsonDocument resp(256);
  resp["ok"]   = ok;
  resp["ip"]   = WiFi.localIP().toString();
  resp["host"] = String(MDNS_NAME) + ".local";
  resp["ws"]   = String("ws://") + (ok ? WiFi.localIP().toString() : WiFi.softAPIP().toString()) + ":" + String(WS_PORT);
  String out; serializeJson(resp, out);
  sendJSON(out);
}
void handleStatus() {
  DynamicJsonDocument doc(256);
  const char* m =
    mode==IDLE ? "idle" :
    mode==FOLLOW_TRACK ? "follow_track" :
    mode==PACE_ATHLETE ? "pace_athlete" :
    mode==MAP_TRACK ? "map_track" : "teleop"; // include teleop
  bool sta = (WiFi.getMode()==WIFI_STA && WiFi.isConnected());
  String ip = sta ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
  doc["mode"] = m;
  doc["wifi"] = sta ? "sta" : "ap";
  doc["ip"]   = ip;
  doc["host"] = String(MDNS_NAME) + ".local";
  doc["ws"]   = String("ws://") + ip + ":" + String(WS_PORT);
  String out; serializeJson(doc, out);
  sendJSON(out);
}

/* ======== Config endpoints ======== */
void handleGetConfig() {
  DynamicJsonDocument d(1024);
  JsonObject o = d.to<JsonObject>();
  cfg.toJson(o);
  String s; serializeJsonPretty(d, s);
  sendJSON(s);
}
void handlePostConfig() {
  if (http.method() == HTTP_OPTIONS) { addCORS(); http.send(204); return; }
  if (!http.hasArg("plain")) return sendErr(400, "no body");
  DynamicJsonDocument d(1024);
  DeserializationError e = deserializeJson(d, http.arg("plain"));
  if (e) return sendErr(400, "bad json");
  JsonObject o = d.as<JsonObject>();
  cfg.fromJson(o);            // merge + clamp
  cfg.save(prefs);            // persist
  DynamicJsonDocument out(1024);
  cfg.toJson(out.to<JsonObject>());
  String s; serializeJson(out, s);
  sendJSON(s);
}
void handleResetConfig() {
  if (http.method() == HTTP_OPTIONS) { addCORS(); http.send(204); return; }
  cfg.setDefaults();
  cfg.save(prefs);
  DynamicJsonDocument out(1024);
  cfg.toJson(out.to<JsonObject>());
  String s; serializeJson(out, s);
  sendJSON(s);
}

/* ===================== WebSocket ===================== */
void wsSendAck(const char* cmd) {
  DynamicJsonDocument a(96); a["ok"]=true; a["cmd"]=cmd;
  String out; serializeJson(a, out); ws.broadcastTXT(out);
}
void handleWSCmd(const JsonDocument& cmd) {
  const char* c = cmd["cmd"] | "";
  if (!strcmp(c, "set_mode")) {
    String v = cmd["value"] | "idle";
    if      (v == "idle")         mode = IDLE;
    else if (v == "follow_track") mode = FOLLOW_TRACK;
    else if (v == "pace_athlete") mode = PACE_ATHLETE;
    else if (v == "map_track")    mode = MAP_TRACK;
    else if (v == "teleop")       mode = TELEOP;
    wsSendAck("set_mode");
  } else if (!strcmp(c, "upload_splits")) {
    String s; serializeJson(cmd["splits"], s);
    prefs.putString("splits", s);
    wsSendAck("upload_splits");
  } else if (!strcmp(c, "teleop")) {
    float x = cmd["x"] | 0.0;
    float y = cmd["y"] | 0.0;
    setTeleopFromXY(x, y);
    wsSendAck("teleop");
  }
}
void onWSEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t len) {
  if (type == WStype_CONNECTED) {
    Serial.printf("[WS] client %u connected\n", num);
  } else if (type == WStype_TEXT) {
    DynamicJsonDocument doc(512);
    if (!deserializeJson(doc, payload, len)) handleWSCmd(doc);
  } else if (type == WStype_DISCONNECTED) {
    Serial.printf("[WS] client %u disconnected\n", num);
  }
}

/* ===================== Telemetry & Control ===================== */
void wsBroadcastTelemetry() {
  // ===== Ambient-aware + sequential emitters + temporal median =====
  uint16_t frame[NUM_SENSORS];
  uint16_t smoothed[NUM_SENSORS];
  int confidence = 0;

  for (int i=0;i<7;i++){
    uint16_t raw = readQTR_RC_delta(sensorPins[i], cfg.qtr_timeout_us, /*left*/true, /*right*/false);
    uint16_t v   = normalize(raw, calMin[i], calMax[i]);
    if (cfg.white_line) v = 1000 - v;
    if (cfg.qtr_gamma != 1.0f) {
      float f = powf(v / 1000.0f, cfg.qtr_gamma);
      v = (uint16_t)(f * 1000.0f + 0.5f);
    }
    frame[i] = v;
  }

  for (int i=7;i<14;i++){
    uint16_t raw = readQTR_RC_delta(sensorPins[i], cfg.qtr_timeout_us, /*left*/false, /*right*/true);
    uint16_t v   = normalize(raw, calMin[i], calMax[i]);
    if (cfg.white_line) v = 1000 - v;
    if (cfg.qtr_gamma != 1.0f) {
      float f = powf(v / 1000.0f, cfg.qtr_gamma);
      v = (uint16_t)(f * 1000.0f + 0.5f);
    }
    frame[i] = v;
  }

  qtrEmitters(true, true);

  temporalPush(frame);
  temporalMedian(smoothed);

  // Confidence & contrast
  for (int i=0;i<NUM_SENSORS;i++) confidence += smoothed[i];
  int contrast = contrastTop3MinusBottom3(smoothed);
  bool lineOk  = (contrast >= cfg.min_line_contrast);

  // Robust centroid around bright band
  bool hasLine=false;
  long error = robustCentroid(smoothed, /*band=*/200, hasLine);
  long absErr = labs(error);

  // ---- Control dispatch (mode-based) ----
  bool arming = (millis() - bootMs < cfg.esc_arm_ms);

  if (mode == FOLLOW_TRACK) {
    if (!lineOk || !hasLine) {
      // Lost line → steer neutral, slow
      steer.writeMicroseconds(cfg.us_neu);
      throttle.writeMicroseconds(arming ? cfg.us_neu : cfg.throttle_min);
      lastSteerUS = cfg.us_neu;
      lastThrottleUS = arming ? cfg.us_neu : cfg.throttle_min;
    } else {
      setSteerFromError(error);
      if (arming) {
        throttle.writeMicroseconds(cfg.us_neu);
        lastThrottleUS = cfg.us_neu;
      } else {
        setThrottleByErrorAndConf(absErr, confidence);
      }
    }
  } else if (mode == TELEOP) {
    // Safety: neutral if command stale or still arming
    uint32_t nowms = millis();
    if (nowms - lastTeleopMs > TELEOP_TIMEOUT_MS || arming) {
      teleopSteerUS = cfg.us_neu;
      teleopThrottleUS = cfg.us_neu;
    }
    steer.writeMicroseconds(teleopSteerUS);
    throttle.writeMicroseconds(teleopThrottleUS);
    lastSteerUS = teleopSteerUS;
    lastThrottleUS = teleopThrottleUS;
  } else {
    // IDLE / PACE_ATHLETE / MAP_TRACK -> hold neutral for now
    steer.writeMicroseconds(cfg.us_neu);
    throttle.writeMicroseconds(cfg.us_neu);
    lastSteerUS = cfg.us_neu;
    lastThrottleUS = cfg.us_neu;
  }

  // IMU sample
  bool imu_ok = imuRead6500();

  // Convert error units to centimeters (1000 units ≈ 4 mm)
  float line_error_cm = (!lineOk || !hasLine) ? NAN : (error * 0.0004f);

  // JSON
  DynamicJsonDocument t(1700);
  t["type"]       = "telemetry";
  t["ts"]         = (uint32_t)millis();
  t["mode"]       =
    mode==IDLE ? "idle" :
    mode==FOLLOW_TRACK ? "follow_track" :
    mode==PACE_ATHLETE ? "pace_athlete" :
    mode==MAP_TRACK ? "map_track" : "teleop";

  // Diagnostics
  t["uptime_ms"]  = (uint32_t)millis();
  t["free_heap"]  = (uint32_t)ESP.getFreeHeap();
  if (WiFi.status() == WL_CONNECTED) t["rssi"] = WiFi.RSSI();

  // RC/QTR
  if (!lineOk || !hasLine) t["line_error_cm"] = nullptr;
  else                      t["line_error_cm"] = line_error_cm;
  t["qtr_quality"]   = confidence;    // 0..14000 (approx; lower on wood)
  t["steer_us"]      = lastSteerUS;
  t["throttle_us"]   = lastThrottleUS;

  JsonArray qn = t.createNestedArray("qtr_norm");
  for (int i=0;i<NUM_SENSORS;i++) qn.add((int)smoothed[i]);

  // IMU
  if (imu_ok) {
    t["ax_ms2"]      = ax_ms2;
    t["ay_ms2"]      = ay_ms2;
    t["az_ms2"]      = az_ms2;
    t["gx_rad"]      = gx_rad;
    t["gy_rad"]      = gy_rad;
    t["gz_rad"]      = gz_rad;
    t["imu_temp_c"]  = imu_temp_c;
    t["imu_yaw_deg"] = imu_yaw_deg;
  } else {
    t["ax_ms2"] = nullptr; t["ay_ms2"] = nullptr; t["az_ms2"] = nullptr;
    t["gx_rad"] = nullptr; t["gy_rad"] = nullptr; t["gz_rad"] = nullptr;
    t["imu_temp_c"] = nullptr; t["imu_yaw_deg"] = nullptr;
  }

  // Encoder-derived kinematics
  t["speed_mps"]   = speed_mps_curr;
  t["distance_m"]  = distance_m_accum;

  // Battery placeholder (wire in later)
  t["battery_v"]   = 7.6;

  // Include current config
  JsonObject co = t.createNestedObject("config");
  cfg.toJson(co);

  String out; serializeJson(t, out);
  ws.broadcastTXT(out);
}

/* ===================== Setup / Loop ===================== */
void setup() {
  Serial.begin(115200);
  delay(200);
  bootMs = millis();

  buildSensorOrder();
  pinMode(EMITTER1_PIN, OUTPUT);
  pinMode(EMITTER2_PIN, OUTPUT);
  digitalWrite(EMITTER1_PIN, HIGH);
  digitalWrite(EMITTER2_PIN, HIGH);

  // Encoder input
  pinMode(ENC_PWM_PIN, INPUT);

  // RC outputs
  steer.setPeriodHertz(50);
  throttle.setPeriodHertz(50);
  steer.attach(STEER_PIN, 800, 2200);
  throttle.attach(THROTTLE_PIN, 800, 2200);

  // Preferences + Config
  prefs.begin("pacebot", false);
  cfg.setDefaults();
  (void)cfg.load(prefs);   // ignore false; defaults already set
  cfg.clampAndFix();

  // Neutral on boot
  steer.writeMicroseconds(cfg.us_neu);
  throttle.writeMicroseconds(cfg.us_neu);

  // Wi-Fi: try STA → else AP
  String ssid = prefs.getString("ssid", "");
  String pass = prefs.getString("pass", "");
  bool staOK = false;
  if (ssid.length()) staOK = startSTA(ssid, pass);
  if (!staOK) startAP();

  // HTTP routes

  // ---- Wi-Fi routes ----
  http.on("/wifi/scan",    HTTP_OPTIONS, [](){ addCORS(); http.send(204); });   // <-- add
  http.on("/wifi/scan",    HTTP_GET,     handleScan);

  http.on("/wifi/status",  HTTP_OPTIONS, [](){ addCORS(); http.send(204); });   // <-- add
  http.on("/wifi/status",  HTTP_GET,     handleStatus);

  http.on("/wifi/connect", HTTP_OPTIONS, [](){ addCORS(); http.send(204); });   // already there
  http.on("/wifi/connect", HTTP_POST,    handleConnectWiFi);

  http.on("/wifi/forget",  HTTP_OPTIONS, [](){ addCORS(); http.send(204); });   // <-- add
  http.on("/wifi/forget",  HTTP_POST, []() {
    addCORS();
    prefs.clear();
    http.send(200, "application/json", "{\"ok\":true,\"msg\":\"WiFi credentials cleared\"}");
    Serial.println("[WiFi] Cleared stored credentials");
  });


  // Config endpoints
  http.on("/config",       HTTP_OPTIONS, [](){ addCORS(); http.send(204); });
  http.on("/config",       HTTP_GET,     handleGetConfig);
  http.on("/config",       HTTP_POST,    handlePostConfig);
  http.on("/config/reset", HTTP_OPTIONS, [](){ addCORS(); http.send(204); });
  http.on("/config/reset", HTTP_POST,    handleResetConfig);

  // catch-all
  http.onNotFound([](){
    if (http.method() == HTTP_OPTIONS) { addCORS(); http.send(204); return; }
    String path = http.uri();
    addCORS();
    http.send(404, "text/plain", "Not found: " + path);
    Serial.printf("[HTTP 404] %s\n", path.c_str());
  });
  http.begin();

  // WebSocket
  ws.begin(); // :81
  ws.onEvent(onWSEvent);

  // IMU
  (void)imuBegin6500();

  // QTR calibration
  qtrCalibrate(cfg.calibrate_ms);

  Serial.println("[SETUP] Ready");
}

void loop() {
  http.handleClient();
  ws.loop();

  // Update encoder-derived speed & distance
  updateEncoder();

  uint32_t now = millis();
  if (now - lastTelem >= 50) { lastTelem = now; wsBroadcastTelemetry(); }
}
