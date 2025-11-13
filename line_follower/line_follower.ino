/*
  ESP32-S3 + Dual Pololu QTRX-HD-07RC (14 sensors) + Servo + ESC
  Precise WHITE line follower on brown surface.

    QTR #1 CTRL (tie ODD/EVEN) -> GPIO 1
    QTR #2 CTRL (tie ODD/EVEN) -> GPIO 2
    QTR #1 sensors L->R -> 17,16,15,7,6,5,4
    QTR #2 sensors L->R -> 12,11,10,9,3,8,18
    Steering servo signal -> GPIO 36
    ESC throttle signal   -> GPIO 35
    All GNDs common. QTR boards at 3.3 V.

  Behaviour:
    - 3 s calibration at boot (move over line & background).
    - White line on brown: invert after normalization.
    - If contrast poor or no line found -> steer neutral, throttle neutral.
    - P steering; throttle reduced as error grows.
*/

#include <Arduino.h>
#include <ESP32Servo.h>

/* ---------------- Pins ---------------- */
#define EMITTER1_PIN 1
#define EMITTER2_PIN 2
const int NUM_SENSORS = 14;
int sensorPins[NUM_SENSORS] = {17,16,15,7,6,5,4, 12,11,10,9,3,8,18};

#define STEER_PIN    36
#define THROTTLE_PIN 35

/* ---------------- Objects ---------------- */
Servo steer, throttle;

/* ---------------- Config (tune here) ---------------- */
const bool WHITE_LINE = true;         // white strip on brown
const bool STEER_REVERSED = false;    // flip if steering is backwards
const uint16_t TIMEOUT_US = 3000;
const uint32_t CAL_MS = 3000;

float Kp = 0.12f;                     // steering gain (µs per 1000 error units)
int steer_min_us = 1200, steer_neu_us = 1500, steer_max_us = 1800;

int throttle_min_us   = 1500;         // neutral / brake
int throttle_cruise_us= 1600;         // base forward
int throttle_max_us   = 1750;         // cap

// slow down as |error| grows
int error_slowdown_thresh = 2200;     // start slowing at this |error|
int deadband_error = 220;             // ignore small errors

// sensing robustness
float qtr_gamma = 0.75f;              // 1.0 linear; <1 boosts bright band
int   min_contrast = 120;             // top3-bottom3 threshold to trust line
const int TEMP_N = 3;                 // temporal median length (fixed 3)

/* ---------------- Storage ---------------- */
uint16_t calMin[NUM_SENSORS], calMax[NUM_SENSORS];
uint16_t hist[NUM_SENSORS][TEMP_N];
uint8_t  hidx = 0;

/* ---------------- Helpers ---------------- */
static inline void emitters(bool leftOn, bool rightOn){
  digitalWrite(EMITTER1_PIN, leftOn  ? HIGH : LOW);
  digitalWrite(EMITTER2_PIN, rightOn ? HIGH : LOW);
  delayMicroseconds(50);
}

uint16_t readQTR_RC_1pin(int pin, uint16_t timeoutUs) {
  pinMode(pin, OUTPUT); digitalWrite(pin, HIGH); delayMicroseconds(10);
  pinMode(pin, INPUT);
  uint32_t t0 = micros();
  while (digitalRead(pin) == HIGH) {
    if ((uint32_t)(micros()-t0) >= timeoutUs) return timeoutUs;
  }
  return (uint16_t)(micros()-t0);
}

uint16_t readDelta(int pin, bool leftOn, bool rightOn) {
  emitters(false,false);
  uint16_t amb = readQTR_RC_1pin(pin, TIMEOUT_US);
  emitters(leftOn,rightOn);
  uint16_t on  = readQTR_RC_1pin(pin, TIMEOUT_US);
  return (on > amb) ? (uint16_t)(on - amb) : 0;
}

inline uint16_t normalize1(uint16_t v, uint16_t rmin, uint16_t rmax){
  if (v < rmin) v = rmin; if (v > rmax) v = rmax;
  uint32_t span = (rmax>rmin)? (uint32_t)(rmax-rmin) : 1;
  return (uint16_t)(((uint32_t)(v - rmin) * 1000UL) / span); // 0..1000, dark=1000
}

static inline uint16_t median3(uint16_t a, uint16_t b, uint16_t c){
  if (a > b) { uint16_t t=a; a=b; b=t; }
  if (b > c) { uint16_t t=b; b=c; c=t; }
  if (a > b) { uint16_t t=a; a=b; b=t; }
  return b;
}

int contrastTop3MinusBottom3(const uint16_t *v, int n) {
  uint16_t max1=0,max2=0,max3=0, min1=1000,min2=1000,min3=1000;
  for (int i=0;i<n;i++){
    uint16_t x=v[i];
    if (x>max1){max3=max2;max2=max1;max1=x;}
    else if (x>max2){max3=max2;max2=x;}
    else if (x>max3){max3=x;}
    if (x<min1){min3=min2;min2=min1;min1=x;}
    else if (x<min2){min3=min2;min2=x;}
    else if (x<min3){min3=x;}
  }
  return (int)((max1+max2+max3)/3) - (int)((min1+min2+min3)/3);
}

/* ---------------- Setup ---------------- */
void setup() {
  Serial.begin(115200); delay(200);

  pinMode(EMITTER1_PIN, OUTPUT);
  pinMode(EMITTER2_PIN, OUTPUT);
  emitters(true,true);

  steer.setPeriodHertz(50);
  throttle.setPeriodHertz(50);
  steer.attach(STEER_PIN, 800, 2200);
  throttle.attach(THROTTLE_PIN, 800, 2200);
  steer.writeMicroseconds(steer_neu_us);
  throttle.writeMicroseconds(throttle_min_us);

  for (int i=0;i<NUM_SENSORS;i++){ calMin[i]=0xFFFF; calMax[i]=0; for(int k=0;k<TEMP_N;k++) hist[i][k]=0; }

  // Calibration
  Serial.println("Calibrating QTR (move over line & background)...");
  unsigned long t0 = millis();
  while (millis() - t0 < CAL_MS) {
    for (int i=0;i<7;i++){
      uint16_t v = readDelta(sensorPins[i], true,false);
      if (v < calMin[i]) calMin[i]=v; if (v > calMax[i]) calMax[i]=v;
    }
    for (int i=7;i<14;i++){
      uint16_t v = readDelta(sensorPins[i], false,true);
      if (v < calMin[i]) calMin[i]=v; if (v > calMax[i]) calMax[i]=v;
    }
    delay(5);
  }
  for (int i=0;i<NUM_SENSORS;i++) if (calMax[i] <= calMin[i]) calMax[i] = calMin[i]+1;
  Serial.println("Calibration done.");
}

/* ---------------- Loop ---------------- */
void loop() {
  uint16_t frame[NUM_SENSORS];

  // Read left half with left emitters only
  for (int i=0;i<7;i++) {
    uint16_t raw = readDelta(sensorPins[i], true,false);
    uint16_t v   = normalize1(raw, calMin[i], calMax[i]); // 0..1000, dark=1000
    frame[i]     = WHITE_LINE ? (uint16_t)(1000 - v) : v; // white=1000
  }
  // Read right half with right emitters only
  for (int i=7;i<14;i++) {
    uint16_t raw = readDelta(sensorPins[i], false,true);
    uint16_t v   = normalize1(raw, calMin[i], calMax[i]);
    frame[i]     = WHITE_LINE ? (uint16_t)(1000 - v) : v;
  }
  emitters(true,true);

  // Gamma for contrast shaping
  for (int i=0;i<NUM_SENSORS;i++) {
    float f = powf(frame[i]/1000.0f, qtr_gamma);
    frame[i] = (uint16_t)(f*1000.0f + 0.5f);
  }

  // Temporal median (3-sample) smoothing
  for (int i=0;i<NUM_SENSORS;i++) hist[i][hidx] = frame[i];
  uint16_t vals[NUM_SENSORS];
  for (int i=0;i<NUM_SENSORS;i++) vals[i] = median3(hist[i][0], hist[i][1], hist[i][2]);
  hidx = (hidx + 1) % TEMP_N;

  // Presence check
  int contrast = contrastTop3MinusBottom3(vals, NUM_SENSORS);
  long num=0, den=0;
  for (int i=0;i<NUM_SENSORS;i++){ int pos = i*1000 - 6500; num += (long)pos * vals[i]; den += vals[i]; }
  bool hasLine = (contrast >= min_contrast) && (den > 0);

  if (!hasLine) {
    steer.writeMicroseconds(steer_neu_us);
    throttle.writeMicroseconds(throttle_min_us);
    Serial.printf("No line: contrast=%d den=%ld\n", contrast, den);
    delay(25);
    return;
  }

  long error = num / den;                       // −6500..+6500
  if (abs(error) <= deadband_error) error = 0;

  int steerOffset = (int)lroundf(Kp * error);
  if (STEER_REVERSED) steerOffset = -steerOffset;
  int steerTarget = steer_neu_us + steerOffset;
  if (steerTarget < steer_min_us) steerTarget = steer_min_us;
  if (steerTarget > steer_max_us) steerTarget = steer_max_us;

  // Throttle schedule: slow down on large error
  int throttleTarget = throttle_cruise_us;
  int absErr = abs(error);
  if (absErr > error_slowdown_thresh) {
    int delta = map(absErr, error_slowdown_thresh, 6500, 0, (throttle_cruise_us - throttle_min_us));
    throttleTarget = throttle_cruise_us - delta;
  }
  if (throttleTarget < throttle_min_us) throttleTarget = throttle_min_us;
  if (throttleTarget > throttle_max_us) throttleTarget = throttle_max_us;

  steer.writeMicroseconds(steerTarget);
  throttle.writeMicroseconds(throttleTarget);

  // Debug every ~200 ms
  static uint32_t last=0; uint32_t now=millis();
  if (now-last>200) {
    Serial.printf("err=%ld  contrast=%d  steer=%d  thr=%d\n",
                  error, contrast, steerTarget, throttleTarget);
    last=now;
  }

  delay(25);
}
