// ESP32-S3 + Dual Pololu QTRX-HD-07RC + Servo + ESC
// Two-phase calibration and line following.

#include <Arduino.h>
#include <ESP32Servo.h>

/* ---------------- Pins ---------------- */
#define EMITTER1_PIN 1
#define EMITTER2_PIN 2

const int NUM_SENSORS = 14;
int sensorPins[NUM_SENSORS] = {
  17,16,15,7,6,5,4,   // S0 to S6  (left board, L->R)
  12,11,10,9,3,8,18   // S7 to S13 (right board, L->R)
};

const int IGNORE_SENSOR_INDEX = 7;

#define STEER_PIN    36
#define THROTTLE_PIN 35

Servo steer;
Servo throttle;

/* ---------------- Config ---------------- */
const uint16_t TIMEOUT_US     = 3000;
const uint32_t CAL_PHASE_MS   = 7000;
const int      MIN_SPREAD     = 150;
const int      DECISION_DEADBAND_ERR = 400;

const bool INVERT_STEERING    = false;
const bool FORCE_INVERT_WHITE = true;

/* --- Steering / throttle --- */
int steer_min_us   = 1100;
int steer_neu_us   = 1500;
int steer_max_us   = 1900;

int throttle_min_us    = 1500;  // ESC neutral
int throttle_cruise_us = 1580;  // slow forward
int throttle_max_us    = 1700;

const int   STEER_MAX_OFFSET_US = 350;
const int   STEER_DEADBAND_US   = 12;

const float ERR_FILTER_ALPHA    = 0.35f;
float       errorFilt           = 0.0f;

const int   ERROR_SLOWDOWN_THRESH = 2000;

/* ---------------- Storage ---------------- */
uint16_t whiteCal[NUM_SENSORS];
uint16_t brownCal[NUM_SENSORS];
bool     sensorBad[NUM_SENSORS];

/* ---------------- Helpers ---------------- */
static inline void emitters(bool leftOn, bool rightOn){
  digitalWrite(EMITTER1_PIN, leftOn  ? HIGH : LOW);
  digitalWrite(EMITTER2_PIN, rightOn ? HIGH : LOW);
  delayMicroseconds(50);
}

uint16_t readQTR_RC_1pin(int pin, uint16_t timeoutUs) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  pinMode(pin, INPUT);
  uint32_t t0 = micros();
  while (digitalRead(pin) == HIGH) {
    if ((uint32_t)(micros() - t0) >= timeoutUs) return timeoutUs;
  }
  return (uint16_t)(micros() - t0);
}

// Ambient-subtracted RC time for one sensor
uint16_t readDelta(int pin, bool leftOn, bool rightOn) {
  emitters(false,false);
  uint16_t amb = readQTR_RC_1pin(pin, TIMEOUT_US);
  emitters(leftOn,rightOn);
  uint16_t on  = readQTR_RC_1pin(pin, TIMEOUT_US);
  return (on > amb) ? (uint16_t)(on - amb) : 0;
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

/* ---------------- Calibration ---------------- */
// Average RC times over CAL_PHASE_MS for one surface (white or brown)
void doPhaseCalibration(uint16_t calOut[NUM_SENSORS], const char* phaseName) {
  uint32_t sum[NUM_SENSORS];
  for (int i=0;i<NUM_SENSORS;i++) sum[i] = 0;
  uint32_t samples = 0;

  Serial.println();
  Serial.println("-------------------------------------------");
  Serial.print("Phase: ");
  Serial.println(phaseName);
  Serial.println("Keep ALL sensors over that surface.");
  Serial.println("-------------------------------------------");

  unsigned long t0 = millis();
  while (millis() - t0 < CAL_PHASE_MS) {
    for (int i=0;i<7;i++) {
      uint16_t v = readDelta(sensorPins[i], true,false);
      sum[i] += v;
    }
    for (int i=7;i<14;i++) {
      uint16_t v = readDelta(sensorPins[i], false,true);
      sum[i] += v;
    }
    samples++;
    delay(2);
  }

  for (int i=0;i<NUM_SENSORS;i++) {
    calOut[i] = (uint16_t)(sum[i] / max<uint32_t>(samples,1));
  }

  Serial.print("Done ");
  Serial.println(phaseName);
  for (int i=0;i<NUM_SENSORS;i++) {
    Serial.print("S"); Serial.print(i);
    Serial.print(": ");
    Serial.println(calOut[i]);
  }
  Serial.println();
}

/* ---------------- Setup ---------------- */
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(EMITTER1_PIN, OUTPUT);
  pinMode(EMITTER2_PIN, OUTPUT);
  emitters(true,true);

  steer.setPeriodHertz(50);
  throttle.setPeriodHertz(50);
  steer.attach(STEER_PIN, 800, 2200);
  throttle.attach(THROTTLE_PIN, 800, 2200);
  steer.writeMicroseconds(steer_neu_us);
  throttle.writeMicroseconds(throttle_min_us);

  Serial.println("QTRX line follower with 2-phase calibration");
  Serial.println("1) Place ALL sensors over WHITE line.");
  Serial.println("   Press ENTER in Serial Monitor to start phase 1.\n");

  while (!Serial.available()) delay(10);
  while (Serial.available()) Serial.read();

  doPhaseCalibration(whiteCal, "White calibration (WHITE LINE)");

  Serial.println("Now place ALL sensors over BROWN ground (no line).");
  Serial.println("Press ENTER in Serial Monitor to start phase 2.\n");
  while (!Serial.available()) delay(10);
  while (Serial.available()) Serial.read();

  doPhaseCalibration(brownCal, "Brown calibration (BROWN GROUND)");

  // Flag sensors with almost no difference between white and brown as bad
  Serial.println("Detecting bad sensors (low white/brown difference)...");
  for (int i=0;i<NUM_SENSORS;i++) {
    uint16_t w = whiteCal[i];
    uint16_t b = brownCal[i];
    uint16_t diff = (w > b) ? (w - b) : (b - w);

    if (i == IGNORE_SENSOR_INDEX) {
      sensorBad[i] = true;
    } else {
      sensorBad[i] = (diff < 2);
    }

    Serial.print("S"); Serial.print(i);
    Serial.print("  whiteCal="); Serial.print(w);
    Serial.print("  brownCal="); Serial.print(b);
    Serial.print("  diff="); Serial.print(diff);
    Serial.print("  -> ");
    Serial.println(sensorBad[i] ? "BAD/IGNORED" : "OK");
  }
  Serial.println();
}

/* ---------------- Loop ---------------- */
void loop() {
  uint16_t rawDelta[NUM_SENSORS];
  uint16_t whiteness[NUM_SENSORS];  // 0..1000, 0 = brown, 1000 = white

  int steerTarget    = steer_neu_us;
  int throttleTarget = throttle_min_us;

  // Read all sensors
  for (int i=0;i<7;i++) {
    rawDelta[i] = readDelta(sensorPins[i], true,false);
  }
  for (int i=7;i<14;i++) {
    rawDelta[i] = readDelta(sensorPins[i], false,true);
  }
  emitters(true,true);

  uint16_t minW = 1000, maxW = 0;

  for (int i=0;i<NUM_SENSORS;i++) {
    if (sensorBad[i]) {
      whiteness[i] = 0;
      continue;
    }

    int32_t wCal = (int32_t)whiteCal[i];
    int32_t bCal = (int32_t)brownCal[i];
    int32_t span = wCal - bCal;  // may be positive or negative

    if (span == 0) {
      whiteness[i] = 0;
    } else {
      int32_t raw = (int32_t)rawDelta[i];
      int32_t num;

      if (span > 0) {
        num = raw - bCal;
      } else {
        num = bCal - raw;
        span = -span;
      }

      if (num < 0)    num = 0;
      if (num > span) num = span;

      uint16_t wNorm = (uint16_t)((num * 1000L) / span);
      if (FORCE_INVERT_WHITE) wNorm = 1000 - wNorm;
      whiteness[i] = wNorm;
    }

    if (whiteness[i] < minW) minW = whiteness[i];
    if (whiteness[i] > maxW) maxW = whiteness[i];
  }

  uint16_t spread = maxW - minW;

  enum Cell { CELL_BROWN, CELL_WHITE, CELL_UNKNOWN };
  Cell baseClass[NUM_SENSORS];
  const char* decision = "NO LINE";
  long errorRaw = 0;

  // If spread is too small, assume no reliable line
  if (spread < MIN_SPREAD) {
    Serial.println("[no line]");
    Serial.print("Decision: NO LINE (spread low: ");
    Serial.print(spread);
    Serial.println(")");
  } else {
    // Dynamic thresholds based on current min/max
    uint16_t tLow  = minW + spread / 3;
    uint16_t tHigh = minW + (spread * 2) / 3;

    int firstWhite = -1;
    int lastWhite  = -1;

    // Classify each sensor as white / brown / unknown
    for (int i=0;i<NUM_SENSORS;i++) {
      if (sensorBad[i]) {
        baseClass[i] = CELL_UNKNOWN;
        continue;
      }

      uint16_t w = whiteness[i];
      if (w >= tHigh) {
        baseClass[i] = CELL_WHITE;
        if (firstWhite < 0) firstWhite = i;
        lastWhite = i;
      } else if (w <= tLow) {
        baseClass[i] = CELL_BROWN;
      } else {
        baseClass[i] = CELL_UNKNOWN;
      }
    }

    if (firstWhite < 0) {
      Serial.println("[no line]");
      Serial.println("Decision: NO LINE (no strong line segment)");
    } else {
      // Enforce a single continuous line segment
      for (int i=firstWhite; i<=lastWhite; i++) {
        if (baseClass[i] == CELL_BROWN) baseClass[i] = CELL_UNKNOWN;
      }

      // Compute weighted line position error
      long num = 0;
      long den = 0;
      for (int i=0;i<NUM_SENSORS;i++) {
        int pos = i * 1000 - 6500;  // -6500..+6500 across array
        int weight = (baseClass[i] == CELL_WHITE) ? whiteness[i] : 0;
        num += (long)pos * (long)weight;
        den += weight;
      }
      if (den != 0) errorRaw = num / den;

      // Filter error to reduce jitter
      float e = (float)errorRaw;
      if (INVERT_STEERING) e = -e;
      errorFilt = errorFilt + ERR_FILTER_ALPHA * (e - errorFilt);

      long decErr = (long)lroundf(errorFilt);

      if (den == 0) {
        decision = "NO LINE (no line cells)";
      } else if (abs(decErr) <= DECISION_DEADBAND_ERR) {
        decision = "GO STRAIGHT";
      } else if (decErr < 0) {
        decision = "TURN LEFT";
      } else {
        decision = "TURN RIGHT";
      }

      // Map filtered error to steering PWM
      float norm = constrain(errorFilt / 6500.0f, -1.0f, 1.0f);
      int steerOffset = (int)lroundf(norm * (float)STEER_MAX_OFFSET_US);
      if (abs(steerOffset) < STEER_DEADBAND_US) steerOffset = 0;

      steerTarget = steer_neu_us + steerOffset;
      if (steerTarget < steer_min_us) steerTarget = steer_min_us;
      if (steerTarget > steer_max_us) steerTarget = steer_max_us;

      // Throttle: slow down when error is large
      int absErr = abs(decErr);
      throttleTarget = throttle_cruise_us;
      if (absErr > ERROR_SLOWDOWN_THRESH) {
        int delta = map(absErr,
                        ERROR_SLOWDOWN_THRESH,
                        6500,
                        0,
                        (throttle_cruise_us - throttle_min_us));
        throttleTarget = throttle_cruise_us - delta;
      }
      if (throttleTarget < throttle_min_us) throttleTarget = throttle_min_us;
      if (throttleTarget > throttle_max_us) throttleTarget = throttle_max_us;

      // Symbolic visualisation of sensor states
      Serial.print("[");
      for (int i=0;i<NUM_SENSORS;i++){
        const char* label;
        if (sensorBad[i]) {
          label = "x";
        } else {
          switch (baseClass[i]) {
            case CELL_WHITE:   label = "||"; break;
            case CELL_BROWN:   label = "--"; break;
            case CELL_UNKNOWN: label = "?";  break;
            default:           label = "?";  break;
          }
        }
        Serial.print(label);
        if (i < NUM_SENSORS-1) Serial.print(", ");
      }
      Serial.println("]");

      // Print decision and key values
      Serial.print("Decision: ");
      Serial.print(decision);
      Serial.print("  (errorRaw=");
      Serial.print(errorRaw);
      Serial.print(", filtErr=");
      Serial.print(errorFilt);
      Serial.print(", minW=");
      Serial.print(minW);
      Serial.print(", maxW=");
      Serial.print(maxW);
      Serial.print(", spread=");
      Serial.print(spread);
      Serial.print(", steer_us=");
      Serial.print(steerTarget);
      Serial.print(", thr_us=");
      Serial.print(throttleTarget);
      Serial.println(")");
    }
  }

  // Numeric dumps for tuning
  Serial.print("rawDelta:  [");
  for (int i=0;i<NUM_SENSORS;i++){
    Serial.print(rawDelta[i]);
    if (i < NUM_SENSORS-1) Serial.print(", ");
  }
  Serial.println("]");

  Serial.print("whiteness: [");
  for (int i=0;i<NUM_SENSORS;i++){
    Serial.print(whiteness[i]);
    if (i < NUM_SENSORS-1) Serial.print(", ");
  }
  Serial.println("]  // 0..1000, 0=brown, 1000=white");

  Serial.println("------------------------------------------------");

  // Apply steering and throttle
  steer.writeMicroseconds(steerTarget);
  throttle.writeMicroseconds(throttleTarget);

  delay(80);
}
