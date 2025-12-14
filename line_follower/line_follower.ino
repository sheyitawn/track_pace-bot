// ESP32-S3 + Dual Pololu QTRX-HD-07RC (14 sensors) + Servo + ESC
// Simple 7s calibration + robust white-line-on-brown line following.

#include <Arduino.h>
#include <ESP32Servo.h>

/* ---------------- Pins ---------------- */
#define EMITTER1_PIN 1
#define EMITTER2_PIN 2

// LEDs for calibration status
#define LED_WHITE_PIN 19   // blinks during calibration
#define LED_BROWN_PIN 20   // ON when ready

const int NUM_SENSORS = 14;
int sensorPins[NUM_SENSORS] = {
  17,16,15,7,6,5,4,      // S0..S6  (left board, L->R)
  12,11,10,9,3,8,18      // S7..S13 (right board, L->R)
};

const int IGNORE_SENSOR_INDEX = 7; // S7 is known bad

// Steering + throttle (ESC)
#define STEER_PIN     36
#define THROTTLE_PIN  35

/* --------------- QTR RC timing config --------------- */
const uint16_t QTR_TIMEOUT_US = 2500;   // Pololu default-ish
uint16_t rawVals[NUM_SENSORS];          // "reflectance" values
uint16_t calMin[NUM_SENSORS];
uint16_t calMax[NUM_SENSORS];
uint16_t normVals[NUM_SENSORS];         // 0..1000 (higher = whiter)

/* --------------- Servo / ESC config --------------- */
Servo steerServo;
Servo throttleServo;

// <<< TUNE: steering range / center
const int STEER_CENTER_US = 1500;   // adjust to your straight-ahead value
const int STEER_RANGE_US  = 400;    // +/- range from center (1100..1900)

// <<< TUNE: ESC values
const int THROTTLE_STOP_US     = 1500;  // neutral
const int THROTTLE_FORWARD_US  = 1550;  // gentle forward
const int THROTTLE_SLOW_US     = 1520;  // slow crawl (for line loss)

/* --------------- Line-following control --------------- */
const long LINE_CENTER_POS = (NUM_SENSORS - 1) * 1000L / 2;  // 0..13000 → ~6500 center

// PD gains (error is in "sensor index * 1000" units)
// <<< TUNE: increase Kp for stronger turns, adjust Kd to reduce wobble
float Kp = 0.00040f;   // proportional
float Kd = 0.00120f;   // derivative

float lastError = 0.0f;

/* ===========================================================
   QTR Reading: RC timing, converted to "reflectance"
   Higher rawVals[i]  = more reflective (whiter)
   Lower rawVals[i]   = darker
   =========================================================== */
void readQTR(uint16_t *values) {
  // Turn on emitters
  digitalWrite(EMITTER1_PIN, HIGH);
  digitalWrite(EMITTER2_PIN, HIGH);

  // Charge all capacitors
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], OUTPUT);
    digitalWrite(sensorPins[i], HIGH);
  }

  delayMicroseconds(10); // charge time

  // Switch to input to start discharging
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  unsigned long start = micros();
  bool done[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) {
    values[i] = 0;
    done[i] = false;
  }

  while (true) {
    unsigned long now = micros();
    uint16_t elapsed = (uint16_t)(now - start);
    bool allDone = true;

    if (elapsed >= QTR_TIMEOUT_US) {
      // timeout: anything still high is at timeout
      for (int i = 0; i < NUM_SENSORS; i++) {
        if (!done[i]) {
          // "reflectance" = 0 at timeout (very dark)
          values[i] = 0;
          done[i] = true;
        }
      }
      break;
    }

    for (int i = 0; i < NUM_SENSORS; i++) {
      if (!done[i]) {
        int v = digitalRead(sensorPins[i]);
        if (v == LOW) {
          // Discharge time is "elapsed".
          // Convert to "reflectance": high reflect = small elapsed → large value.
          uint16_t reflect = (elapsed >= QTR_TIMEOUT_US) ? 0 : (QTR_TIMEOUT_US - elapsed);
          values[i] = reflect;
          done[i] = true;
        }
      }
    }

    for (int i = 0; i < NUM_SENSORS; i++) {
      if (!done[i]) {
        allDone = false;
        break;
      }
    }
    if (allDone) break;
  }

  // (Optional) Turn emitters off between reads if you want to save power
  // digitalWrite(EMITTER1_PIN, LOW);
  // digitalWrite(EMITTER2_PIN, LOW);
}

/* ===========================================================
   Calibration: 7 seconds total
   - Move robot so each sensor sees white and brown
   - LED_WHITE blinks during calibration
   - LED_BROWN turns ON when done
   =========================================================== */
void calibrateQTR() {
  // init calibration arrays
  for (int i = 0; i < NUM_SENSORS; i++) {
    calMin[i] = 65535;
    calMax[i] = 0;
  }

  unsigned long start = millis();
  const unsigned long CAL_TIME_MS = 7000;

  while (millis() - start < CAL_TIME_MS) {
    // Blink LED_WHITE about 4 Hz
    if (((millis() / 125) % 2) == 0) {
      digitalWrite(LED_WHITE_PIN, HIGH);
    } else {
      digitalWrite(LED_WHITE_PIN, LOW);
    }

    readQTR(rawVals);

    // Update calibration limits
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (i == IGNORE_SENSOR_INDEX) continue;

      uint16_t v = rawVals[i];
      if (v < calMin[i]) calMin[i] = v;
      if (v > calMax[i]) calMax[i] = v;
    }

    delay(5); // small delay to avoid hammering
  }

  // End of calibration: turn off WHITE LED, turn on BROWN LED
  digitalWrite(LED_WHITE_PIN, LOW);
  digitalWrite(LED_BROWN_PIN, HIGH);
}

/* ===========================================================
   Normalize sensor values to 0..1000
   - 0   -> darkest seen during calibration
   - 1000-> brightest seen during calibration (white line)
   =========================================================== */
void normalizeSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (i == IGNORE_SENSOR_INDEX) {
      normVals[i] = 0;
      continue;
    }

    uint16_t v = rawVals[i];
    uint16_t minV = calMin[i];
    uint16_t maxV = calMax[i];

    if (maxV <= minV) {
      normVals[i] = 0;
      continue;
    }

    long val = (long)(v - minV) * 1000L / (long)(maxV - minV);
    if (val < 0) val = 0;
    if (val > 1000) val = 1000;
    normVals[i] = (uint16_t)val;
  }
}

/* ===========================================================
   readLine():
   - Uses normalized values (higher = whiter = line)
   - Returns position in 0..(NUM_SENSORS-1)*1000
   - If no line detected, returns last known position
   =========================================================== */
long readLinePosition() {
  static long lastPos = LINE_CENTER_POS; // fallback if line lost

  long weightedSum = 0;
  long sum = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (i == IGNORE_SENSOR_INDEX) continue;

    uint16_t v = normVals[i]; // 0..1000, high = line
    long weight = i * 1000L;

    weightedSum += (long)v * weight;
    sum += v;
  }

  if (sum == 0) {
    // Line lost: return last known position
    return lastPos;
  }

  long pos = weightedSum / sum;
  lastPos = pos;
  return pos;
}

/* ===========================================================
   Apply steering + throttle based on line position
   =========================================================== */
void updateControl() {
  long pos = readLinePosition();

  // error: positive if line is to the LEFT or RIGHT?
  // Here: error = CENTER - pos
  // If your robot steers the wrong way, flip the sign below.
  long error = LINE_CENTER_POS - pos;

  float e = (float)error;
  float pTerm = Kp * e;
  float dTerm = Kd * (e - lastError);
  float control = pTerm + dTerm;
  lastError = e;

  // Clamp control to [-1, 1]
  if (control > 1.0f) control = 1.0f;
  if (control < -1.0f) control = -1.0f;

  // Steering pulse
  int steerPulse = STEER_CENTER_US + (int)(control * STEER_RANGE_US);
  steerPulse = constrain(steerPulse, STEER_CENTER_US - STEER_RANGE_US, STEER_CENTER_US + STEER_RANGE_US);

  // Basic line-loss detection: if total "whiteness" is tiny, slow down
  long sumWhiteness = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (i == IGNORE_SENSOR_INDEX) continue;
    sumWhiteness += normVals[i];
  }

  int throttlePulse;
  if (sumWhiteness < 2000) {
    // Likely off the line → slow crawl
    throttlePulse = THROTTLE_SLOW_US;
  } else {
    throttlePulse = THROTTLE_FORWARD_US;
  }

  steerServo.writeMicroseconds(steerPulse);
  throttleServo.writeMicroseconds(throttlePulse);
}

/* ===========================================================
   SETUP
   =========================================================== */
void setup() {
  Serial.begin(115200);

  pinMode(EMITTER1_PIN, OUTPUT);
  pinMode(EMITTER2_PIN, OUTPUT);
  digitalWrite(EMITTER1_PIN, LOW);
  digitalWrite(EMITTER2_PIN, LOW);

  pinMode(LED_WHITE_PIN, OUTPUT);
  pinMode(LED_BROWN_PIN, OUTPUT);
  digitalWrite(LED_WHITE_PIN, LOW);
  digitalWrite(LED_BROWN_PIN, LOW);

  // Attach servos
  steerServo.attach(STEER_PIN, 1000, 2000);
  throttleServo.attach(THROTTLE_PIN, 1000, 2000);

  // Stop motors initially
  steerServo.writeMicroseconds(STEER_CENTER_US);
  throttleServo.writeMicroseconds(THROTTLE_STOP_US);

  delay(1000); // small pause

  // 7-second calibration
  calibrateQTR();

  // Short pause before starting line following
  throttleServo.writeMicroseconds(THROTTLE_STOP_US);
  steerServo.writeMicroseconds(STEER_CENTER_US);
  delay(500);
}

/* ===========================================================
   LOOP
   =========================================================== */
void loop() {
  // Read sensors
  readQTR(rawVals);

  // Normalize to 0..1000 (white line = high)
  normalizeSensors();

  // Update steering + throttle
  updateControl();

  // Optional small delay for stability
  delay(5);
}
