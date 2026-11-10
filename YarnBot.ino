// YarnBot — ESP32-C6 RC tank mixer for two IBT-4 motor drivers,
// with latching 3S LiPo low-voltage cutoff.
//
// Requires Arduino-ESP32 core 3.x (FQBN esp32:esp32:esp32c6).
//
// Hardware note: put a 100 nF capacitor from BAT_PIN to GND. The 330k/100k
// divider has ~77 kOhm source impedance; the cap keeps ADC readings stable.

#include <Arduino.h>

// ---------------------------------------------------------------- Config ----

#define DEBUG 1  // set to 0 to disable Serial debug output

// Pins
constexpr uint8_t PIN_RC_THROTTLE = 18;
constexpr uint8_t PIN_RC_STEER    = 19;
constexpr uint8_t PIN_L_IN1       = 20;
constexpr uint8_t PIN_L_IN2       = 21;
constexpr uint8_t PIN_R_IN1       = 22;
constexpr uint8_t PIN_R_IN2       = 23;
constexpr uint8_t PIN_BAT         = 2;   // ADC1_CH2

// RC calibration (microseconds)
constexpr uint16_t RC_MIN        = 1000;
constexpr uint16_t RC_MID        = 1500;
constexpr uint16_t RC_MAX        = 2000;
constexpr uint16_t RC_DEADBAND   = 40;
constexpr uint16_t RC_VALID_MIN  = 800;   // pulses outside this window are ignored
constexpr uint16_t RC_VALID_MAX  = 2200;
constexpr uint32_t RC_TIMEOUT_MS = 100;   // failsafe if no pulse for this long
constexpr bool     REVERSE_THROTTLE = false;
constexpr bool     REVERSE_STEER    = false;

// Motor output
constexpr uint32_t PWM_FREQ_HZ   = 20000;
constexpr uint8_t  PWM_RES_BITS  = 10;
constexpr uint32_t PWM_MAX       = (1u << PWM_RES_BITS) - 1;
constexpr bool     INVERT_LEFT   = false;
constexpr bool     INVERT_RIGHT  = false;
constexpr float    SLEW_PER_SEC  = 4.0f;  // full-scale units/sec; 0 disables slew limiting

// Battery monitor
constexpr float    DIV_R1        = 330.0f;  // kOhm, top
constexpr float    DIV_R2        = 100.0f;  // kOhm, bottom
constexpr float    BAT_CAL       = 1.00f;   // trim against a multimeter
constexpr float    CUTOFF_V      = 9.9f;
constexpr uint32_t CUTOFF_HOLD_MS   = 1000;  // must stay below CUTOFF_V this long
constexpr uint32_t BAT_SAMPLE_MS    = 20;
constexpr float    BAT_EMA_ALPHA    = 0.05f;

// ------------------------------------------------------------- RC input ----

struct RcChannel {
  uint8_t pin;
  volatile uint32_t riseUs;
  volatile uint16_t widthUs;
  volatile uint32_t lastUpdateMs;
};

static RcChannel rcThrottle = {PIN_RC_THROTTLE, 0, 0, 0};
static RcChannel rcSteer    = {PIN_RC_STEER,    0, 0, 0};
static portMUX_TYPE rcMux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR rcIsr(void *arg) {
  RcChannel *ch = static_cast<RcChannel *>(arg);
  const uint32_t now = micros();
  if (digitalRead(ch->pin)) {
    ch->riseUs = now;
  } else {
    const uint32_t w = now - ch->riseUs;
    if (w >= RC_VALID_MIN && w <= RC_VALID_MAX) {
      portENTER_CRITICAL_ISR(&rcMux);
      ch->widthUs = static_cast<uint16_t>(w);
      ch->lastUpdateMs = millis();
      portEXIT_CRITICAL_ISR(&rcMux);
    }
  }
}

static void rcSnapshot(const RcChannel &ch, uint16_t &width, uint32_t &lastMs) {
  portENTER_CRITICAL(&rcMux);
  width = ch.widthUs;
  lastMs = ch.lastUpdateMs;
  portEXIT_CRITICAL(&rcMux);
}

// Map a pulse width to -1..+1 with a center deadband.
static float rcNormalize(uint16_t us, bool reverse) {
  float v;
  const int d = static_cast<int>(us) - RC_MID;
  if (abs(d) <= RC_DEADBAND) {
    v = 0.0f;
  } else if (d > 0) {
    v = static_cast<float>(d - RC_DEADBAND) / (RC_MAX - RC_MID - RC_DEADBAND);
  } else {
    v = static_cast<float>(d + RC_DEADBAND) / (RC_MID - RC_MIN - RC_DEADBAND);
  }
  v = constrain(v, -1.0f, 1.0f);
  return reverse ? -v : v;
}

// --------------------------------------------------------- Motor output ----

static void setMotor(uint8_t in1, uint8_t in2, float value, bool invert) {
  if (invert) value = -value;
  const uint32_t duty = static_cast<uint32_t>(lroundf(fabsf(value) * PWM_MAX));
  if (value > 0.0f) {
    ledcWrite(in2, 0);
    ledcWrite(in1, duty);
  } else if (value < 0.0f) {
    ledcWrite(in1, 0);
    ledcWrite(in2, duty);
  } else {
    ledcWrite(in1, 0);  // coast
    ledcWrite(in2, 0);
  }
}

static float slew(float current, float target, float maxStep) {
  if (maxStep <= 0.0f) return target;
  if (target > current + maxStep) return current + maxStep;
  if (target < current - maxStep) return current - maxStep;
  return target;
}

// ------------------------------------------------------ Battery monitor ----

static float    vbatFiltered  = 0.0f;
static bool     cutoffLatched = false;
static uint32_t belowSinceMs  = 0;
static uint32_t lastBatSampleMs = 0;

static float readVbat() {
  const uint32_t mv = analogReadMilliVolts(PIN_BAT);
  return mv / 1000.0f * (DIV_R1 + DIV_R2) / DIV_R2 * BAT_CAL;
}

static void updateBattery(uint32_t nowMs) {
  if (nowMs - lastBatSampleMs < BAT_SAMPLE_MS) return;
  lastBatSampleMs = nowMs;

  vbatFiltered += BAT_EMA_ALPHA * (readVbat() - vbatFiltered);

  if (cutoffLatched) return;
  if (vbatFiltered < CUTOFF_V) {
    if (belowSinceMs == 0) belowSinceMs = nowMs;
    if (nowMs - belowSinceMs >= CUTOFF_HOLD_MS) {
      cutoffLatched = true;
      Serial.printf("LOW BATTERY CUTOFF: %.2f V — motors disabled until reset\n", vbatFiltered);
    }
  } else {
    belowSinceMs = 0;
  }
}

static void updateStatusLed(uint32_t nowMs) {
#ifdef RGB_BUILTIN
  static bool lastOn = false;
  const bool on = cutoffLatched && ((nowMs / 250) % 2 == 0);
  if (on != lastOn) {
    lastOn = on;
    rgbLedWrite(RGB_BUILTIN, on ? 40 : 0, 0, 0);
  }
#else
  (void)nowMs;
#endif
}

// ---------------------------------------------------------------- Main ----

static float    outLeft = 0.0f, outRight = 0.0f;
static bool     armed = false;
static uint32_t lastLoopMs = 0;

void setup() {
  Serial.begin(115200);

  // Motors off before anything else.
  for (uint8_t pin : {PIN_L_IN1, PIN_L_IN2, PIN_R_IN1, PIN_R_IN2}) {
    ledcAttach(pin, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcWrite(pin, 0);
  }

  pinMode(PIN_RC_THROTTLE, INPUT);
  pinMode(PIN_RC_STEER, INPUT);
  attachInterruptArg(digitalPinToInterrupt(PIN_RC_THROTTLE), rcIsr, &rcThrottle, CHANGE);
  attachInterruptArg(digitalPinToInterrupt(PIN_RC_STEER), rcIsr, &rcSteer, CHANGE);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_BAT, ADC_11db);
  // Seed the filter so it doesn't start at 0 V and trip the cutoff.
  float sum = 0.0f;
  for (int i = 0; i < 16; i++) sum += readVbat();
  vbatFiltered = sum / 16.0f;

  lastLoopMs = millis();
  Serial.printf("YarnBot ready. Vbat = %.2f V\n", vbatFiltered);
}

void loop() {
  const uint32_t nowMs = millis();
  const float dt = (nowMs - lastLoopMs) / 1000.0f;
  lastLoopMs = nowMs;

  // 1. RC input + failsafe
  uint16_t thrUs, strUs;
  uint32_t thrLast, strLast;
  rcSnapshot(rcThrottle, thrUs, thrLast);
  rcSnapshot(rcSteer, strUs, strLast);

  const bool signalOk = thrUs != 0 && strUs != 0 &&
                        (nowMs - thrLast) < RC_TIMEOUT_MS &&
                        (nowMs - strLast) < RC_TIMEOUT_MS;

  float throttle = 0.0f, steer = 0.0f;
  if (signalOk) {
    throttle = rcNormalize(thrUs, REVERSE_THROTTLE);
    steer    = rcNormalize(strUs, REVERSE_STEER);
    // Arm only once both sticks have been seen at neutral.
    if (!armed && throttle == 0.0f && steer == 0.0f) {
      armed = true;
#if DEBUG
      Serial.println("Armed");
#endif
    }
  } else if (armed) {
    armed = false;  // must re-center sticks after signal loss
#if DEBUG
    Serial.println("Signal lost — disarmed");
#endif
  }

  // 2. Battery
  updateBattery(nowMs);
  updateStatusLed(nowMs);

  // 3. Mix (tank style)
  float left  = constrain(throttle + steer, -1.0f, 1.0f);
  float right = constrain(throttle - steer, -1.0f, 1.0f);

  // 4. Safety gates
  if (cutoffLatched || !signalOk || !armed) {
    left = right = 0.0f;
  }

  // 5. Slew + output (immediate stop on safety gates)
  if (left == 0.0f && right == 0.0f && (cutoffLatched || !signalOk)) {
    outLeft = outRight = 0.0f;
  } else {
    const float step = SLEW_PER_SEC * dt;
    outLeft  = slew(outLeft, left, step);
    outRight = slew(outRight, right, step);
  }
  setMotor(PIN_L_IN1, PIN_L_IN2, outLeft, INVERT_LEFT);
  setMotor(PIN_R_IN1, PIN_R_IN2, outRight, INVERT_RIGHT);

  // 6. Debug
#if DEBUG
  static uint32_t lastPrintMs = 0;
  if (nowMs - lastPrintMs >= 200) {
    lastPrintMs = nowMs;
    Serial.printf("thr=%4u str=%4u | T=%+.2f S=%+.2f | L=%+.2f R=%+.2f | %.2fV%s%s%s\n",
                  thrUs, strUs, throttle, steer, outLeft, outRight, vbatFiltered,
                  signalOk ? "" : " NOSIG", armed ? "" : " DISARMED",
                  cutoffLatched ? " CUTOFF" : "");
  }
#endif
}
