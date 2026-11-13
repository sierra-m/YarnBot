// YarnBot — ESP32-C6 RC tank mixer for two IBT-4 motor drivers,
// with latching 3S LiPo low-voltage cutoff.
//
// Requires Arduino-ESP32 core 3.x (FQBN esp32:esp32:esp32c6).
//
// Hardware note: put a 100 nF capacitor from BAT_PIN to GND. The 330k/100k
// divider has ~77 kOhm source impedance; the cap keeps ADC readings stable.

#include <Arduino.h>
#include <driver/mcpwm_cap.h>

// ---------------------------------------------------------------- Config ----

#define DEBUG 1  // set to 0 to disable Serial debug output
#define RC_STATS 0     // 1 = print per-channel pulse min/max/avg once per second
#define RC_LOOPBACK 0  // 1 = bench test: output a fixed pulse on PIN_LOOPBACK, motors forced off

// Pins
constexpr uint8_t PIN_RC_THROTTLE = 18;
constexpr uint8_t PIN_RC_STEER    = 20;
constexpr uint8_t PIN_L_IN1       = 19;
constexpr uint8_t PIN_L_IN2       = 21;
constexpr uint8_t PIN_R_IN1       = 22;
constexpr uint8_t PIN_R_IN2       = 23;
constexpr uint8_t PIN_BAT         = 2;   // ADC1_CH2
constexpr uint8_t PIN_LOOPBACK    = 1;   // RC_LOOPBACK test output; jumper to an RC input pin

// RC calibration (microseconds)
constexpr uint16_t RC_MIN        = 1000;
constexpr uint16_t RC_MID        = 1500;
constexpr uint16_t RC_MAX        = 2000;
constexpr uint16_t RC_DEADBAND   = 20;
constexpr uint16_t RC_VALID_MIN  = 800;   // pulses outside this window are ignored
constexpr uint16_t RC_VALID_MAX  = 2200;
constexpr uint32_t RC_TIMEOUT_MS = 100;   // failsafe if no pulse for this long
constexpr bool     REVERSE_THROTTLE = false;
constexpr bool     REVERSE_STEER    = false;
constexpr uint32_t RC_CAP_RES_HZ    = 10000000;  // MCPWM capture timer tick rate (0.1 us)

// RC_LOOPBACK test pulse
constexpr uint16_t LOOPBACK_US       = 1500;
constexpr uint32_t LOOPBACK_HZ       = 50;
constexpr uint8_t  LOOPBACK_RES_BITS = 16;  // ~0.3 us per step at 50 Hz

// Motor output
constexpr uint32_t PWM_FREQ_HZ   = 20000;
constexpr uint8_t  PWM_RES_BITS  = 10;
constexpr uint32_t PWM_MAX       = (1u << PWM_RES_BITS) - 1;
constexpr bool     INVERT_LEFT   = true;
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

// Pulse widths are measured with the MCPWM capture peripheral: the hardware
// latches a timer value at each edge, so interrupt latency doesn't affect the
// result (a micros()-in-ISR approach jittered by up to ~130 us).
struct RcChannel {
  uint8_t pin;
  volatile uint32_t riseTicks;
  volatile bool     riseSeen;    // a rising edge is waiting for its falling edge
  volatile uint16_t widthUs;
  volatile uint32_t lastUpdateMs;
  // RC_STATS accumulators, reset each time they are read
  volatile uint16_t statMin;
  volatile uint16_t statMax;
  volatile uint32_t statSum;
  volatile uint32_t statCount;
};

static RcChannel rcThrottle = {PIN_RC_THROTTLE, 0, false, 0, 0, 0, 0, 0, 0};
static RcChannel rcSteer    = {PIN_RC_STEER,    0, false, 0, 0, 0, 0, 0, 0};
static portMUX_TYPE rcMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t rcTicksPerUs = RC_CAP_RES_HZ / 1000000;  // updated from the driver in rcBegin()

static bool IRAM_ATTR rcOnCapture(mcpwm_cap_channel_handle_t, const mcpwm_capture_event_data_t *ev,
                                  void *arg) {
  RcChannel *ch = static_cast<RcChannel *>(arg);
  if (ev->cap_edge == MCPWM_CAP_EDGE_POS) {
    ch->riseTicks = ev->cap_value;
    ch->riseSeen = true;
    return false;
  }
  if (!ch->riseSeen) return false;  // falling edge without a matching rise: ignore
  ch->riseSeen = false;

  const uint32_t ticks = ev->cap_value - ch->riseTicks;  // wraps correctly
  const uint32_t w = (ticks + rcTicksPerUs / 2) / rcTicksPerUs;
  if (w >= RC_VALID_MIN && w <= RC_VALID_MAX) {
    portENTER_CRITICAL_ISR(&rcMux);
    ch->widthUs = static_cast<uint16_t>(w);
    ch->lastUpdateMs = millis();
#if RC_STATS
    if (ch->statCount == 0 || w < ch->statMin) ch->statMin = static_cast<uint16_t>(w);
    if (w > ch->statMax) ch->statMax = static_cast<uint16_t>(w);
    ch->statSum = ch->statSum + w;
    ch->statCount = ch->statCount + 1;
#endif
    portEXIT_CRITICAL_ISR(&rcMux);
  }
  return false;
}

// Set up one capture timer with a channel per RC input. Returns false on any
// driver error; the inputs then never update, so the failsafe keeps motors off.
static bool rcBegin() {
  mcpwm_cap_timer_handle_t timer = nullptr;
  mcpwm_capture_timer_config_t tcfg = {};
  tcfg.group_id = 0;
  tcfg.clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT;
  tcfg.resolution_hz = RC_CAP_RES_HZ;
  if (mcpwm_new_capture_timer(&tcfg, &timer) != ESP_OK) return false;

  uint32_t resHz = 0;
  if (mcpwm_capture_timer_get_resolution(timer, &resHz) != ESP_OK || resHz < 1000000) return false;
  rcTicksPerUs = resHz / 1000000;

  for (RcChannel *ch : {&rcThrottle, &rcSteer}) {
    mcpwm_cap_channel_handle_t chan = nullptr;
    mcpwm_capture_channel_config_t ccfg = {};
    ccfg.gpio_num = ch->pin;
    ccfg.prescale = 1;
    ccfg.flags.pos_edge = 1;
    ccfg.flags.neg_edge = 1;
    if (mcpwm_new_capture_channel(timer, &ccfg, &chan) != ESP_OK) return false;
    mcpwm_capture_event_callbacks_t cbs = {};
    cbs.on_cap = rcOnCapture;
    if (mcpwm_capture_channel_register_event_callbacks(chan, &cbs, ch) != ESP_OK) return false;
    if (mcpwm_capture_channel_enable(chan) != ESP_OK) return false;
  }

  return mcpwm_capture_timer_enable(timer) == ESP_OK &&
         mcpwm_capture_timer_start(timer) == ESP_OK;
}

static void rcSnapshot(const RcChannel &ch, uint16_t &width, uint32_t &lastMs) {
  portENTER_CRITICAL(&rcMux);
  width = ch.widthUs;
  lastMs = ch.lastUpdateMs;
  portEXIT_CRITICAL(&rcMux);
}

#if RC_STATS
// Print and reset the stats for one channel, e.g. "thr n=50 min=1498 max=1503 avg=1500.4 spread=5".
static void rcPrintStats(const char *name, RcChannel &ch) {
  portENTER_CRITICAL(&rcMux);
  const uint16_t mn = ch.statMin, mx = ch.statMax;
  const uint32_t sum = ch.statSum, n = ch.statCount;
  ch.statMin = 0;
  ch.statMax = 0;
  ch.statSum = 0;
  ch.statCount = 0;
  portEXIT_CRITICAL(&rcMux);
  if (n == 0) {
    Serial.printf("%s n=0 (no valid pulses)", name);
  } else {
    Serial.printf("%s n=%lu min=%u max=%u avg=%.1f spread=%u", name,
                  static_cast<unsigned long>(n), mn, mx,
                  static_cast<double>(sum) / n, mx - mn);
  }
}
#endif

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

// ledcWrite only when the duty changes; loop() runs far faster than the PWM
// period, and rewriting the same value every pass is wasted driver work.
static uint32_t pwmDuty[SOC_GPIO_PIN_COUNT];  // all 0, matching setup()

static void pwmWrite(uint8_t pin, uint32_t duty) {
  if (pwmDuty[pin] == duty) return;
  pwmDuty[pin] = duty;
  ledcWrite(pin, duty);
}

static void setMotor(uint8_t in1, uint8_t in2, float value, bool invert) {
  if (invert) value = -value;
  const uint32_t duty = static_cast<uint32_t>(lroundf(fabsf(value) * PWM_MAX));
  if (value > 0.0f) {
    pwmWrite(in2, 0);
    pwmWrite(in1, duty);
  } else if (value < 0.0f) {
    pwmWrite(in1, 0);
    pwmWrite(in2, duty);
  } else {
    pwmWrite(in1, 0);  // coast
    pwmWrite(in2, 0);
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

  if (!rcBegin()) {
    Serial.println("RC capture setup FAILED - no RC input, motors will stay off");
  }

#if RC_LOOPBACK
  // Fixed test pulse. Jumper PIN_LOOPBACK to the throttle and/or steering pin
  // (receiver unplugged from those pins). Motors stay off in this mode.
  if (ledcAttach(PIN_LOOPBACK, LOOPBACK_HZ, LOOPBACK_RES_BITS)) {
    const uint32_t duty = static_cast<uint32_t>(
        lround(static_cast<double>(LOOPBACK_US) * LOOPBACK_HZ * (1u << LOOPBACK_RES_BITS) / 1e6));
    ledcWrite(PIN_LOOPBACK, duty);
    Serial.printf("LOOPBACK TEST: %u us @ %lu Hz on GPIO%u, motors disabled\n", LOOPBACK_US,
                  static_cast<unsigned long>(LOOPBACK_HZ), PIN_LOOPBACK);
  } else {
    Serial.println("LOOPBACK TEST: ledcAttach failed");
  }
#endif

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
  if (RC_LOOPBACK || cutoffLatched || !signalOk || !armed) {
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

#if RC_STATS
  static uint32_t lastStatsMs = 0;
  if (nowMs - lastStatsMs >= 1000) {
    lastStatsMs = nowMs;
    Serial.print("STATS ");
    rcPrintStats("thr", rcThrottle);
    Serial.print(" | ");
    rcPrintStats("str", rcSteer);
    Serial.println();
  }
#endif
}
