/*
 * ============================================================================
 *  FORECAST - Landslide / Slope-Instability Early Warning Node
 *  Target : ESP32 Dev Module (Arduino framework, Arduino IDE)
 * ============================================================================
 *
 *  WHAT THIS NODE DOES
 *    - Reads a capacitive soil-moisture sensor (ADC1) -> 0..100 %
 *    - Reads an MPU6050 and computes slope tilt (deg) from the ACCELEROMETER
 *      gravity vector only (no gyro integration -> no drift), relative to a
 *      stored "level" baseline.
 *    - Keeps a rolling 2-second average of both values.
 *    - EDGE REFLEX (works with no Wi-Fi / no server):
 *          moisture_avg > 80 %  AND  tilt_avg > 3.0 deg
 *      -> servo gate goes to CLOSED angle + buzzer sounds, immediately.
 *    - POSTs JSON telemetry to a Python backend. Wi-Fi + HTTP run in a
 *      separate FreeRTOS task on core 0, so a slow/dead network can never
 *      delay sampling, the reflex, the servo or the buzzer (core 1 / loop()).
 *
 *  ARDUINO IDE SETUP
 *    Board   : Tools > Board > esp32 > "ESP32 Dev Module"
 *    Core    : Espressif "esp32" board package (2.0.x or 3.x both work)
 *    Libraries (Sketch > Include Library > Manage Libraries):
 *        1. "Adafruit MPU6050"      (by Adafruit)  -> accept the dependencies
 *        2. "Adafruit Unified Sensor"                 (dependency)
 *        3. "Adafruit BusIO"                          (dependency)
 *        4. "ESP32Servo"            (by Kevin Harrington / John K. Bennett)
 *      Not needed (built into the ESP32 core): WiFi, HTTPClient, Wire,
 *      Preferences. JSON is formatted manually, so ArduinoJson is NOT needed.
 *
 *  WIRING
 *      Capacitive moisture  AOUT -> GPIO34 (ADC1_CH6, input-only)
 *                           VCC  -> 3V3 ,  GND -> GND
 *      MPU6050 (GY-521)     SDA  -> GPIO21 ,  SCL -> GPIO22
 *                           VCC  -> 3V3 ,  GND -> GND ,  AD0 -> GND (addr 0x68)
 *      Servo (gate)         signal -> GPIO18
 *                           power  -> separate 5 V supply (NOT the ESP32 3V3
 *                           pin); that supply's GND MUST be joined to ESP32
 *                           GND. Add a 470-1000 uF capacitor across servo
 *                           power to prevent brown-out resets when it moves.
 *      Buzzer (ACTIVE)      + -> GPIO25 (via transistor if it draws >20 mA),
 *                           - -> GND
 *      Button (optional)    Uses the on-board BOOT button (GPIO0, to GND).
 *                           Short press = manual alarm reset
 *                           Hold >= 3 s = re-zero tilt baseline (only when
 *                           no alarm is active; hold the pipe at its true
 *                           "level / at rest" position while doing it)
 *
 *  ONLY USE ADC1 PINS (GPIO32-39) for the moisture sensor: ADC2 does not work
 *  while Wi-Fi is on.
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>

// ============================================================================
//  1. USER CONFIGURATION  (edit everything you need in this block)
// ============================================================================

// ---- Node identity (the backend uses this for 2-of-3 Top/Middle/Bottom voting)
#define NODE_ID               "node_top"        // node_top / node_mid / node_bottom

// ---- Wi-Fi + backend ------------------------------------------------------
#define WIFI_SSID             "YOUR_WIFI_SSID"
#define WIFI_PASSWORD         "YOUR_WIFI_PASSWORD"
#define BACKEND_URL           "http://192.168.1.100:5000/api/telemetry"
#define TELEMETRY_PERIOD_MS   5000UL   // normal POST interval
#define WIFI_RETRY_MS         10000UL  // wait between reconnect attempts
#define HTTP_TIMEOUT_MS       3000UL   // max time a POST may take (net task only)

// ---- Pins -----------------------------------------------------------------
#define PIN_MOISTURE_ADC      34       // ADC1 only!
#define PIN_I2C_SDA           21
#define PIN_I2C_SCL           22
#define PIN_SERVO             18
#define PIN_BUZZER            25
#define PIN_BUTTON            0        // BOOT button, active LOW
#define BUZZER_ACTIVE_HIGH    true     // false if your buzzer module is active-LOW

// ---- Moisture calibration (RAW 12-bit ADC counts, 0..4095) ------------------
// Capacitive sensors read HIGH in air/dry and LOW in water. Measure yours:
//   DRY_VALUE : sensor in open air / bone-dry soil
//   WET_VALUE : sensor in water / fully saturated soil
// (If your sensor behaves the opposite way, just swap the two numbers.)
#define DRY_VALUE             3200
#define WET_VALUE             1400

// ---- Alarm thresholds (from the FORECAST design) --------------------------
#define MOISTURE_ALARM_PCT    80.0f    // trigger when avg moisture  >  this ...
#define TILT_ALARM_DEG        3.0f     // ... AND avg tilt           >  this
// Hysteresis: alarm is only considered "over" once the combined condition
// moisture > CLEAR_PCT AND tilt > CLEAR_DEG is FALSE for CLEAR_HOLD_MS.
#define MOISTURE_CLEAR_PCT    75.0f
#define TILT_CLEAR_DEG        2.5f
#define CLEAR_HOLD_MS         30000UL  // must stay clear this long to reset

// true  = gate re-opens automatically after the clear condition holds
// false = alarm LATCHES until someone presses the button (inspect first)
#define AUTO_RESET_ENABLED    true

// ---- Servo gate angles (degrees, 0..180) - adjust for your mechanism -----------
#define SERVO_OPEN_ANGLE      90       // gate raised  (normal, traffic flows)
#define SERVO_CLOSED_ANGLE    0        // gate lowered (barrier down)
#define SERVO_MIN_US          500
#define SERVO_MAX_US          2400

// ---- Buzzer pattern while alarming -----------------------------------------
#define BUZZER_PERIOD_MS      1000UL   // one on/off cycle
#define BUZZER_ON_MS          600UL    // on-time within each cycle

// ---- Sampling / averaging -------------------------------------------------
#define SAMPLE_INTERVAL_MS    20UL     // 50 Hz sampling
#define AVG_WINDOW_MS         2000UL   // 2-second rolling window
#define WINDOW_SAMPLES        (AVG_WINDOW_MS / SAMPLE_INTERVAL_MS)   // = 100
#define ADC_OVERSAMPLE        8        // ADC reads averaged per sample

// ---- Tilt baseline ----------------------------------------------------------
#define BASELINE_SAMPLES      100      // ~2 s of samples averaged for "level"
#define USE_SAVED_BASELINE    true     // keep baseline in flash across reboots
// WHY SAVE IT: if the node browns-out/reboots while the slope has already
// crept, re-zeroing at boot would silently erase the tilt we are watching.

#define SERIAL_LOG_MS         1000UL   // debug print interval

// ============================================================================
//  2. GLOBAL STATE
// ============================================================================

Adafruit_MPU6050 mpu;
Servo            gateServo;
Preferences      prefs;

// ---- Rolling window buffers (index shared by both sensors) ------------------
static float    moistBuf[WINDOW_SAMPLES];
static float    tiltBuf[WINDOW_SAMPLES];
static uint16_t winIdx   = 0;
static uint16_t winCount = 0;                 // how many valid samples so far
static float    moistAvg = 0.0f;              // 2 s average moisture (%)
static float    tiltAvg  = 0.0f;              // 2 s average tilt (deg)
static int      lastRaw  = 0;                 // last raw ADC (for calibration)
static float    lastTilt = 0.0f;

// ---- MPU / baseline -------------------------------------------------------
static bool     mpuOk          = false;
static uint8_t  mpuFailCount   = 0;
static uint32_t lastMpuRetryMs = 0;
static bool     baselineValid  = false;       // a "level" vector is available
static bool     baselineActive = false;       // currently capturing a new one
static uint16_t baseCount      = 0;
static double   baseSumX = 0, baseSumY = 0, baseSumZ = 0;
static float    baseX = 0, baseY = 0, baseZ = 1;   // unit vector of "level"

// ---- Alarm state ----------------------------------------------------------
static bool     alarmActive     = false;
static uint32_t alarmCount      = 0;          // total alarms since boot
static uint32_t clearSinceMs    = 0;          // when clear condition began (0=not)
static int      servoAngleNow   = -1;

// ---- Telemetry shared between core 1 (sensing) and core 0 (network) --------
struct Telemetry {
  float    moisture;
  float    tilt;
  int      raw;
  bool     alarm;
  bool     mpuOk;
  uint32_t alarmCount;
};
static Telemetry sharedTelem = {0, 0, 0, false, false, 0};
static portMUX_TYPE telemMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool     urgentSend    = false;   // send NOW (alarm changed)
static volatile bool     wifiUp        = false;
static volatile uint32_t wifiReconnects = 0;

// ============================================================================
//  3. SENSOR HELPERS
// ============================================================================

/** Oversampled raw ADC read of the moisture sensor. */
int readMoistureRaw() {
  uint32_t sum = 0;
  for (int i = 0; i < ADC_OVERSAMPLE; i++) sum += analogRead(PIN_MOISTURE_ADC);
  return (int)(sum / ADC_OVERSAMPLE);
}

/** Convert raw ADC counts to 0..100 % using DRY/WET calibration. */
float moistureToPercent(int raw) {
  float pct = (float)(DRY_VALUE - raw) * 100.0f / (float)(DRY_VALUE - WET_VALUE);
  if (pct < 0.0f)   pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return pct;
}

/** (Re)initialise the MPU6050. Returns true on success. */
bool initMpu() {
  if (!mpu.begin(0x68, &Wire)) return false;
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);   // on-chip low-pass: kills vibration
  return true;
}

/** Begin a non-blocking capture of a new "level" baseline. */
void startBaselineCapture() {
  baselineActive = true;
  baselineValid  = false;       // reflex is paused until the new zero exists
  baseCount = 0;
  baseSumX = baseSumY = baseSumZ = 0;
  winCount = 0;                 // refill the window with post-zero data
  winIdx   = 0;
  Serial.println(F("[BASELINE] capturing - keep node still at its level position"));
}

/** Feed one accelerometer sample (m/s^2) into the baseline capture. */
void baselineAccumulate(float ax, float ay, float az) {
  baseSumX += ax; baseSumY += ay; baseSumZ += az;
  if (++baseCount < BASELINE_SAMPLES) return;

  float x = baseSumX / baseCount, y = baseSumY / baseCount, z = baseSumZ / baseCount;
  float mag = sqrtf(x * x + y * y + z * z);
  if (mag < 1.0f) {             // nonsense (free-fall / bad read): try again
    startBaselineCapture();
    return;
  }
  baseX = x / mag; baseY = y / mag; baseZ = z / mag;
  baselineValid = true;
  baselineActive = false;

  prefs.begin("forecast", false);
  prefs.putFloat("bx", baseX); prefs.putFloat("by", baseY); prefs.putFloat("bz", baseZ);
  prefs.putBool("bok", true);
  prefs.end();
  Serial.printf("[BASELINE] saved: (%.4f, %.4f, %.4f)\n", baseX, baseY, baseZ);
}

/**
 * Tilt = angle between the current gravity vector and the baseline gravity
 * vector. Independent of how the PCB is mounted, uses accelerometer only.
 */
float computeTiltDeg(float ax, float ay, float az) {
  float mag = sqrtf(ax * ax + ay * ay + az * az);
  if (mag < 1e-3f) return lastTilt;
  float dot = (ax * baseX + ay * baseY + az * baseZ) / mag;
  if (dot > 1.0f)  dot = 1.0f;
  if (dot < -1.0f) dot = -1.0f;
  return acosf(dot) * 180.0f / PI;
}

// ============================================================================
//  4. SAMPLING + 2-SECOND ROLLING AVERAGE   (runs every SAMPLE_INTERVAL_MS)
// ============================================================================

void sampleSensors() {
  // ---- moisture ----
  lastRaw = readMoistureRaw();
  float moist = moistureToPercent(lastRaw);

  // ---- tilt ----
  if (mpuOk) {
    sensors_event_t a, g, t;
    if (mpu.getEvent(&a, &g, &t)) {
      mpuFailCount = 0;
      float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
      if (baselineActive)      baselineAccumulate(ax, ay, az);
      else if (baselineValid)  lastTilt = computeTiltDeg(ax, ay, az);
    } else if (++mpuFailCount >= 25) {   // ~0.5 s of consecutive failures
      mpuOk = false;
      Serial.println(F("[MPU] read failures - marking sensor offline"));
    }
  } else if (millis() - lastMpuRetryMs >= 2000UL) {   // try to bring it back
    lastMpuRetryMs = millis();
    if (initMpu()) { mpuOk = true; mpuFailCount = 0; Serial.println(F("[MPU] recovered")); }
  }

  // ---- push into ring buffers (reuse last tilt if MPU hiccuped) ----
  moistBuf[winIdx] = moist;
  tiltBuf[winIdx]  = lastTilt;
  winIdx = (winIdx + 1) % WINDOW_SAMPLES;
  if (winCount < WINDOW_SAMPLES) winCount++;

  // ---- averages (recomputed each tick: 100 adds, no accumulated drift) ----
  float sm = 0, st = 0;
  for (uint16_t i = 0; i < winCount; i++) { sm += moistBuf[i]; st += tiltBuf[i]; }
  moistAvg = sm / winCount;
  tiltAvg  = st / winCount;
}

// ============================================================================
//  5. EDGE REFLEX (alarm state machine) - purely local, never touches Wi-Fi
// ============================================================================

/** True only when we have a full 2 s window and a trustworthy tilt reading. */
bool dataTrustworthy() {
  return (winCount >= WINDOW_SAMPLES) && mpuOk && baselineValid && !baselineActive;
}

void evaluateReflex() {
  if (!dataTrustworthy()) return;     // warm-up / sensor fault: hold current state
  uint32_t now = millis();

  bool triggerCond = (moistAvg > MOISTURE_ALARM_PCT) && (tiltAvg > TILT_ALARM_DEG);
  bool holdCond    = (moistAvg > MOISTURE_CLEAR_PCT) && (tiltAvg > TILT_CLEAR_DEG);

  if (!alarmActive) {
    if (triggerCond) {
      alarmActive  = true;
      clearSinceMs = 0;
      alarmCount++;
      urgentSend   = true;            // tell the backend ASAP (if reachable)
      Serial.println(F("!!! ALARM: gate CLOSED, siren ON !!!"));
    }
  } else {
    // Alarm is active: track how long the (hysteresis-widened) danger is gone.
    if (holdCond) {
      clearSinceMs = 0;               // still dangerous -> restart clear timer
    } else if (clearSinceMs == 0) {
      clearSinceMs = now ? now : 1;   // danger just ended -> start timer
    }
    if (AUTO_RESET_ENABLED && clearSinceMs != 0 && (now - clearSinceMs >= CLEAR_HOLD_MS)) {
      alarmActive = false;
      clearSinceMs = 0;
      urgentSend  = true;
      Serial.println(F("[ALARM] auto-reset: conditions clear, gate re-opened"));
    }
  }
}

/** Manual reset (button). Only honoured when the danger condition is not present. */
void manualReset() {
  if (!alarmActive) return;
  bool holdCond = (moistAvg > MOISTURE_CLEAR_PCT) && (tiltAvg > TILT_CLEAR_DEG);
  if (holdCond) {
    Serial.println(F("[BUTTON] reset refused: conditions still dangerous"));
    return;
  }
  alarmActive = false;
  clearSinceMs = 0;
  urgentSend = true;
  Serial.println(F("[BUTTON] manual reset: gate re-opened"));
}

/** Drive servo + buzzer from alarmActive. Non-blocking (millis pattern). */
void updateOutputs() {
  // Servo: write only on change so we don't fight the PWM every loop.
  int target = alarmActive ? SERVO_CLOSED_ANGLE : SERVO_OPEN_ANGLE;
  if (target != servoAngleNow) {
    gateServo.write(target);
    servoAngleNow = target;
  }

  // Buzzer: repeating beep while alarming, silent otherwise.
  bool on = alarmActive && ((millis() % BUZZER_PERIOD_MS) < BUZZER_ON_MS);
  digitalWrite(PIN_BUZZER, (on == BUZZER_ACTIVE_HIGH) ? HIGH : LOW);
}

// ============================================================================
//  6. BUTTON  (short = manual reset, long hold = re-zero tilt)
// ============================================================================

void handleButton() {
  static bool     wasDown = false;
  static uint32_t downAt  = 0;
  bool down = (digitalRead(PIN_BUTTON) == LOW);
  uint32_t now = millis();

  if (down && !wasDown) downAt = now;
  if (!down && wasDown) {
    uint32_t held = now - downAt;
    if (held >= 3000UL) {
      if (!alarmActive) startBaselineCapture();
      else Serial.println(F("[BUTTON] re-zero refused while alarm active"));
    } else if (held >= 50UL) {
      manualReset();
    }
  }
  wasDown = down;
}

// ============================================================================
//  7. NETWORK TASK  (core 0)  - Wi-Fi keep-alive + HTTP POST telemetry
//     Anything slow or blocking in here is harmless to the safety reflex,
//     which lives entirely in loop() on core 1.
// ============================================================================

/** Build JSON and POST it. Returns HTTP status code (<0 = transport error). */
int postTelemetry(const Telemetry &t) {
  char body[320];
  snprintf(body, sizeof(body),
    "{\"node_id\":\"%s\",\"moisture_pct\":%.1f,\"tilt_deg\":%.2f,"
    "\"moisture_raw\":%d,\"alarm_active\":%s,\"alarm_count\":%lu,"
    "\"mpu_ok\":%s,\"wifi_connected\":%s,\"wifi_rssi\":%d,"
    "\"wifi_reconnects\":%lu,\"uptime_ms\":%lu}",
    NODE_ID, t.moisture, t.tilt, t.raw,
    t.alarm ? "true" : "false", (unsigned long)t.alarmCount,
    t.mpuOk ? "true" : "false",
    (WiFi.status() == WL_CONNECTED) ? "true" : "false",
    (int)WiFi.RSSI(), (unsigned long)wifiReconnects, (unsigned long)millis());

  HTTPClient http;
  http.setConnectTimeout(HTTP_TIMEOUT_MS);
  http.setTimeout(HTTP_TIMEOUT_MS);
  if (!http.begin(BACKEND_URL)) return -1;
  http.addHeader("Content-Type", "application/json");
  int code = http.POST((uint8_t *)body, strlen(body));
  http.end();
  return code;
}

void networkTask(void *) {
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);          // don't wear out flash with credentials
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t lastAttempt = millis();
  uint32_t lastPost    = 0;
  bool     wasUp       = false;

  for (;;) {
    bool up = (WiFi.status() == WL_CONNECTED);
    wifiUp = up;

    if (up && !wasUp) { Serial.printf("[WIFI] connected, IP %s\n", WiFi.localIP().toString().c_str()); }
    if (!up && wasUp) { wifiReconnects++; Serial.println(F("[WIFI] link lost - local reflex unaffected")); }
    wasUp = up;

    if (!up) {
      // Manual retry on a timer (never blocks sensing; this is a separate task).
      if (millis() - lastAttempt >= WIFI_RETRY_MS) {
        lastAttempt = millis();
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        Serial.println(F("[WIFI] reconnect attempt"));
      }
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    if (urgentSend || (millis() - lastPost >= TELEMETRY_PERIOD_MS)) {
      urgentSend = false;
      Telemetry snap;
      portENTER_CRITICAL(&telemMux);   // copy a consistent snapshot
      snap = sharedTelem;
      portEXIT_CRITICAL(&telemMux);

      int code = postTelemetry(snap);
      lastPost = millis();
      Serial.printf("[HTTP] POST -> %d\n", code);
      if (code < 200 || code >= 300) {
        if (snap.alarm) urgentSend = true;   // alarm state must reach the server: retry
        lastPost = millis() - TELEMETRY_PERIOD_MS + 2000UL;  // retry in ~2 s
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ============================================================================
//  8. ARDUINO ENTRY POINTS
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("\n=== FORECAST node booting ==="));

  // ---- outputs first, so the gate is in a known (open) state ----
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, BUZZER_ACTIVE_HIGH ? LOW : HIGH);   // silent
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  ESP32PWM::allocateTimer(0);
  gateServo.setPeriodHertz(50);
  gateServo.attach(PIN_SERVO, SERVO_MIN_US, SERVO_MAX_US);
  gateServo.write(SERVO_OPEN_ANGLE);
  servoAngleNow = SERVO_OPEN_ANGLE;

  // ---- ADC ----
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_MOISTURE_ADC, ADC_11db);         // ~0..3.1 V range

  // ---- I2C + MPU6050 ----
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  mpuOk = initMpu();
  Serial.println(mpuOk ? F("[MPU] OK") : F("[MPU] NOT FOUND - check wiring (will keep retrying)"));

  // ---- tilt baseline: reuse saved one, or capture a new one ----
  bool haveSaved = false;
  if (USE_SAVED_BASELINE) {
    prefs.begin("forecast", true);
    haveSaved = prefs.getBool("bok", false);
    if (haveSaved) {
      baseX = prefs.getFloat("bx", 0); baseY = prefs.getFloat("by", 0); baseZ = prefs.getFloat("bz", 1);
    }
    prefs.end();
  }
  if (haveSaved) {
    baselineValid = true;
    Serial.printf("[BASELINE] loaded from flash: (%.4f, %.4f, %.4f)\n", baseX, baseY, baseZ);
  } else {
    startBaselineCapture();           // first boot: install node level, then power up
  }

  // ---- network task on core 0; loop() (sensing + reflex) stays on core 1 ----
  xTaskCreatePinnedToCore(networkTask, "net", 8192, nullptr, 1, nullptr, 0);

  Serial.println(F("=== running ==="));
}

void loop() {
  static uint32_t nextSample = 0;
  static uint32_t lastLog    = 0;
  uint32_t now = millis();

  // ---- fixed-rate sampling (no delay(); catch-up safe) ----
  if ((int32_t)(now - nextSample) >= 0) {
    nextSample = now + SAMPLE_INTERVAL_MS;
    sampleSensors();
    evaluateReflex();

    // Publish a snapshot for the network task.
    portENTER_CRITICAL(&telemMux);
    sharedTelem.moisture   = moistAvg;
    sharedTelem.tilt       = tiltAvg;
    sharedTelem.raw        = lastRaw;
    sharedTelem.alarm      = alarmActive;
    sharedTelem.mpuOk      = mpuOk;
    sharedTelem.alarmCount = alarmCount;
    portEXIT_CRITICAL(&telemMux);
  }

  handleButton();
  updateOutputs();          // runs every pass so the buzzer pattern is smooth

  if (now - lastLog >= SERIAL_LOG_MS) {
    lastLog = now;
    Serial.printf("raw=%d  moist=%.1f%%  tilt=%.2f deg  win=%u/%lu  alarm=%d  wifi=%d  mpu=%d\n",
                  lastRaw, moistAvg, tiltAvg, winCount, (unsigned long)WINDOW_SAMPLES,
                  alarmActive, wifiUp, mpuOk);
  }
}
