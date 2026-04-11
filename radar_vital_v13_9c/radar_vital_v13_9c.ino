/* radar_vital_v13_9c.ino
 *
 * XIAO ESP32-C6 + MR60BHA2 60 GHz FMCW radar + MLX90614 + HD44780 20x4 LCD
 * + Active Buzzer for audio feedback
 *
 * Firmware release: v13.9c
 * CSV schema release: v13.9c / trainer contract v8.5c
 *
* Manuscript-facing calibration / release notes
* -------------------------------------------
* + FW_VERSION is v13.9c.
* + The raw CSV now exposes both sketch identity and radar-module identity:
*   sketch_major/sketch_sub/sketch_mod and
*   module_fw_major/module_fw_sub/module_fw_mod.
* + The stabilized phase columns are now named
*   heart_phase_stabilized and breath_phase_stabilized.
* + The v13.9c telemetry tail retains the branch-stable v13.9a schema and
*   adds publish-readiness and anchor-drift policy diagnostics.
* + The main calibration constants introduced in the v13.7 line remain:
*   - CHIP_HR_BIAS_CORRECTION_BPM = 6.0f (blind raw-only reseed damping)
*   - clutter warmup alpha: 0.08 -> 0.005 over valid-phase warmup samples
*   - anchor-aware RR harmonic guard around the smoothRR anchor
*
* =========================================================================
* CHANGELOG v13.9c (publish readiness + drift brake hardening)
* =========================================================================
* + Require phase-backed publish readiness before HR/RR rows can become
*   logged-valid after settling or motion recovery.
* + Add weak-PQI anchor-drift braking so locked windows resist rapid
*   tracking drift under marginal evidence.
* + Preserve the widened v13.9a telemetry contract so analysis artifacts
*   remain comparable across 13.9a/13.9c.
*
* =========================================================================
* CHANGELOG v13.7.5 (Audit Bugfixes & Improvements)
 * =========================================================================
 * + Guard trustedPhaseHR write with a validity gate on the candidate source
 *   (HR_PATH_AUTO or HR_PATH_SPECTRAL).
 * + Widen CSV telemetry payload to 136 columns to explicitly emit 
 *   rr_seed_from_raw_used rather than having the dashboard derive it.
 * + Ensure CSV values for NaN explicitly print as -1.0 using explicit sentinel block.
 *
 * =========================================================================
 * CHANGELOG v13.7.4 (Lock-First anchoring + widened CSV telemetry)
 * =========================================================================
 * + Implement Lock-First anchor logic: added currentGoverningHRAnchor() and
 *   hrPublishGraceBlocked logic to prevent raw-HR bias from poisoning the
 *   session if a trusted phase-derived anchor is available.
 * + Widen the DATA telemetry payload from 131 to 135 columns.
 * + Add anchor-diagnostic telemetry: hr_trusted_phase_anchor,
 *   hr_anchor_source, hr_anchor_err_bpm, and hr_raw_high_bias_suspect.
 * + Add trackingHR and lastTrackingHRMs to maintain more granular tracking
 *   vs. raw candidate state history.
 * + Synchronize header/version records and CSV column count documentation.
 *
 * =========================================================================
 * CHANGELOG v13.7.3 (release alignment for trainer/dashboard truthfulness hardening)
 * =========================================================================
 * + Correct the manuscript/header identity to match the real file and release:
 *   radar_vital_v13.7.3.ino / FW_VERSION v13.7.3.
 * + Keep the v13.7 DSP logic unchanged while promoting the paired trainer/dashboard
 *   hardening release to v8.3.3 for stale-value blanking, RR-stage truthfulness,
 *   and corrected firmware-version null handling.
 * + Confirm the live CSV tail contract includes:
 *   phase_warmup_complete, clutter_warmup_count, current_clutter_alpha,
 *   use_fast_path, phase_valid_this_frame, hr_path_source,
 *   fw_major, fw_sub, fw_mod.
 * + Preserve hr_path_source as a dashboard/trainer-compatible mirror of the
 *   current HR update source so older UI consumers do not silently lose the
 *   path field after the v13.7 telemetry additions.
 * + No DSP-path logic change is intentionally introduced in firmware for this
 *   point release; the substantive fixes land in the paired v8.3.3 trainer and
 *   dashboard truthfulness layer.
 *
 * =========================================================================
 * CHANGELOG v13.7 (phase-valid truthfulness + safer seeding + clutter experiment)
 * =========================================================================
 * + Add phase_valid_this_frame so CSV rows distinguish current-frame phase
 *   validity from the older phase_fresh / seen-recently semantics.
 * + Reset hrGateReason and rrGateReason at the start of each DSP window so
 *   gate telemetry is current-window truthful instead of stale-held.
 * + Tighten first trusted HR seeding: the first session seed now waits for a
 *   valid phase-derived candidate instead of allowing blind raw-only anchor
 *   poisoning at startup.
 * + Replace the blind 18-26 br/min RR raw-harmonic guard with an anchor-aware
 *   check around the current smoothRR estimate.
 * + Add phaseWarmupComplete, clutterWarmupCount, and currentClutterAlpha for
 *   warmup instrumentation and the bounded clutter-alpha experiment.
 * + Introduce a phase-gated dynamic clutter alpha that starts high during the
 *   earliest valid-phase warmup and decays toward the steady-state floor.
 *
 * =========================================================================
 * CHANGELOG v13.6 (window expansion + triple-agreement telemetry + phase labels)
 * =========================================================================
 * + Increase BUF_SIZE from 256 to 384 and WIN_FULL from 160 to 256 so long
 *   sessions can accumulate a wider HR/RR analysis window without truncating
 *   the active ring buffer at the older ceiling.
 * + Add a third zero-crossing estimator on the filtered heart and breath
 *   paths, then log HR/RR triple-agreement flags against the existing
 *   autocorrelation and spectral estimators for cleaner training labels.
 * + Add trainer-facing estimator telemetry: hr_zc_bpm/hr_zc_conf,
 *   hr_spec_bpm/hr_spec_mag, rr_zc_bpm/rr_zc_conf, rr_spec_bpm/rr_spec_conf,
 *   plus hr_triple_agree and rr_triple_agree.
 * + Add harmonic_mode as a consolidated bitmask label so windows affected by
 *   RR raw-harmonic correction, HR half-rate ambiguity, arbiter correction,
 *   or RR harmonic/subharmonic rejection remain learnable instead of being
 *   visible only through scattered gate fields.
 * + Add session_phase as a compact numeric phase label:
 *   0=absent, 1=warmup, 2=settling, 3=locked, 4=post_motion, 5=leaving.
 * + Keep paired raw Doppler index and scaled cm/s telemetry explicit for
 *   downstream calibration work; trainer v8.2 consumes the raw index and the
 *   scaled speed side by side instead of trusting the physics conversion alone.
 * + Re-tighten v13_rawHRInRestingZone() upper bound to 80 BPM after the s32
 *   smoke test showed the wider 48-100 BPM band could reopen use_fast_path and
 *   reintroduce the older raw-HR sum-frequency / intermodulation leakage.
 *
 * =========================================================================
 * Legacy changelog history retained below
 * =========================================================================
 */
 
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <driver/gpio.h>
#include <LiquidCrystal_I2C.h>
#include "Seeed_Arduino_mmWave.h"
#include <Adafruit_MLX90614.h>
#include <BH1750.h>
#include <Adafruit_NeoPixel.h>
#include <esp_task_wdt.h>
#include <Preferences.h>

#ifdef ESP32
#include <HardwareSerial.h>
#if defined(CONFIG_IDF_TARGET_ESP32C6) || \
    defined(CONFIG_IDF_TARGET_ESP32C3) || \
    defined(CONFIG_IDF_TARGET_ESP32H2)
HardwareSerial mmWaveSerial(0);
#else
HardwareSerial mmWaveSerial(1);
#endif
#else
#define mmWaveSerial Serial1
#endif

#ifndef PI
#define PI 3.14159265358979323846f
#endif

static inline float safe_float(float x) {
  return isfinite(x) ? x : 0.0f;
}

// =========================================================================
// PIN DEFINITIONS
// =========================================================================
#ifndef D0
#define D0 0
#endif
#ifndef D1
#define D1 1
#endif
#define BUZZER_PIN D0
#define NEOPIXEL_PIN D1

// =========================================================================
// LOGGING & OBSERVABILITY
// =========================================================================
#define LOG_MODE 1       // 1 = Enable CSV "DATA,..." logging
#define FW_VERSION "v13.9c"
#define SKETCH_VERSION_MAJOR 13
#define SKETCH_VERSION_SUB 9
#define SKETCH_VERSION_MOD 3
#define DIAG_PLOTTER 0   // 1 = Enable live Serial Plotter DSP diagnostics, 0 = Off
#define LOG_INTERVAL_MS 200

// =========================================================================
// BUZZER TIMING CONSTANTS
// =========================================================================
#define BEEP_VERY_SHORT 40
#define BEEP_SHORT 100
#define BEEP_MEDIUM 180
#define BEEP_LONG 350
#define BEEP_VERY_LONG 600
#define BEEP_GAP_SHORT 100
#define BEEP_GAP_MEDIUM 180
#define BEEP_GAP_LONG 300
#define BEEP_CLICK 20
#define BUZZER_SAFETY_TIMEOUT_MS 2500
#define BUZZER_MOTION_WARNING_ENABLED false

// =========================================================================
// BUZZER EVENT ENUM
// =========================================================================
enum BuzzerEvent {
  BUZZ_NONE = 0, BUZZ_STARTUP, BUZZ_CALIBRATION_DONE, BUZZ_CALIBRATION_FAIL,
  BUZZ_PERSON_DETECTED, BUZZ_PERSON_LEFT, BUZZ_HR_LOCKED,
  BUZZ_HR_HIGH_ALERT, BUZZ_HR_LOW_ALERT, BUZZ_MOTION_WARNING,
  BUZZ_SENSOR_ERROR, BUZZ_CLICK
};

// =========================================================================
// FORWARD DECLARATIONS & TYPES
// =========================================================================
struct SpectralResult { float freq; float mag; };

void buzzerInit(); void buzzerOn(); void buzzerOff(); void buzzerForceOff();
void buzzerPlay(BuzzerEvent event); bool buzzerIsBusy(); void buzzerUpdate();
void checkHRAlerts(float hr, bool hrValid);
void i2cRecover(); void resetVitals(); void lcdCreateChars();
bool scanForLCD(); static bool probeI2C(uint8_t addr);
static bool tryInitBH1750();
static bool i2cSafeReadLux(unsigned long now);
static bool i2cSafeReadMLX(unsigned long now, float& amb, float& obj);
extern unsigned long lastMlxRetry;
void detectSpectral(float* buf, int n, float fs, float fmin, float fmax, float& outFreq, float& outMag);
bool detectLowestSpectralFundamental(float* buf, int n, float fs, float fmin, float fmax, float& outBpm, float& outConf, float anchorBpm = 0.0f);
static inline float heartPQIWeight(float pqi);
static inline float currentHeartPQIGate();
void renderWelcomeScreen(); void renderGoodbyeScreen();

// =========================================================================
// BUZZER STATE
// =========================================================================
static bool buzzerEnabled = true;
static bool buzzerSilentMode = false;
static BuzzerEvent currentBuzzEvent = BUZZ_NONE;
static int buzzStep = 0;
static unsigned long buzzStepStartMs = 0;
static unsigned long buzzEventStartMs = 0;
static bool buzzPinState = false;

const float HR_ALERT_HIGH = 130.0f;
const float HR_ALERT_LOW = 45.0f;
const unsigned long HR_ALERT_COOLDOWN = 60000UL;
const unsigned long DEBOUNCE_HR_LOCKED = 60000UL;
const unsigned long DEBOUNCE_MOTION = 30000UL;
static unsigned long lastHRAlertMs = (unsigned long)(0UL - HR_ALERT_COOLDOWN);
static bool hrAlertActive = false;
static unsigned long lastHRLockedBeepMs = 0;
static unsigned long lastMotionBeepMs = 0;

// =========================================================================
// BUZZER FUNCTIONS
// =========================================================================
void buzzerInit() {
  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);
  buzzPinState = false; currentBuzzEvent = BUZZ_NONE;
  buzzStep = 0;
  Serial.println("[BUZZER] Initialized on D0");
}
void buzzerOn() {
  if (!buzzPinState && buzzerEnabled && !buzzerSilentMode) {
    digitalWrite(BUZZER_PIN, HIGH);
    buzzPinState = true;
  }
}
void buzzerOff() {
  if (buzzPinState) { digitalWrite(BUZZER_PIN, LOW); buzzPinState = false; }
}
void buzzerForceOff() {
  digitalWrite(BUZZER_PIN, LOW); buzzPinState = false;
  currentBuzzEvent = BUZZ_NONE; buzzStep = 0;
}
void buzzerPlay(BuzzerEvent event) {
  if (!buzzerEnabled || buzzerSilentMode) return;
  if (currentBuzzEvent != BUZZ_NONE) return;
  currentBuzzEvent = event;
  buzzStep = 0;
  unsigned long t = millis();
  buzzStepStartMs = t;
  buzzEventStartMs = t;
  Serial.printf("[BUZZER] Event: %d\n", (int)event);
}
bool buzzerIsBusy() { return currentBuzzEvent != BUZZ_NONE; }

void buzzerUpdate() {
  unsigned long now = millis();
  if (currentBuzzEvent != BUZZ_NONE) {
    if (now - buzzEventStartMs > BUZZER_SAFETY_TIMEOUT_MS) {
      Serial.println("[BUZZER] Safety timeout!");
      buzzerForceOff(); return;
    }
  }
  if (currentBuzzEvent == BUZZ_NONE) { if (buzzPinState) buzzerOff(); return; }
  if (!buzzerEnabled || buzzerSilentMode) { buzzerForceOff(); return; }
  unsigned long elapsed = now - buzzStepStartMs;
  switch (currentBuzzEvent) {
    case BUZZ_STARTUP:
      switch (buzzStep) {
        case 0: buzzerOn();
          if (elapsed >= BEEP_SHORT) { buzzerOff(); buzzStep = 1; buzzStepStartMs = now; } break;
        case 1: if (elapsed >= BEEP_GAP_MEDIUM) { buzzStep = 2; buzzStepStartMs = now; } break;
        case 2: buzzerOn();
          if (elapsed >= BEEP_SHORT) { buzzerOff(); currentBuzzEvent = BUZZ_NONE; buzzStep = 0; } break;
      } break;
    case BUZZ_CALIBRATION_DONE:
      switch (buzzStep) {
        case 0: buzzerOn();
          if (elapsed >= BEEP_VERY_SHORT) { buzzerOff(); buzzStep = 1; buzzStepStartMs = now; } break;
        case 1: if (elapsed >= BEEP_GAP_SHORT) { buzzStep = 2; buzzStepStartMs = now; } break;
        case 2: buzzerOn();
          if (elapsed >= BEEP_SHORT) { buzzerOff(); buzzStep = 3; buzzStepStartMs = now; } break;
        case 3: if (elapsed >= BEEP_GAP_SHORT) { buzzStep = 4; buzzStepStartMs = now; } break;
        case 4: buzzerOn();
          if (elapsed >= BEEP_MEDIUM) { buzzerOff(); currentBuzzEvent = BUZZ_NONE; buzzStep = 0; } break;
      } break;
    case BUZZ_CALIBRATION_FAIL:
      if (buzzStep == 0) { buzzerOn(); if (elapsed >= BEEP_VERY_LONG) { buzzerOff();
          currentBuzzEvent = BUZZ_NONE; buzzStep = 0; } } break;
    case BUZZ_PERSON_DETECTED:
      if (buzzStep == 0) { buzzerOn();
          if (elapsed >= BEEP_SHORT) { buzzerOff(); currentBuzzEvent = BUZZ_NONE; buzzStep = 0; } } break;
    case BUZZ_PERSON_LEFT:
      switch (buzzStep) {
        case 0: buzzerOn();
          if (elapsed >= BEEP_MEDIUM) { buzzerOff(); buzzStep = 1; buzzStepStartMs = now; } break;
        case 1: if (elapsed >= BEEP_GAP_MEDIUM) { buzzStep = 2; buzzStepStartMs = now; } break;
        case 2: buzzerOn();
          if (elapsed >= BEEP_SHORT) { buzzerOff(); currentBuzzEvent = BUZZ_NONE; buzzStep = 0; } break;
      } break;
    case BUZZ_HR_LOCKED:
      if (buzzStep == 0) { buzzerOn(); if (elapsed >= BEEP_VERY_SHORT) { buzzerOff();
          currentBuzzEvent = BUZZ_NONE; buzzStep = 0; } } break;
    case BUZZ_HR_HIGH_ALERT:
      switch (buzzStep) {
        case 0: buzzerOn();
          if (elapsed >= BEEP_SHORT) { buzzerOff(); buzzStep = 1; buzzStepStartMs = now; } break;
        case 1: if (elapsed >= BEEP_GAP_SHORT) { buzzStep = 2; buzzStepStartMs = now; } break;
        case 2: buzzerOn();
          if (elapsed >= BEEP_SHORT) { buzzerOff(); buzzStep = 3; buzzStepStartMs = now; } break;
        case 3: if (elapsed >= BEEP_GAP_SHORT) { buzzStep = 4; buzzStepStartMs = now; } break;
        case 4: buzzerOn();
          if (elapsed >= BEEP_SHORT) { buzzerOff(); currentBuzzEvent = BUZZ_NONE; buzzStep = 0; } break;
      } break;
    case BUZZ_HR_LOW_ALERT:
      switch (buzzStep) {
        case 0: buzzerOn();
          if (elapsed >= BEEP_LONG) { buzzerOff(); buzzStep = 1; buzzStepStartMs = now; } break;
        case 1: if (elapsed >= BEEP_GAP_LONG) { buzzStep = 2; buzzStepStartMs = now; } break;
        case 2: buzzerOn();
          if (elapsed >= BEEP_LONG) { buzzerOff(); currentBuzzEvent = BUZZ_NONE; buzzStep = 0; } break;
      } break;
    case BUZZ_MOTION_WARNING:
      if (buzzStep == 0) { buzzerOn(); if (elapsed >= BEEP_VERY_SHORT) { buzzerOff();
          currentBuzzEvent = BUZZ_NONE; buzzStep = 0; } } break;
    case BUZZ_SENSOR_ERROR:
      if (buzzStep == 0) { buzzerOn();
          if (elapsed >= BEEP_LONG) { buzzerOff(); currentBuzzEvent = BUZZ_NONE; buzzStep = 0; } } break;
    case BUZZ_CLICK:
      if (buzzStep == 0) { buzzerOn(); if (elapsed >= BEEP_CLICK) { buzzerOff();
          currentBuzzEvent = BUZZ_NONE; buzzStep = 0; } } break;
    default: buzzerForceOff(); break;
  }
}

void checkHRAlerts(float hr, bool hrValid) {
  bool outOfRange = hrValid && isfinite(hr) && hr > 0.0f &&
                    (hr > HR_ALERT_HIGH || (hr < HR_ALERT_LOW && hr > 0.0f));
  hrAlertActive = outOfRange;
  if (!outOfRange) return;
  if (!buzzerEnabled || buzzerSilentMode) return;
  if (buzzerIsBusy()) return;
  unsigned long now = millis();
  if (now - lastHRAlertMs < HR_ALERT_COOLDOWN) return;
  if (hr > HR_ALERT_HIGH) {
    buzzerPlay(BUZZ_HR_HIGH_ALERT);
    lastHRAlertMs = now;
    Serial.printf("[ALERT] High HR: %.0f\n", hr);
  } else {
    buzzerPlay(BUZZ_HR_LOW_ALERT);
    lastHRAlertMs = now;
    Serial.printf("[ALERT] Low HR: %.0f\n", hr);
  }
}

// =========================================================================
// USER-FRIENDLY LABEL CONSTANTS
// =========================================================================
const char* STATUS_LOCKED = "LOCK";
const char* STATUS_SENSING = "SENS";
const char* STATUS_WAITING = "WAIT";
const char* STATUS_MOVING = "MOVE";
const char* STATUS_HOLD = "HOLD";
const char* STATUS_EXIT = "EXIT";
const char* HR_ZONE_REST = "Rest";
const char* HR_ZONE_NORMAL = "OK";
const char* HR_ZONE_ACTIVE = "Act";
const char* HR_ZONE_HIGH = "High";
const char* TEMP_COLD = "Cold";
const char* TEMP_COMFORT = "OK";
const char* TEMP_WARM = "Warm";
const char* HINTS[] = {
  "Stand 50-150cm away",
  "Stay still",
  "Face the sensor",
  "Breathe normally",
  "Avoid thick clothes"
};
const int NUM_HINTS = 5;

// =========================================================================
// CONFIGURATION
// =========================================================================
const float MLX_EMISSIVITY = 0.98f;
static const unsigned long NVS_WRITE_MIN_INTERVAL_MS = 600000UL;
static unsigned long lastNvsWriteMs = 0;
static bool nvsEverWritten = false;
static bool wdtActive = false;
static const unsigned long RADAR_RECOVERY_GRACE_MS = 3000UL;
static const unsigned long BH1750_RETRY_INTERVAL_MS = 10000UL;
static unsigned long lastBh1750RetryMs = 0UL;
static unsigned long lastLcdRescanMs = 0UL;
static inline void wdtReset() { if (wdtActive) esp_task_wdt_reset(); }
static inline void wdtResetEvery(int i, int interval) { if (interval > 0 && (i % interval) == 0) wdtReset(); }
static const size_t MMWAVE_RX_BUFFER_SIZE = 4096;
static const int RADAR_UART_BACKLOG_BYTES = 192;
static const int LCD_COLS = 20;
static const int LCD_ROWS = 4;

SEEED_MR60BHA2 mmWave;

// =========================================================================
// v13 SPATIAL / MODULE TELEMETRY
// =========================================================================
static PeopleCounting lastPointCloud;
static PeopleCounting lastTargetInfo;
static bool lastPointCloudValid = false;
static bool lastTargetInfoValid = false;
static int numDetectedTargets = -1;
static float maxDopplerAbs = 0.0f;
static float maxDopplerSpeedCms = 0.0f;
static bool dopplerMotionDetected = false;
static bool clusterAnomaly = false;
static bool multiTargetDetected = false;
static float primaryTargetX = 0.0f;
static float primaryTargetY = 0.0f;
static int32_t primaryTargetDop = 0;
static float primaryTargetSpeedCms = 0.0f;
static int32_t primaryTargetCluster = -1;
static int spatialSource = 0; // 0=none, 1=target_info, 2=point_cloud_fallback
static unsigned long spatialFreshAgeMs = 999999UL;
static FirmwareInfo moduleVersion;
static bool moduleVersionValid = false;
static bool fwVersionCheckedThisBoot = false;
static unsigned long lastPointCloudRxMs = 0UL;
static unsigned long lastTargetInfoRxMs = 0UL;
static bool useDirectRawHR = false;
static uint8_t directRawHR_consecWindows = 0;
LiquidCrystal_I2C* lcdPtr = nullptr;
uint8_t lcdAddr = 0;
Adafruit_MLX90614 mlx;
BH1750 lightMeter;
Adafruit_NeoPixel pixel(1, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
Preferences prefs;

bool lcdConnected = false;
bool mlxReady = false;
bool bh1750Ready = false;
uint8_t bh1750Addr = 0x5C;
float luxLevel = 300.0f;

static uint8_t lcdObjBuf[sizeof(LiquidCrystal_I2C)] __attribute__((aligned(4)));
static bool lcdObjAllocated = false;
static uint32_t lastLedColor = 0xFFFFFFFF;
static uint8_t lcdRowCache[LCD_ROWS][LCD_COLS];
static bool lcdRowCacheValid[LCD_ROWS] = {false, false, false, false};

// =========================================================================
// DISPLAY STATE & TIMING VARIABLES [v11.0.0]
// =========================================================================
enum DisplayState {
  DISP_NONE, DISP_SPLASH, DISP_INIT_SENSORS, DISP_CALIBRATING,
  DISP_CALIB_RESULT, DISP_WELCOME, DISP_VITALS, DISP_GOODBYE, DISP_IDLE
};
static DisplayState dispState = DISP_NONE;
static DisplayState prevDispState = DISP_NONE;

// =========================================================================
// PRESENCE STATE MACHINE [v11 foundation = corrected v10.8.9c core]
// =========================================================================
static const uint8_t PRESENCE_ABSENT = 0;
static const uint8_t PRESENCE_ENTERING = 1;
static const uint8_t PRESENCE_PRESENT = 2;
static const uint8_t PRESENCE_SILENT_HOLD = 3;
static const uint8_t PRESENCE_LEAVING = 4;

static uint8_t presenceState = PRESENCE_ABSENT;
static unsigned long presenceStateSinceMs = 0;
static unsigned long lastStrongPresenceMs = 0;
static unsigned long lastWeakPresenceMs = 0;
static unsigned long lastLivePhaseMs = 0;
static unsigned long reentryLockoutMs = 0;
static unsigned long lastEvidenceLogMs = 0;
static unsigned long lastLatchedEvidenceMs = 0;

static const unsigned long ENTER_CONFIRM_MS = 1200UL;
static const unsigned long ENTER_CONFIRM_STRONG_MS = 400UL;
static const unsigned long LEAVE_CONFIRM_MS = 4000UL;
static const unsigned long SILENT_HOLD_MS = 12000UL;
static const unsigned long REENTRY_LOCKOUT_MS = 4000UL;
static const unsigned long WEAK_ENTRY_RETRY_COOLDOWN_MS = 1500UL;
static const unsigned long LIVE_PHASE_HOLD_MS = 5000UL;
static const unsigned long LIVE_PHASE_PACKET_MAX_MS = 3000UL;
static const unsigned long RADAR_PRESENCE_EVIDENCE_MS = 1000UL;
static const unsigned long DSP_PRESENCE_EVIDENCE_MS = 1500UL;
static const unsigned long DISTANCE_PRESENCE_EVIDENCE_MS = 1000UL;
static const unsigned long PHASE_LIVENESS_DECAY_MS = 200UL;
static const unsigned long CONFIDENCE_DECAY_MS = 250UL;
static const unsigned long EVIDENCE_LATCH_MAX_MS = 1000UL;
static const unsigned long AMBIENT_EVIDENCE_HOLD_MS = 2000UL;

static float phaseLivenessScore = 0.0f;
static float presenceConfidence = 0.0f;
static float distanceConfidence = 0.0f;
static unsigned long lastPhaseLivenessDecayMs = 0;
static unsigned long lastConfidenceDecayMs = 0;
static unsigned long lastPresenceTickMs = 0;
static bool latchedStrongEvidence = false;
static bool latchedWeakEvidence = false;
static bool entrySawStrongEvidence = false;
static unsigned long entryStrongSinceMs = 0;
static unsigned long weakEntryCooldownMs = 0;
static unsigned long calibResultStartMs = 0;
static const unsigned long CALIB_RESULT_DISPLAY_MS = 2000UL;
static const unsigned long WELCOME_DISPLAY_MS = 1500UL;
static const unsigned long GOODBYE_DISPLAY_MS = 1200UL;
static unsigned long lastCalibDisplay = 0;
static int lastCalibPct = -1;

static unsigned long lastHRUpdateMs = 0;
static unsigned long lastRRUpdateMs = 0;
static unsigned long lastDistanceUpdateMs = 0;
static unsigned long lastPresenceUpdateMs = 0;
static unsigned long lastPhaseDataMs = 0;
static unsigned long lastValidRateMs=0;
static unsigned long lastTrustedVitalMs = 0;
static unsigned long lastTrustedHRMs = 0;
static unsigned long lastTrustedRRMs = 0;
static float trustedPhaseHR = 0.0f;
static unsigned long lastTrustedPhaseHRMs = 0UL;
static float trackingHR = 0.0f;
static unsigned long lastTrackingHRMs = 0UL;
static uint8_t hrLowerPersistWindows = 0;
static uint8_t hrUpwardConfirmWindows = 0;
static uint8_t hrGraceBlockWindows = 0;
static bool hrDownwardOverrideActive = false;
static bool hrPublishGraceBlocked = false;

// [v11.0.0] Freshness Constants for Dual-Pipeline
static const unsigned long HR_STALE_MS = 10000UL;
static const unsigned long RR_STALE_MS = 10000UL;
static const unsigned long DISTANCE_STALE_MS = 8000UL;
static const unsigned long PRESENCE_STALE_MS = 5000UL;
static const unsigned long HR_DISPLAY_GRACE_MS = 3000UL;

static const unsigned long LOG_VITAL_GRACE_MS = 2500UL;
static const unsigned long HR_PUBLISH_MAX_AGE_MS = 3000UL;
static const unsigned long CANDIDATE_FRESH_MS = 2000UL;
static const unsigned long PHASE_FRESH_REQ_MS = 2500UL;
static const unsigned long PHASE_STALE_DOWNGRADE_MS = 3000UL;

static unsigned long lastHRDisplayedMs = 0;
static float lastDisplayedHR = 0;
static bool hrVisibleOnVitalsScreen = false;
static unsigned long lastIdleRedrawMs = 0;
static unsigned long lastMotionDetectedMs = 0;
static const unsigned long MOTION_PERSIST_MS = 500UL;
static bool persistedMotion = false;
static float lastSavedGain = -1.0f;
static float lastSavedHR = -1.0f;
static float lastSavedRR = -1.0f;
static bool nvsGainValid = false;
static float nvsRestoredGain = 1.0f;
static uint8_t lastLedBrightness = 0xFF;
static unsigned long welcomeStartMs = 0;
static unsigned long goodbyeStartMs = 0;
static bool welcomeShown = false;
static unsigned long lastTransitionMs = 0;
static const unsigned long TRANSITION_DELAY_MS = 50UL;
static bool animPending = false;
static bool animPendingSmall = false;

static int medWarmup = 0;
static unsigned long lastAmbientJumpMs = 0;
static unsigned long lastAmbientVoteMs = 0;
static int detectWindow_snap = 0;
static bool skipDSP = false; 
static int skipDSP_consecMisses = 0;
static const int SKIP_DSP_MISS_THRESH = 3; 

#define DIST_VAR_WIN 15
static float distVarBuf[DIST_VAR_WIN] = {0};
static int distVarIdx = 0;
static int distVarCount = 0;
static uint8_t ghostSuspectCounter = 0; 
static float currentDistStdDev = 0.0f;
static float dbEnvLP = 0.0f;
static bool currentStaticReflector = false;
static uint8_t staticReflectorConsecutive = 0;
static const float STATIC_REFLECTOR_MAX_STD_CM = 1.5f;
static const uint8_t STATIC_REFLECTOR_REQUIRED_CONSECUTIVE = 20; // ~4s at 5Hz fresh distance samples

// =========================================================================
// DIAGNOSTIC GLOBALS
// =========================================================================
static float diagAvgE_Pre  = 0.0f;
static float diagAvgE_RLS  = 0.0f;
static float diagAvgE_Gate = 0.0f;

// =========================================================================
// LCD CUSTOM CHARACTER DATA
// =========================================================================
#define CHAR_HEART 0
#define CHAR_SCAN2 1
#define CHAR_BREATH 2
#define CHAR_DOT_FULL 3
#define CHAR_DOT_EMPTY 4
#define CHAR_DEGREE 5
#define CHAR_PERSON 6
#define CHAR_SCAN 7

byte charHeart[8] = { 0x00, 0x0A, 0x1F, 0x1F, 0x1F, 0x0E, 0x04, 0x00 };
byte charHeartSmall[8] = { 0x00, 0x0A, 0x15, 0x11, 0x0A, 0x04, 0x00, 0x00 };
byte charBreath[8] = { 0x0E, 0x1F, 0x1F, 0x0E, 0x0E, 0x1F, 0x1F, 0x0E };
byte charDotFull[8] = { 0x00, 0x0E, 0x1F, 0x1F, 0x1F, 0x0E, 0x00, 0x00 };
byte charDotEmpty[8] = { 0x00, 0x0E, 0x11, 0x11, 0x11, 0x0E, 0x00, 0x00 };
byte charDegree[8] = { 0x06, 0x09, 0x09, 0x06, 0x00, 0x00, 0x00, 0x00 };
byte charPerson[8] = { 0x04, 0x0E, 0x04, 0x1F, 0x04, 0x0A, 0x11, 0x00 };
byte charScan1[8] = { 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E, 0x1F };
byte charScan2_data[8] = { 0x04, 0x0A, 0x04, 0x04, 0x04, 0x04, 0x0E, 0x1F };

static bool heartAnimFrame = false;
static bool customCharsValid = false;

// =========================================================================
// LCD CHAR / UTILITY FUNCTIONS
// =========================================================================
static inline bool radarSerialBacklogged(int threshold = RADAR_UART_BACKLOG_BYTES) {
  return mmWaveSerial.available() >= threshold;
}

static void invalidateLcdRowCache() {
  memset(lcdRowCache, ' ', sizeof(lcdRowCache));
  memset(lcdRowCacheValid, 0, sizeof(lcdRowCacheValid));
}

static void lcdBuildBlankRow(uint8_t* row) {
  if (!row) return;
  memset(row, ' ', LCD_COLS);
}

static void lcdCopyTextToRow(uint8_t* row, int start, int width, const char* text) {
  if (!row || !text || start >= LCD_COLS || width <= 0) return;
  if (start < 0) {
    width += start;
    start = 0;
  }
  if (width <= 0) return;
  int limit = min(width, LCD_COLS - start);
  int len = min((int)strlen(text), limit);
  for (int i = 0; i < len; ++i) row[start + i] = (uint8_t)text[i];
}

static void lcdBuildCenteredRow(uint8_t* row, const char* text) {
  lcdBuildBlankRow(row);
  if (!text) return;
  int len = min((int)strlen(text), LCD_COLS);
  int start = (LCD_COLS - len) / 2;
  for (int i = 0; i < len; ++i) row[start + i] = (uint8_t)text[i];
}

static void lcdWriteCachedRowRaw(int row, const uint8_t* rowData, bool force = false) {
  if (!lcdPtr || !rowData || row < 0 || row >= LCD_ROWS) return;

  int col = 0;
  while (col < LCD_COLS) {
    bool changed = force || !lcdRowCacheValid[row] || lcdRowCache[row][col] != rowData[col];
    if (!changed) {
      ++col;
      continue;
    }

    int start = col;
    while (col < LCD_COLS &&
           (force || !lcdRowCacheValid[row] || lcdRowCache[row][col] != rowData[col])) {
      ++col;
    }

    lcdPtr->setCursor(start, row);
    for (int i = start; i < col; ++i) lcdPtr->write(rowData[i]);
  }

  memcpy(lcdRowCache[row], rowData, LCD_COLS);
  lcdRowCacheValid[row] = true;
}

static void lcdWriteCachedTextRow(int row, const char* text, bool force = false) {
  uint8_t rowData[LCD_COLS];
  lcdBuildBlankRow(rowData);
  lcdCopyTextToRow(rowData, 0, LCD_COLS, text ? text : "");
  lcdWriteCachedRowRaw(row, rowData, force);
}

static void lcdWriteCachedCenteredRow(int row, const char* text, bool force = false) {
  uint8_t rowData[LCD_COLS];
  lcdBuildCenteredRow(rowData, text ? text : "");
  lcdWriteCachedRowRaw(row, rowData, force);
}

void lcdCreateChars() {
  if (!lcdPtr) return;
  lcdPtr->createChar(CHAR_HEART, charHeart);
  lcdPtr->createChar(CHAR_SCAN2, charScan2_data);
  lcdPtr->createChar(CHAR_BREATH, charBreath);
  lcdPtr->createChar(CHAR_DOT_FULL, charDotFull);
  lcdPtr->createChar(CHAR_DOT_EMPTY, charDotEmpty);
  lcdPtr->createChar(CHAR_DEGREE, charDegree);
  lcdPtr->createChar(CHAR_PERSON, charPerson);
  lcdPtr->createChar(CHAR_SCAN, charScan1);
  heartAnimFrame = false;
  customCharsValid = true;
  lcdPtr->setCursor(0, 0);
}

void setHeartChar(bool small) {
  if (!lcdPtr || !customCharsValid) return;
  animPending = true;
  animPendingSmall = small;
}

static void flushAnimPending() {
  if (!animPending || !lcdPtr || !customCharsValid) return;
  lcdPtr->createChar(CHAR_HEART, animPendingSmall ? charHeartSmall : charHeart);
  lcdPtr->setCursor(0, 0); animPending = false;
}

void lcdPrintPadded(const char* str, int width) {
  if (!lcdPtr) return;
  if (width < 0) width = 0;
  if (width > 20) width = 20;
  int len = (int)strlen(str); if (len > width) len = width;
  for (int i = 0; i < len; i++) lcdPtr->write(str[i]);
  for (int i = len; i < width; i++) lcdPtr->write(' ');
}

void lcdPrintCentered(int row, const char* str) {
  if (!lcdPtr) return;
  int len = (int)strlen(str);
  if (len > 20) len = 20;
  int pad = (20 - len) / 2;
  lcdPtr->setCursor(0, row);
  for (int i = 0; i < pad; i++) lcdPtr->write(' ');
  for (int i = 0; i < len; i++) lcdPtr->write(str[i]);
  for (int i = pad + len; i < 20; i++) lcdPtr->write(' ');
}

void lcdSmoothTransition() {
  if (!lcdPtr) return;
  unsigned long now = millis();
  if (now - lastTransitionMs < TRANSITION_DELAY_MS) {
    unsigned long waitMs = TRANSITION_DELAY_MS - (now - lastTransitionMs);
    unsigned long start = millis();
    while (millis() - start < waitMs) { buzzerUpdate(); wdtReset(); delay(1);
    } 
  }
  lastTransitionMs = millis();
}

void drawProgressBar(int row, int percent, bool animated) {
  if (!lcdPtr) return;
  percent = constrain(percent, 0, 100);
  lcdPtr->setCursor(0, row); lcdPtr->write('[');
  int filled = (percent * 14) / 100;
  for (int i = 0; i < 14; i++) {
    if (i < filled) lcdPtr->write((char)0xFF);
    else if (i == filled && animated && percent < 100) lcdPtr->write((millis() / 150) % 2 ? (char)0xFF : '-');
    else lcdPtr->write('-');
  }
  lcdPtr->write(']');
  char pctStr[5]; snprintf(pctStr, sizeof(pctStr), "%3d%%", percent);
  lcdPtr->print(pctStr);
}

void drawSignalDots(float quality) {
  if (!lcdPtr) return;
  int dots = 0;
  if (quality >= 0.80f) dots = 5;
  else if (quality >= 0.60f) dots = 4;
  else if (quality >= 0.40f) dots = 3;
  else if (quality >= 0.20f) dots = 2;
  else if (quality > 0.05f) dots = 1;
  for (int i = 0; i < 5; i++)
    lcdPtr->write(byte(i < dots ? CHAR_DOT_FULL : CHAR_DOT_EMPTY));
}

const char* getHRZone(float hr) {
  if (hr < 60.0f) return HR_ZONE_REST;
  if (hr < 100.0f) return HR_ZONE_NORMAL;
  if (hr < 140.0f) return HR_ZONE_ACTIVE;
  return HR_ZONE_HIGH;
}


// =========================================================================
// I2C / LCD INIT
// =========================================================================
void i2cRecover() {
  wdtReset(); Wire.end();
  delay(10); wdtReset();
  gpio_reset_pin((gpio_num_t)SCL); gpio_reset_pin((gpio_num_t)SDA);
  pinMode(SCL, OUTPUT);
  pinMode(SDA, INPUT_PULLUP);
  digitalWrite(SCL, HIGH);
  delayMicroseconds(5);
  for (int i = 0; i < 9; i++) {
    wdtReset(); buzzerUpdate();
    digitalWrite(SCL, HIGH); delayMicroseconds(5);
    digitalWrite(SCL, LOW); delayMicroseconds(5);
  }
  // Generate a STOP condition: SDA low while SCL high, then SDA high.
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);  delayMicroseconds(5);
  digitalWrite(SCL, HIGH); delayMicroseconds(5);
  digitalWrite(SDA, HIGH); delayMicroseconds(5);
  gpio_reset_pin((gpio_num_t)SCL); gpio_reset_pin((gpio_num_t)SDA);
  Wire.begin(SDA, SCL); Wire.setTimeOut(100); wdtReset();
}

void lcdReInit() {
  if (!lcdPtr) return;
  if (!probeI2C(lcdAddr)) {
    i2cRecover();
    if (!probeI2C(lcdAddr)) {
      lcdConnected = false;
      invalidateLcdRowCache();
      return;
    }
  }
  lcdPtr->init(); lcdPtr->backlight();
  invalidateLcdRowCache();
  customCharsValid = false; lcdCreateChars();
  prevDispState = DISP_NONE; lastLedColor = 0xFFFFFFFF;
}

static bool probeI2C(uint8_t addr) {
  Wire.beginTransmission(addr); return (Wire.endTransmission() == 0);
}


static bool tryInitBH1750() {
  uint8_t candidates[] = {0x5C, 0x23};
  for (uint8_t i = 0; i < sizeof(candidates); ++i) {
    uint8_t addr = candidates[i];
    if (probeI2C(addr) && lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, addr, &Wire)) {
      bh1750Addr = addr;
      bh1750Ready = true;
      return true;
    }
  }
  bh1750Ready = false;
  return false;
}

static bool i2cSafeReadLux(unsigned long now) {
  wdtReset();
  Wire.setTimeOut(50);
  float lux = lightMeter.readLightLevel();
  Wire.setTimeOut(100);
  if (isfinite(lux) && lux >= 0.0f) {
    luxLevel = lux;
    return true;
  }
  bh1750Ready = false;
  lastBh1750RetryMs = now;
  i2cRecover();
  tryInitBH1750();
  return false;
}

static bool i2cSafeReadMLX(unsigned long now, float& amb, float& obj) {
  wdtReset();
  Wire.setTimeOut(50);
  amb = mlx.readAmbientTempC();
  wdtReset();
  obj = mlx.readObjectTempC();
  Wire.setTimeOut(100);
  if (isfinite(amb) && isfinite(obj)) return true;
  mlxReady = false;
  lastMlxRetry = now;
  i2cRecover();
  mlxReady = mlx.begin();
  if (bh1750Ready) tryInitBH1750();
  if (lcdConnected) lcdReInit();
  return false;
}

bool scanForLCD() {
  Wire.setTimeOut(50);
  uint8_t candidates[] = { 0x27, 0x3F, 0x20, 0x21, 0x22, 0x24, 0x25, 0x26 };
  bool found = false;
  for (int i = 0; i < (int)(sizeof(candidates)/sizeof(candidates[0])); i++) {
    wdtReset(); uint8_t addr = candidates[i];
    if (addr == 0x23 || addr == 0x5C) continue;
    if (bh1750Ready && addr == bh1750Addr) continue;
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      if (lcdObjAllocated && lcdPtr) { lcdPtr->~LiquidCrystal_I2C();
        lcdPtr = nullptr; lcdObjAllocated = false; }
      lcdPtr = new (lcdObjBuf) LiquidCrystal_I2C(addr, 20, 4);
      lcdObjAllocated = true;
      lcdPtr->init(); lcdPtr->backlight();
      invalidateLcdRowCache();
      customCharsValid = false; lcdCreateChars();
      lcdAddr = addr; prevDispState = DISP_NONE;
      Serial.printf("[LCD] Found 0x%02X\n", addr);
      found = true; break;
    }
  }
  Wire.setTimeOut(100); return found;
}

// =========================================================================
// DSP CONSTANTS
// =========================================================================
#define BUF_SIZE 384
const float HR_MIN = 40.0f;
const float HR_MAX = 180.0f;
const float RR_MIN = 8.0f;
const float RR_MAX = 35.0f;
const float RR_BIAS_CORRECTION_BPM = -1.0f;        // v12.2: conservative empirical downward correction
const float RR_MODULE_RAW_HARMONIC_RATIO = 1.6f;   // empirical raw-RR harmonic correction candidate
const float RR_MODULE_RAW_CORRECTION_BLEND = 0.3f;  // reserved for future adaptive blend tuning
const float RR_BIAS_CORRECTION_MAX_ABS = 1.5f;      // v12.2: cap correction magnitude
const float HR_CONF_THRESHOLD = 0.18f;
const float HR_CONF_THRESHOLD_FAST = 0.15f;
const float RR_CONF_THRESHOLD = 0.20f;
const float PQI_LOCK_THRESHOLD = 0.35f; 
const float PQI_LOCK_THRESHOLD_MID_RANGE = 0.20f; 
const float PQI_LOCK_THRESHOLD_NEAR_FIELD = 0.15f; 
enum HrPathSource : uint8_t {
  HR_PATH_NONE = 0,
  HR_PATH_AUTO = 1,
  HR_PATH_SPECTRAL = 2,
  HR_PATH_RAW_BLEND = 3,
  HR_PATH_RAW_FALLBACK = 4,
  HR_PATH_RAW_ONLY_NO_PHASE = 5,
  HR_PATH_DIRECT_RAW_FAST = 6
};
enum HrConfidenceSource : uint8_t {
  HR_CONF_NONE = 0,
  HR_CONF_AUTO_PHASE = 1,
  HR_CONF_AUTO_PHASE_RAW_BLEND = 2,
  HR_CONF_SPECTRAL_ONLY = 3,
  HR_CONF_SPECTRAL_RAW_BLEND = 4,
  HR_CONF_RAW_FALLBACK = 5,
  HR_CONF_RAW_ONLY = 6,
  HR_CONF_DIRECT_RAW_FAST = 7
};
enum HrAnchorSource : uint8_t {
  HR_ANCHOR_NONE = 0,
  HR_ANCHOR_TRUSTED_PHASE = 1,
  HR_ANCHOR_TRACKING_RAW = 2,
  HR_ANCHOR_LATCHED_RAW = 3,
  HR_ANCHOR_ARBITER = 4,
  HR_ANCHOR_REJECTPHASE = 5
};
enum HarmonicModeBit : uint8_t {
  HARM_MODE_RR_RAW_HARMONIC_CORRECTED = 0x01,
  HARM_MODE_HR_ARBITER_CORRECTED      = 0x02,
  HARM_MODE_HR_RAW_LOOKED_HALF_RATE   = 0x04,
  HARM_MODE_HR_HARMONIC_AMBIGUOUS     = 0x08,
  HARM_MODE_RR_SUBHARMONIC_REJECTED   = 0x10,
  HARM_MODE_RR_HARMONIC_REJECTED      = 0x20
};
const float RR_PQI_LOG_THRESHOLD = 0.20f;
const float HR_LOG_PQI_MIN = 0.10f;
const float HR_PQI_UPDATE_FLOOR = 0.08f;   // v12.2: lower tracking floor without fully opening weak windows
const float RR_SPEC_FMIN_LOW = 0.13f;
const float RR_SPEC_FMIN_DEFAULT = 0.18f;
const float RR_SPEC_FMAX = 0.60f;
const float RR_SPEC_BLEND_TOL_BPM = 1.25f;
const float RR_SPEC_RESCUE_MAX_ERR_BPM = 1.75f;
const float RR_RAW_UPDATE_MAX_DIFF_BPM = 2.5f;
const float HR_RAW_BLEND_MAX_DIFF = 8.0f;
const float HR_SPEC_RAW_PLAUS_MAX_DIFF = 22.0f;
const float HR_HARMONIC_ALT_RATIO = 0.80f;
const float HR_HARMONIC_RAW_ARBITER_BPM = 20.0f;
const float HR_HARMONIC_HALF_PLAUS_MAX  = 62.0f;
const float HR_HARMONIC_RAW_PLAUS_MIN   = 70.0f;
const unsigned long RR_POST_MOTION_HOLDOFF_MS = 3000UL;
const unsigned long RR_DISPLAY_GRACE_MS = 6000UL; // v11.6.7: display-only grace, preserves strict CSV invariants
// v11.6.5: surgical trust / rejection gates
const float HR_PQI_WEAK_REJECT = 0.12f;
const float HR_PQI_SOFT_REJECT = 0.20f;
const float HR_RAW_DISAGREE_HARD_BPM = 10.0f;
const float HR_RAW_DISAGREE_SOFT_BPM = 18.0f;
const float RR_PQI_MIN_ACCEPT = 0.12f;
const float RR_CONF_MIN_ACCEPT = 0.25f;
const float RR_RAW_DISAGREE_BPM = 6.0f;
const float RR_SUBHARMONIC_ANCHOR_RATIO = 0.60f;
const float RR_HARMONIC_UPPER_ANCHOR_RATIO = 1.35f;
const float RR_RAW_FALLBACK_MAX_ERR_BPM = 4.0f;
const unsigned long RR_SUBHARMONIC_ANCHOR_MAX_AGE_MS = 12000UL;
const float MIN_PHASE_ENERGY = 1e-5f;   
const float PEAK_PROMINENCE_MIN = 0.05f;
const float RR_PEAK_PROMINENCE_MIN = 0.020f;   // Session 9a: relax RR only, keep HR strict
const int   RR_WIDE_LAG_SPAN = 58;             // typical RR lag span at ~10 Hz (75-17), preserves gradual relaxation
const float HR_BAND_HALF = 25.0f;
const float HR_BAND_HALF_RAW = 18.0f;
const float RR_SEED_MAX_BPM = 16.0f;
const uint8_t RR_SEED_MIN_CONSISTENT_READS = 3;
const float RR_SEED_CONSISTENT_MAX_DIFF_BPM = 2.0f;
const uint8_t RR_PERSISTENT_OUTLIER_MIN_WINDOWS = 5;
const uint8_t RR_FUNDAMENTAL_RECOVERY_MIN_WINDOWS = 3;
const float RR_FUNDAMENTAL_RECOVERY_REL_TOL = 0.12f;
const float RR_FUNDAMENTAL_RECOVERY_ABS_TOL_BPM = 1.5f;
const float RR_FUNDAMENTAL_RECOVERY_MIN_PQI = 0.22f;
const float RR_FUNDAMENTAL_RECOVERY_MIN_PQI_NEAR_FIELD = 0.15f;
const float RR_FUNDAMENTAL_RECOVERY_MIN_CONF = 0.30f;
const uint8_t RR_FUNDAMENTAL_RECOVERY_MIN_WINDOWS_NEAR_FIELD = 4;
const uint8_t FALSE_PRESENCE_ABSENCE_SCORE_MAX = 12;
const uint8_t FALSE_PRESENCE_ABSENCE_SCORE_EXIT = 8;
const float FALSE_PRESENCE_PHASE_MIN = 0.18f;
const unsigned long FALSE_PRESENCE_FORCE_EXIT_MS = 3000UL;
const unsigned long NO_HUMAN_CLEAR_STATE_MS = 5000UL;
const unsigned long PUBLISH_WARMUP_MS = 8000UL;
const unsigned long CANDIDATE_MAX_AGE_MS = 12000UL;
const unsigned long FINAL_PUBLISH_CANDIDATE_STALE_MS = 5000UL;
const int SKIPDSP_FORCED_ABSENT_THRESH = 60;
const unsigned long SKIPDSP_FORCED_ABSENT_MIN_MS = 15000UL;
const uint8_t RR_MIDSESSION_RAW_REANCHOR_MIN_CONSISTENT_READS = 3;
const float RR_MIDSESSION_RAW_REANCHOR_MAX_ANCHOR_ERR_BPM = 4.0f;
const unsigned long RR_MIDSESSION_RAW_REANCHOR_GRACE_MS = 5000UL;
const uint8_t HR_COLD_START_MAX_WINDOWS = 5;
const float HR_COLD_START_CEILING_BPM = 115.0f;
const float HR_COLD_START_CONFIRM_BPM = 110.0f;
const float HR_RAW_DISAGREE_STRONG_BPM = 10.0f;
const float HR_RAW_DISAGREE_MED_BPM = 6.0f;
static const uint8_t DIRECT_RAW_HR_ENTRY_COUNT = 8;
static const float DIRECT_RAW_HR_EXIT_DISAGREE = 12.0f;
static const unsigned long DIRECT_RAW_HR_MAX_STALENESS_MS = 3000UL;
static const float RADAR_SPEED_STEP_CM_S = 0.49f; // diagnostic speed estimate from sibling TF/SDK evidence; do not over-trust for gating
static const unsigned long TARGET_INFO_FRESH_MS = 800UL;
static const unsigned long POINT_CLOUD_FRESH_MS = 400UL;
static const int DOPPLER_SUBTLE_THRESHOLD_INDEX = 1;
static const int DOPPLER_MOTION_THRESHOLD_INDEX = 3;
static const int CLUSTER_ANOMALY_MIN_CLUSTERS = 2;
static const float MODULE_MAX_DISTANCE_CM = 600.0f;

const float MIN_GAIN = 0.2f;
const float MAX_GAIN = 5.0f;
const float TARGET_ENERGY = 0.008f;

const int CALIBRATION_FRAMES = 100;
const unsigned long CALIB_TIMEOUT_MS = 60000UL;
const int MOTION_WARMUP_FRAMES = 10;
const int WIN_FAST = 64;
const int WIN_FULL = 256;
const float RECAL_RATIO_HI = 4.0f;
const float RECAL_RATIO_LO = 0.25f;
const int RECAL_HOLD_FRAMES = 50;
const unsigned long DISPLAY_INTERVAL = 500UL;
const unsigned long MLX_RETRY_INTERVAL = 3000UL;

float kfHR_x = 75.0f, kfHR_P = 1.0f;
const float kfHR_Q = 0.18f;
float       kfHR_R = 4.0f;
float kfRR_x = 15.0f, kfRR_P = 1.0f;
const float kfRR_Q = 0.05f, kfRR_R = 2.0f;
const float HR_TRIPLE_AGREE_BPM = 4.0f;
const float RR_TRIPLE_AGREE_BPM = 1.5f;
static int motionCooldown = 0;
float hrConfidence = 0;
static int rawHR_consecutive_valid = 0;           // v11.6.7 deadlock breaker
static const int RAW_HR_SEED_MIN_COUNT = 5;       // ~500 ms at 10 Hz
static unsigned long rawHR_last_seed_ms = 0UL;    // v11.6.9 seed cooldown
static const unsigned long RAW_HR_SEED_COOLDOWN_MS = 30000UL;
static const unsigned long RAW_HR_FIRST_SEED_COOLDOWN_MS = 15000UL;
static bool sessionFirstSeedDone = false;
static const float RAW_HR_SEED_MIN_STABILITY = 0.40f;
static const float RAW_HR_SEED_MAX_DIFF_WITH_CAND = 15.0f;
static const float HR_FIRST_PHASE_SEED_MIN_BPM = 65.0f;
static const float HR_FIRST_PHASE_SEED_PQI_SUSPECT = 0.60f;
static const uint8_t HR_FIRST_PHASE_SEED_CONFIRM_CYCLES = 3;
static const float HR_FIRST_PHASE_SEED_CONFIRM_DELTA_BPM = 4.0f;
static const unsigned long AGC_SKIP_RECOVERY_MS = 8000UL;
static const float CHIP_HR_BIAS_CORRECTION_BPM = 6.0f;   // v13.7.3: unchanged from v13.7.2; only applied on blind raw-only reseed path
static const float HR_LOG_PQI_BYPASS_MIN = 0.10f;
static const float HR_LOG_CONF_BYPASS_MIN = 0.45f;
static const float HR_LOG_CAND_CONF_BYPASS_MIN = 0.08f;
static const unsigned long HR_PUBLISH_GRACE_MS = 3500UL;
static const float HR_PUBLISH_GRACE_MAX_DELTA_BPM = 8.0f;
static const float HR_PUBLISH_GRACE_MIN_PQI = 0.10f;
static const float HR_BIAS_DELTA_BPM = 8.0f;
static const uint8_t HR_DOWNWARD_OVERRIDE_WINDOWS = 2;
static const uint8_t HR_UPWARD_CONFIRM_WINDOWS = 3;
static const uint8_t HR_PUBLISH_GRACE_BLOCK_WINDOWS = 2;
static const float HR_DOWNWARD_RAW_BLEND_MULT = 0.2f;
static const float HR_DOWNWARD_COHERENCE_RESIST_MULT = 0.5f;
static unsigned long lastValidDisplayRRMs = 0UL;  // v11.6.7 display grace only
static float lastValidDisplayRR = 0.0f;
static unsigned long lastValidPublishHRMs = 0UL;
static float lastValidPublishHR = 0.0f;
static uint8_t rrHoldAgreementCount = 0;          // v11.6.7 early release after motion
static float lastLoggedHrBandMin = HR_MIN;
static float lastLoggedHrBandMax = HR_MAX;
static float lastOutlierRRCandidate = 0.0f;
static uint8_t rrOutlierPersistCount = 0;

// =========================================================================
// HR VARIABILITY
// =========================================================================
#define HR_VAR_WIN 16
static float hrVarBuf[HR_VAR_WIN] = { 0 };
static int hrVarIdx = 0;
static int hrVarCount = 0;

static float computeHRVar() {
  if (hrVarCount < 3) return 0.0f;
  float mean = 0.0f;
  for (int i = 0; i < hrVarCount; i++) mean += hrVarBuf[i];
  mean /= hrVarCount;
  float var = 0.0f;
  for (int i = 0; i < hrVarCount; i++) { float d = hrVarBuf[i] - mean;
    var += d * d; }
  return sqrtf(var / (float)(hrVarCount - 1));
}

static void pushHRVar(float hr) {
  hrVarBuf[hrVarIdx] = hr;
  hrVarIdx = (hrVarIdx + 1) % HR_VAR_WIN;
  if (hrVarCount < HR_VAR_WIN) hrVarCount++;
}

// =========================================================================
// DSP FUNCTIONS
// =========================================================================

float adaptiveQ_HR() {
  float baseQ = (hrConfidence > 0.8f) ? 0.04f : kfHR_Q;
  if (motionCooldown > 0) {
    float alpha = fminf((float)motionCooldown / 15.0f, 1.0f);
    return baseQ * (1.0f + alpha * 0.8f);
  }
  return baseQ;
}

float kalmanHR(float z) {
  float Q = adaptiveQ_HR(); kfHR_P += Q;
  float K = kfHR_P / (kfHR_P + kfHR_R);
  kfHR_x += K * (z - kfHR_x); kfHR_P *= (1.0f - K);
  return kfHR_x;
}

float adaptiveQ_RR() {
  if (motionCooldown > 0) {
    float alpha = fminf((float)motionCooldown / 15.0f, 1.0f);
    return kfRR_Q * (1.0f + alpha * 0.8f);
  }
  return kfRR_Q;
}

float kalmanRR(float z, float confScale = 1.0f) {
  float effectiveR = kfRR_R / fmaxf(confScale * confScale, 0.01f);
  kfRR_P += adaptiveQ_RR(); float K = kfRR_P / (kfRR_P + effectiveR);
  kfRR_x += K * (z - kfRR_x); kfRR_P *= (1.0f - K);
  return kfRR_x;
}

float bph_b0 = 0.3707f, bph_b1 = 0.0f, bph_b2 = -0.3707f;
float bph_a1 = -0.7414f, bph_a2 = 0.2586f;
float bph_x1 = 0, bph_x2 = 0, bph_y1 = 0, bph_y2 = 0;

float bpHeartProcess(float x) {
  float y = bph_b0*x + bph_b1*bph_x1 + bph_b2*bph_x2 - bph_a1*bph_y1 - bph_a2*bph_y2;
  bph_x2 = bph_x1;
  bph_x1 = x; bph_y2 = bph_y1; bph_y1 = y; return y;
}

float bpb_b0 = 0.0676f, bpb_b1 = 0.0f, bpb_b2 = -0.0676f;
float bpb_a1 = -1.8210f, bpb_a2 = 0.8647f;
float bpb_x1 = 0, bpb_x2 = 0, bpb_y1 = 0, bpb_y2 = 0;

float bpBreathProcess(float x) {
  float y = bpb_b0*x + bpb_b1*bpb_x1 + bpb_b2*bpb_x2 - bpb_a1*bpb_y1 - bpb_a2*bpb_y2;
  bpb_x2 = bpb_x1;
  bpb_x1 = x; bpb_y2 = bpb_y1; bpb_y1 = y; return y;
}

void bpResetHeart() { bph_x1=0; bph_x2=0; bph_y1=0; bph_y2=0; }
void bpResetBreath() { bpb_x1=0; bpb_x2=0; bpb_y1=0; bpb_y2=0; }

#define SG_WIN 5
float sgBufH[SG_WIN]={0}, sgBufB[SG_WIN]={0};
int sgIdx=0, sgWarmup=0;

float savitzkyGolay(float* buf) {
  int n = (sgIdx - 1 + SG_WIN) % SG_WIN;
  return (-3.0f*buf[(n-4+SG_WIN)%SG_WIN] + 12.0f*buf[(n-3+SG_WIN)%SG_WIN]
          + 17.0f*buf[(n-2+SG_WIN)%SG_WIN] + 12.0f*buf[(n-1+SG_WIN)%SG_WIN]
          - 3.0f*buf[n]) / 35.0f;
}

float heartBuf[BUF_SIZE], breathBuf[BUF_SIZE], fusedBuf[BUF_SIZE];
unsigned long phaseTimes[BUF_SIZE];
static float linearHeart_snap[BUF_SIZE];
static float linearBreath_snap[BUF_SIZE];
static float linearFused_snap[BUF_SIZE];
static int bufCount_snap = 0;
static float fs_snap = 10.0f;
float pqiBreath = 0;

int bufIndex = 0, bufCount = 0;
float prevHeartPhase=0, prevBreathPhase=0;
float heartOffset=0, breathOffset=0;
float clutterHeart=0, clutterBreath=0;
float prevHeartDiff=0, prevBreathDiff=0;
static bool firstPhaseSample = true;
float prevStableHeartPhase=0, prevStableBreathPhase=0;
float radarDistance = 0; // in cm
static float lastGoodDistance = 0.0f; // in cm

float stabilizePhase(float phase, float& prevStable) {
  float effectiveDistance = (radarDistance > 1.0f) ? radarDistance :
                            ((lastGoodDistance > 1.0f) ? lastGoodDistance : 100.0f);
  float threshold = 0.5f + fmaxf(0.0f, effectiveDistance - 50.0f) * 0.003f;
  float diff = phase - prevStable;
  if (fabsf(diff) > threshold) phase = prevStable + diff * 0.2f;
  prevStable = phase;
  return phase;
}

#define MED_WIN 5
float medBufH[MED_WIN]={0}, medBufB[MED_WIN]={0};
int medIdx = 0;

float medianOf5(float* a) {
  float s[MED_WIN];
  for (int i=0;i<MED_WIN;i++) s[i]=a[i];
  for (int i=1;i<MED_WIN;i++) {
    float key=s[i]; int j=i-1;
    while (j>=0 && s[j]>key) { s[j+1]=s[j]; j--; }
    s[j+1]=key;
  }
  return s[MED_WIN/2];
}

#define RLS_TAPS 16
#define RLS_LAMBDA_HR 0.90f
#define RLS_DELTA 1000.0f

static float rlsW[RLS_TAPS]={0};
static float rlsX[RLS_TAPS]={0};
static float rlsP[RLS_TAPS][RLS_TAPS];
static bool rlsInitialised = false;
static bool rlsWarmedUp = false;
static int rlsFrameCount = 0;
// v13.7 P2/P3: Warmup instrumentation and Clutter experiment
static int   clutterWarmupCount = 0;
static bool  phaseWarmupComplete = false;
static float currentClutterAlpha = 0.005f;

void rlsReset() {
  memset(rlsW,0,sizeof(rlsW));
  memset(rlsX,0,sizeof(rlsX));
  for (int i=0;i<RLS_TAPS;i++)
    for (int j=0;j<RLS_TAPS;j++)
      rlsP[i][j] = (i==j) ? RLS_DELTA : 0.0f;
  rlsInitialised=true; rlsWarmedUp=false; rlsFrameCount=0;
}

float rlsUpdate(float ref, float primary, bool freezeWeights) {
  if (!rlsInitialised) rlsReset();
  for (int i=RLS_TAPS-1;i>0;i--) rlsX[i]=rlsX[i-1];
  rlsX[0]=ref;
  float yhat=0.0f;
  for (int i=0;i<RLS_TAPS;i++) yhat += rlsW[i]*rlsX[i];
  float e = primary - yhat;
  if (!isfinite(e)) { rlsReset(); return primary; }
  if (!freezeWeights) {
    float Px[RLS_TAPS]={0};
    for (int i=0;i<RLS_TAPS;i++)
      for (int j=0;j<RLS_TAPS;j++)
        Px[i] += rlsP[i][j]*rlsX[j];

    float denom=RLS_LAMBDA_HR;
    for (int i=0;i<RLS_TAPS;i++) denom += rlsX[i]*Px[i];
    if (!isfinite(denom) || denom <= 1e-6f) { rlsReset(); return primary; }

    for (int i=0;i<RLS_TAPS;i++) {
      float ki=Px[i]/denom;
      rlsW[i] += ki*e;
      for (int j=0;j<RLS_TAPS;j++)
        rlsP[i][j] = (rlsP[i][j] - ki*Px[j]) / RLS_LAMBDA_HR;
    }
    float maxP = 0.0f;
    for (int i = 0; i < RLS_TAPS; i++)
        if (fabsf(rlsP[i][i]) > maxP) maxP = fabsf(rlsP[i][i]);

    if (maxP > 1e4f || !isfinite(maxP)) {
        rlsReset();
        return primary;
    }
    if (++rlsFrameCount >= RLS_TAPS*2) rlsWarmedUp=true;
  }
  if (!rlsWarmedUp) return primary;
  return e;
}

// =========================================================================
// FRESHNESS HELPERS [v11.0.0]
// =========================================================================
bool isHRFresh() { return lastHRUpdateMs>0 && (millis()-lastHRUpdateMs<HR_STALE_MS); }
bool isRRFresh() { return lastRRUpdateMs>0 && (millis()-lastRRUpdateMs<RR_STALE_MS); }
bool isDistanceFresh() { return lastDistanceUpdateMs>0 && (millis()-lastDistanceUpdateMs<DISTANCE_STALE_MS); }
bool isPresenceFresh() { return lastPresenceUpdateMs>0 && (millis()-lastPresenceUpdateMs<PRESENCE_STALE_MS); }

bool isPhaseFresh() {
  return lastPhaseDataMs > 0 && (millis() - lastPhaseDataMs < PHASE_FRESH_REQ_MS);
}

bool isTrustedVitalFresh() {
  return lastTrustedVitalMs > 0 && (millis() - lastTrustedVitalMs < LOG_VITAL_GRACE_MS);
}
bool isTrustedHRFresh() {
  return lastTrustedHRMs > 0 && (millis() - lastTrustedHRMs < LOG_VITAL_GRACE_MS);
}
bool isTrustedRRFresh() {
  return lastTrustedRRMs > 0 && (millis() - lastTrustedRRMs < LOG_VITAL_GRACE_MS);
}


static float lastCalculatedFs = 10.0f;
float motionLP = 0;
int motionWarmup = 0;
static uint8_t motionSpikeCount = 0;

bool motionDetected(float dh, float db) {
  float energy = dh*dh + db*db;
  motionLP = 0.95f*motionLP + 0.05f*energy;
  if (motionWarmup < MOTION_WARMUP_FRAMES) { motionWarmup++; motionSpikeCount = 0; return false; }
  bool spike = (energy > fmaxf(motionLP*8.0f, 1e-6f));
  if (spike) {
    if (motionSpikeCount < 255) motionSpikeCount++;
  } else {
    motionSpikeCount = 0;
  }
  return motionSpikeCount >= 2;
}

float computePQI(float* buf, int n, float fs, float minBPM, float maxBPM) {
  if (n<=0 || fs<=0.0f || minBPM<=0.0f || maxBPM<=0.0f) return 0.0f;
  float energy=0;
  for (int i=0;i<n;i++) {
    wdtResetEvery(i, 64);
    energy += buf[i]*buf[i];
  }
  if (energy<=1e-6f) return 0.0f;
  float r0 = autocorr(buf, n, 0);
  if (r0 <= 1e-6f || !isfinite(r0)) return 0.0f;

  int minLag = max(2, (int)(fs*60.0f/maxBPM));
  int maxLag = min(n/2, (int)(fs*60.0f/minBPM));
  float best=0;

  for (int lag=minLag; lag<=maxLag; lag++) {
    wdtResetEvery(lag - minLag, 8);
    int cnt=n-lag; float ac=0;
    for (int i=0;i<cnt;i++) {
      wdtResetEvery(i, 64);
      ac += buf[i]*buf[i+lag];
    }
    ac /= (float)cnt;
    if (ac>best) best=ac;
  }
  return constrain(best/(energy/n), 0.0f, 1.0f);
}

#define LUT_SIZE 512
static float sinLUT[LUT_SIZE];
void buildSinLUT() { for (int i=0;i<LUT_SIZE;i++) sinLUT[i]=sinf(2.0f*PI*i/LUT_SIZE); }

float lut_sin(float ph) {
  if (!isfinite(ph)) return 0.0f;
  ph=fmodf(ph,2.0f*PI); if (ph<0.0f) ph+=2.0f*PI;
  int idx=(int)(ph*(LUT_SIZE/(2.0f*PI)))%LUT_SIZE;
  if (idx<0||idx>=LUT_SIZE) idx=0;
  return sinLUT[idx];
}

float goertzel(float* buf, int n, float freq, float fs) {
  if (n <= 0 || fs <= 0.0f || !isfinite(freq)) return 0.0f;
  float coeff=2.0f*cosf(2.0f*PI*freq/fs);
  float s0=0,s1=0,s2=0;
  for (int k=0;k<n;k++) {
    wdtResetEvery(k, 32);
    if (!isfinite(buf[k])) return 0.0f;
    s0=buf[k]+coeff*s1-s2;
    s2=s1; s1=s0;
  }
  float power = s1*s1+s2*s2-coeff*s1*s2;
  if (!isfinite(power)) return 0.0f;
  return fmaxf(0.0f, power);
}

float autocorr(float* buf, int n, int lag) {
  if (lag<0||lag>=n) return 0.0f;
  int cnt=n-lag; if (cnt<=0) return 0.0f;
  float sum=0;
  for (int i=0;i<cnt;i++) {
    wdtResetEvery(i, 64);
    sum += buf[i]*buf[i+lag];
  }
  return sum/(float)cnt;
}

float parabolicPeak(float v1, float v2, float v3) {
  float denom=v1-2.0f*v2+v3;
  if (fabsf(denom)<1e-10f) return 0.0f;
  return 0.5f*(v1-v3)/denom;
}

float harmonicSpectrumScore(float* buf, int n, float fs, float bpm) {
  float f=bpm/60.0f, score=0;
  float maxFreq = fs * 0.45f; 
  for (int h=1;h<=3;h++) {
    if (f * h > maxFreq) break;
    float mag=sqrtf(goertzel(buf,n,f*h,fs));
    if (!isfinite(mag)) mag = 0.0f;
    if (h==1) score+=mag; else if (h==2) score+=0.5f*mag; else if (h==3) score+=0.25f*mag;
  }
  return score;
}

static inline float detectRateProminenceThreshold(float minRate, float maxRate, int minLag, int maxLag) {
  // Session 9a: real breath_phase windows can fall below a fixed 0.05 prominence
  // even when motion is absent and the signal is otherwise plausible. Keep HR strict;
  // relax only RR, and only when the lag search is wide enough that neighbouring
  // respiration lags become naturally similar.
  bool rrBand = (maxRate <= (RR_MAX + 0.5f));
  if (!rrBand) return PEAK_PROMINENCE_MIN;

  int lagSpan = maxLag - minLag;
  float relax = constrain((float)lagSpan / (float)RR_WIDE_LAG_SPAN, 0.0f, 1.0f);
  return PEAK_PROMINENCE_MIN - (PEAK_PROMINENCE_MIN - RR_PEAK_PROMINENCE_MIN) * relax;
}

bool detectRate(float* buf, int n, float fs,
                float minRate, float maxRate,
                float& rateOut, float& confOut, float confThreshold) {
  int minLag=max(1,(int)(fs*60.0f/maxRate));
  int maxLag=min(n-1,(int)(fs*60.0f/minRate));
  if (minLag>=maxLag) return false;

  float r0 = autocorr(buf, n, 0);
  if (r0 <= 1e-6f) return false;

  float best=-1e9f, secondBest=-1e9f; int bestLag=minLag;
  for (int lag = minLag; lag <= maxLag; lag++) {
    float v = autocorr(buf, n, lag) / r0;
    if (v > best) {
      best = v;
      bestLag = lag;
    }
  }

  int exclusionLags = ((maxLag - minLag) >= 6) ? 2 : 0;
  for (int lag = minLag; lag <= maxLag; lag++) {
    if (abs(lag - bestLag) <= exclusionLags) continue;
    float v = autocorr(buf, n, lag) / r0;
    if (v > secondBest) secondBest = v;
  }
  if (secondBest <= -1e8f) secondBest = best;

  float prominenceThreshold = detectRateProminenceThreshold(minRate, maxRate, minLag, maxLag);
  if ((best - secondBest) < prominenceThreshold) return false; 

  int subLag = bestLag * 2;
  if (subLag < maxLag) {
    float subVal = autocorr(buf, n, subLag) / r0;
    float subRate = fs * 60.0f / (float)subLag;
    if (subVal >= best * 0.40f && subRate >= minRate && subRate <= maxRate) {
      bestLag = subLag;
      best = subVal;
    }
  }

  confOut=constrain(best,0.0f,1.0f);
  if (confOut<confThreshold) return false;

  float period;
  if (bestLag>minLag && bestLag<maxLag) {
    float v1 = autocorr(buf, n, bestLag - 1) / r0;
    float v3 = autocorr(buf, n, bestLag + 1) / r0;
    float delta = constrain(parabolicPeak(v1, best, v3), -0.5f, 0.5f);
    period=(bestLag+delta)/fs;
  } else { period=(float)bestLag/fs; }
  if (period<=0.0f) return false;
  rateOut=constrain(60.0f/period,minRate,maxRate);
  return true;
}

void detectSpectral(float* buf, int n, float fs, float fmin, float fmax, float& outFreq, float& outMag) {
  int bins=max(20,min(120,(int)((fmax-fmin)*(float)n/fs)+2));
  float bestMag=0,bestFreq=fmin;

  for (int i=0;i<bins;i++) {
    wdtResetEvery(i, 8);
    float freq=fmin+i*(fmax-fmin)/bins;
    float mag=sqrtf(goertzel(buf,n,freq,fs));
    if (!isfinite(mag)) mag = 0.0f;
    if (mag>bestMag) { bestMag=mag; bestFreq=freq; }
  }
  outFreq = bestFreq*60.0f;
  outMag = bestMag;
}

bool detectLowestSpectralFundamental(float* buf, int n, float fs, float fmin, float fmax, float& outBpm, float& outConf, float anchorBpm) {
  if (n < 32 || fs <= 0.0f || fmax <= fmin) return false;
  const int bins = 96;
  float mags[bins];
  float step = (fmax - fmin) / (float)(bins - 1);
  float meanMag = 0.0f;

  for (int i = 0; i < bins; i++) {
    wdtResetEvery(i, 8);
    float freq = fmin + step * (float)i;
    mags[i] = sqrtf(goertzel(buf, n, freq, fs));
    meanMag += mags[i];
  }
  meanMag /= (float)bins;
  float noiseFloor = fmaxf(meanMag, 1e-4f);
  float peakFloor = fmaxf(noiseFloor * 1.35f, 0.0025f);

  bool useAnchor = (anchorBpm >= RR_MIN && anchorBpm <= RR_MAX);
  int chosenIdx = -1;
  float chosenMag = 0.0f;
  float chosenScore = -1e9f;

  for (int i = 1; i < bins - 1; i++) {
    wdtResetEvery(i, 8);
    bool localPeak = (mags[i] >= mags[i-1]) && (mags[i] > mags[i+1]);
    if (!localPeak) continue;
    if (mags[i] < peakFloor) continue;

    if (!useAnchor) {
      chosenIdx = i;
      chosenMag = mags[i];
      break;  // lowest-frequency plausible peak wins when no anchor exists
    }

    float candidateFreq = fmin + step * (float)i;
    float candidateBpm = candidateFreq * 60.0f;
    float closeness = 1.0f - constrain(fabsf(candidateBpm - anchorBpm) / 6.0f, 0.0f, 1.0f);
    float lowBias = 1.0f - constrain((candidateBpm - RR_MIN) / fmaxf(RR_MAX - RR_MIN, 1.0f), 0.0f, 1.0f);
    float magNorm = constrain((mags[i] - peakFloor) / fmaxf(peakFloor, 1e-6f), 0.0f, 2.0f) * 0.5f;
    float score = 0.45f * closeness + 0.40f * lowBias + 0.15f * magNorm;

    if (score > chosenScore) {
      chosenScore = score;
      chosenIdx = i;
      chosenMag = mags[i];
    }
  }

  if (chosenIdx < 0) return false;

  float chosenFreq = fmin + step * (float)chosenIdx;

  // If we landed on a harmonic, prefer a lower plausible sub-harmonic if it is visible.
  for (int div = 3; div >= 2; --div) {
    wdtReset();
    float subFreq = chosenFreq / (float)div;
    if (subFreq < fmin) continue;
    int subCenter = (int)lroundf((subFreq - fmin) / step);
    float subBestMag = 0.0f;
    int subBestIdx = -1;
    for (int j = max(1, subCenter - 2); j <= min(bins - 2, subCenter + 2); ++j) {
      wdtResetEvery(j, 4);
      bool localPeak = (mags[j] > mags[j-1]) && (mags[j] > mags[j+1]);
      if (!localPeak) continue;
      if (mags[j] > subBestMag) {
        subBestMag = mags[j];
        subBestIdx = j;
      }
    }
    if (subBestIdx >= 0 &&
        subBestMag >= noiseFloor * 1.15f &&
        subBestMag >= chosenMag * 0.35f) {
      chosenIdx = subBestIdx;
      chosenMag = subBestMag;
      chosenFreq = fmin + step * (float)chosenIdx;
    }
  }

  outBpm = constrain(chosenFreq * 60.0f, RR_MIN, RR_MAX);
  outConf = constrain((chosenMag - peakFloor) / fmaxf(chosenMag, 1e-6f), 0.0f, 1.0f);
  return true;
}

static inline float heartPQIWeight(float pqi) {
  pqi = constrain(pqi, 0.0f, 1.0f);
  return pqi * sqrtf(fmaxf(pqi, 0.0f)); // pqi^1.5, much more punitive in low-quality windows
}

static inline float currentHeartPQIGate() {
  float effectiveDistance = (radarDistance > 1.0f) ? radarDistance : lastGoodDistance;
  if (effectiveDistance > 1.0f && effectiveDistance <= 40.0f) return PQI_LOCK_THRESHOLD_NEAR_FIELD;
  if (effectiveDistance > 40.0f && effectiveDistance <= 80.0f) return PQI_LOCK_THRESHOLD_MID_RANGE;
  return PQI_LOCK_THRESHOLD;
}

static inline float currentRRSpectralFMin(float rrAnchorBpm) {
  return (rrAnchorBpm >= RR_MIN && rrAnchorBpm <= 17.0f) ? RR_SPEC_FMIN_LOW : RR_SPEC_FMIN_DEFAULT;
}

static bool detectRateZeroCrossing(float* buf, int n, float fs,
                                   float minRate, float maxRate,
                                   float& rateOut, float& confOut) {
  rateOut = 0.0f;
  confOut = 0.0f;
  if (!buf || n < 32 || fs <= 0.0f || maxRate <= minRate) return false;

  float mean = 0.0f;
  float maxAbs = 0.0f;
  for (int i = 0; i < n; ++i) {
    wdtResetEvery(i, 64);
    mean += buf[i];
    maxAbs = fmaxf(maxAbs, fabsf(buf[i]));
  }
  mean /= (float)n;

  float var = 0.0f;
  for (int i = 0; i < n; ++i) {
    wdtResetEvery(i, 64);
    float d = buf[i] - mean;
    var += d * d;
  }
  float stdv = sqrtf(fmaxf(var / (float)n, 0.0f));
  float eps = fmaxf(1e-5f, fmaxf(0.08f * maxAbs, 0.20f * stdv));

  float crossings[BUF_SIZE];
  int crossingCount = 0;
  float prev = buf[0] - mean;
  int prevSign = (prev > eps) ? 1 : ((prev < -eps) ? -1 : 0);

  for (int i = 1; i < n && crossingCount < BUF_SIZE; ++i) {
    wdtResetEvery(i, 64);
    float cur = buf[i] - mean;
    int curSign = (cur > eps) ? 1 : ((cur < -eps) ? -1 : 0);

    if (prevSign != 0 && curSign != 0 && curSign != prevSign) {
      float denom = fabsf(prev) + fabsf(cur);
      float frac = (denom > 1e-9f) ? (fabsf(prev) / denom) : 0.5f;
      crossings[crossingCount++] = (float)(i - 1) + frac;
      prevSign = curSign;
    } else if (curSign != 0) {
      prevSign = curSign;
    }
    prev = cur;
  }

  if (crossingCount < 4) return false;
  float spanSamples = crossings[crossingCount - 1] - crossings[0];
  if (!(spanSamples > 1.0f)) return false;

  float durationS = spanSamples / fs;
  if (!(durationS > 0.25f)) return false;

  float halfCycles = (float)(crossingCount - 1);
  float bpm = ((halfCycles * 0.5f) * 60.0f) / durationS;
  if (!isfinite(bpm) || bpm < minRate || bpm > maxRate) return false;

  float meanDt = 0.0f, varDt = 0.0f;
  int intervalCount = 0;
  for (int i = 1; i < crossingCount; ++i) {
    float dt = crossings[i] - crossings[i - 1];
    if (dt > 0.0f) {
      meanDt += dt;
      intervalCount++;
    }
  }
  if (intervalCount <= 0) return false;
  meanDt /= (float)intervalCount;
  for (int i = 1; i < crossingCount; ++i) {
    float dt = crossings[i] - crossings[i - 1];
    if (dt > 0.0f) {
      float e = dt - meanDt;
      varDt += e * e;
    }
  }
  float stdDt = sqrtf(fmaxf(varDt / (float)intervalCount, 0.0f));
  float cv = (meanDt > 1e-6f) ? (stdDt / meanDt) : 1.0f;
  float regularity = constrain(1.0f - cv, 0.0f, 1.0f);
  float density = constrain((float)crossingCount / 8.0f, 0.0f, 1.0f);

  rateOut = bpm;
  confOut = constrain(0.65f * regularity + 0.35f * density, 0.0f, 1.0f);
  return true;
}

static const uint8_t SESSION_PHASE_ABSENT      = 0;
static const uint8_t SESSION_PHASE_WARMUP      = 1;
static const uint8_t SESSION_PHASE_SETTLING    = 2;
static const uint8_t SESSION_PHASE_LOCKED      = 3;
static const uint8_t SESSION_PHASE_POST_MOTION = 4;
static const uint8_t SESSION_PHASE_LEAVING     = 5;


// =========================================================================
// STATE VARIABLES
// =========================================================================
float radarGain=1.0f, baselineEnergy=0, rollingEnergy=1e-5f; 
bool calibrationDone=false;
int calibrationCount=0, recalFrames=0;
unsigned long calibStartMs=0;
float smoothHR=70.0f, smoothRR=15.0f, smoothTemp=NAN;
int hrState=0, prevHrState=0;
float pqiHeart=0, lastStableHR=0;
int rejectionCount=0;
static int dspTask=0;
static bool lastAutoValid=false;
static float lastHrAuto=0, lastConfHR=0, lastRRDSP=0, lastConfRR=0;
static bool lastRRValid=false, wasMotion=false;
// v11 keeps these as backward-compatible debug fields in the CSV schema.
// They no longer drive presence logic directly.
int presentVotes=0, absentVotes=0;
bool humanDetected=false;
float rawHR=0, rawRR=0;

struct SessionStats {
  bool active;
  unsigned long startMs;
  unsigned long endMs;
  unsigned long lastUpdateMs;
  unsigned long motionMs;
  unsigned long lockMs;
  double hrSum;
  unsigned long hrTimeMs;
  int hrCount;
  double rrSum;
  unsigned long rrTimeMs;
  int rrCount;
  double tempSum;
  unsigned long tempTimeMs;
  int tempCount;
  float hrMin;
  float hrMax;
};
static SessionStats sessionStats = {};
static bool rawHRValid=false;
static bool rawRRValid=false;
static float rawRREffective=0.0f;
static bool  rawRRLikelyHarmonic=false;
static float rrSeedLastRaw = NAN;
static uint8_t rrSeedConsistentCount = 0;
static uint8_t hrColdStartWindowsRemaining = 0;
static unsigned long lastPersonDetectedEventMs = 0;
static bool hrHarmonicAmbiguous = false;
static bool hrRawAgreementGood = false;
static float lastHRAgreementErr = NAN;
static float hrZcBpmLogged = 0.0f;
static float hrZcConfLogged = 0.0f;
static float hrSpecBpmLogged = 0.0f;
static float hrSpecMagLogged = 0.0f;
static bool  hrTripleAgreeLogged = false;
static float rrZcBpmLogged = 0.0f;
static float rrZcConfLogged = 0.0f;
static float rrSpecBpmLogged = 0.0f;
static float rrSpecConfLogged = 0.0f;
static bool  rrTripleAgreeLogged = false;

// Diagnostic candidate telemetry (pre-gate values and reject reasons)
static float candidateHR = 0.0f;
static float candidateRR = 0.0f;
static float candidateHRConf = 0.0f;
static float candidateRRConf = 0.0f;
static float hrGatePqiUsed = 0.0f;
static float rrGatePqiUsed = 0.0f;
static uint8_t hrGateReason = 0;
static uint8_t rrGateReason = 0;
static uint8_t hrPublishReason = 0;
static uint8_t rrPublishReason = 0;
static unsigned long lastCandidateHRMs = 0;
static unsigned long lastCandidateRRMs = 0;
static float pqiHeartAtCandidateTime = 0.0f;
static unsigned long hrAgeMsLogged = 0;
static unsigned long candidateHrAgeMsLogged = 0;
static unsigned long candidateRrAgeMsLogged = 0;
static bool hrUpdatedThisCycle = false;
static uint8_t hrUpdateSourceThisCycle = HR_PATH_NONE; // 0=none,1=auto,2=spectral,3=raw_blend,4=raw_fallback,5=raw_only_no_phase,6=direct_raw_fast
static uint8_t hrConfidenceSourceThisCycle = HR_CONF_NONE;
static bool dspRanThisFrame = false;
static uint8_t hrPathSourceLatched = HR_PATH_NONE;
static uint8_t hrConfidenceSourceLatched = HR_CONF_NONE;
static bool rrPhaseBackedUpdateThisCycle = false;
static uint8_t hrFirstPhaseSeedConfirmCount = 0;
static float hrFirstPhaseSeedConfirmLastBpm = 0.0f;
static unsigned long agcFloorRecoveryStartMs = 0UL;
static unsigned int phaseValidRunLen = 0;
static unsigned int phaseInvalidRunLen = 0;
static unsigned int hrPhaseBackedUpdateCount = 0;
static unsigned int rrPhaseBackedUpdateCount = 0;
static bool nearFieldReflectorSuspect = false;
static bool agcFloorSuspect = false;
static bool phaseBackedPublishReady = false;
static bool hrAnchorDriftSuspect = false;
static unsigned int phaseGapFillCount = 0;
static unsigned int clutterRewarmCount = 0;
static bool experimentalProfileEnabled = false;

// =========================================================================
// v11.6.14 FUNNEL TELEMETRY STATE
// =========================================================================
static bool    hrArbiterCorrectedThisWindow    = false;
static bool    hrRejectPhaseRejectedThisWindow = false;
static bool    hrCoherenceRejectedThisWindow   = false;
static uint8_t hrRawSourceThisWindow           = 0; // 0=none,1=fresh,2=latch
static float   hrPreRejectPhase                = 0.0f;
static float   hrPostRejectPhase               = 0.0f;
static float   hrPostBlend                     = 0.0f;
static float   hrPostCoherence                 = 0.0f;
static float   hrFinalPublishCandidate         = 0.0f;
static bool    hrArbiterAnchorUsed             = false;
static float   hrArbiterAnchorValue            = 0.0f;
static uint8_t hrRejectPhaseAnchorUsed         = 0; // 0=none,1=raw,2=latch,3=arbiter,4=smooth,5=stable
static float   hrRejectPhaseAnchorValue        = 0.0f;
static bool    hrRawLooksLikeHalfRateLogged    = false;
static float   hrTrustedAnchorValue            = 0.0f;
static float   hrTrustedPhaseAnchorLogged      = NAN;
static uint8_t hrAnchorSourceLogged            = HR_ANCHOR_NONE;
static float   hrAnchorErrBpmLogged            = NAN;
static bool    hrRawHighBiasSuspectLogged      = false;
static bool    rrAnchorFreshLogged             = false;
static float   rrPreAcceptPhase                = 0.0f;
static float   rrPostAcceptPhase               = 0.0f;
static float   rrPostBlend                     = 0.0f;
static float   rrPostBiasCorrection            = 0.0f;
static float   rrPostKalman                    = 0.0f;
static float   rrFinalPublishCandidate         = 0.0f;
static float   rrAnchorValueLogged             = 0.0f;
static unsigned long rrAnchorAgeMsLogged       = 0UL;
static uint8_t rrAnchorSourceLogged            = 0; // 0=none,1=kf,2=smooth,3=display
static float   rrAnchorConfidenceLogged        = 0.0f;
static uint8_t rrFundamentalRecoveryCountLogged= 0;
static bool    rrFundamentalRecoveryTriggeredLogged = false;
static uint8_t rrRawSeedConsistentCountLogged  = 0;
static bool    rrMidSessionRawReanchorAllowedLogged = false;
static bool    rrMidSessionRawReanchorBlockedLogged = false;
static uint8_t rrMidSessionRawReanchorReasonLogged  = 0; // 0=none,1=consistency,2=anchor_err,3=candidate_err
static float   rrRawAnchorErrBpmLogged         = -1.0f;
static bool    hrBypassPqiOk                   = false;
static bool    hrBypassConfOk                  = false;
static bool    hrBypassGateOk                  = false;
static bool    hrBypassActive                  = false;
static bool    hrGraceEligible                 = false;
static bool    hrGraceActive                   = false;

enum HrGateReason : uint8_t {
  HR_GATE_OK = 0,
  HR_GATE_NO_AUTO = 1,
  HR_GATE_PQI = 2,
  HR_GATE_HARMONIC = 3,
  HR_GATE_RAW_DISAGREE = 4,
  HR_GATE_SPEC_REJECT = 5,
  HR_GATE_MOTION = 6,
  HR_GATE_DISPLAY_ONLY = 7,
  HR_GATE_RAW_FALLBACK = 8,
  HR_GATE_UPWARD_CONFIRM = 9
};

enum RrGateReason : uint8_t {
  RR_GATE_OK = 0,
  RR_GATE_NO_AUTO = 1,
  RR_GATE_PQI = 2,
  RR_GATE_CONF = 3,
  RR_GATE_RAW_DISAGREE = 4,
  RR_GATE_SUBHARMONIC = 5,
  RR_GATE_HARMONIC = 6,
  RR_GATE_MOTION = 7,
  RR_GATE_HOLDOFF = 8,
  RR_GATE_DISPLAY_ONLY = 9
};

enum HrPublishReason : uint8_t {
  HR_PUB_OK = 0,
  HR_PUB_NO_HUMAN = 1,
  HR_PUB_TRUST_STALE = 2,
  HR_PUB_NO_EVIDENCE = 3,
  HR_PUB_STATE = 4,
  HR_PUB_AGE = 5,
  HR_PUB_STALE = 6,
  HR_PUB_PQI = 7,
  HR_PUB_CONF = 8,
  HR_PUB_HARMONIC = 9,
  HR_PUB_RAW_DISAGREE = 10,
  HR_PUB_OTHER = 11,
  HR_PUB_GRACE_BLOCKED = 12
};

enum RrPublishReason : uint8_t {
  RR_PUB_OK = 0,
  RR_PUB_NO_HUMAN = 1,
  RR_PUB_TRUST_STALE = 2,
  RR_PUB_NO_EVIDENCE = 3,
  RR_PUB_STALE = 4,
  RR_PUB_PQI = 5,
  RR_PUB_HOLDOFF = 6,
  RR_PUB_SOURCE = 7,
  RR_PUB_OTHER = 8
};

static const char* hrGateReasonName(uint8_t r) {
  switch (r) {
    case HR_GATE_OK: return "OK";
    case HR_GATE_NO_AUTO: return "NO_AUTO";
    case HR_GATE_PQI: return "PQI";
    case HR_GATE_HARMONIC: return "HARMONIC";
    case HR_GATE_RAW_DISAGREE: return "RAW_DISAGREE";
    case HR_GATE_SPEC_REJECT: return "SPEC_REJECT";
    case HR_GATE_MOTION: return "MOTION";
    case HR_GATE_DISPLAY_ONLY: return "DISPLAY_ONLY";
    case HR_GATE_RAW_FALLBACK: return "RAW_FALLBACK";
    case HR_GATE_UPWARD_CONFIRM: return "UPWARD_CONFIRM";
    default: return "UNKNOWN";
  }
}

static const char* rrGateReasonName(uint8_t r) {
  switch (r) {
    case RR_GATE_OK: return "OK";
    case RR_GATE_NO_AUTO: return "NO_AUTO";
    case RR_GATE_PQI: return "PQI";
    case RR_GATE_CONF: return "CONF";
    case RR_GATE_RAW_DISAGREE: return "RAW_DISAGREE";
    case RR_GATE_SUBHARMONIC: return "SUBHARMONIC";
    case RR_GATE_HARMONIC: return "HARMONIC";
    case RR_GATE_MOTION: return "MOTION";
    case RR_GATE_HOLDOFF: return "HOLDOFF";
    case RR_GATE_DISPLAY_ONLY: return "DISPLAY_ONLY";
    default: return "UNKNOWN";
  }
}

static const char* hrPublishReasonName(uint8_t r) {
  switch (r) {
    case HR_PUB_OK: return "OK";
    case HR_PUB_NO_HUMAN: return "NO_HUMAN";
    case HR_PUB_TRUST_STALE: return "TRUST_STALE";
    case HR_PUB_NO_EVIDENCE: return "NO_EVIDENCE";
    case HR_PUB_STATE: return "STATE";
    case HR_PUB_AGE: return "AGE";
    case HR_PUB_STALE: return "STALE";
    case HR_PUB_PQI: return "PQI";
    case HR_PUB_CONF: return "CONF";
    case HR_PUB_HARMONIC: return "HARMONIC";
    case HR_PUB_RAW_DISAGREE: return "RAW_DISAGREE";
    case HR_PUB_OTHER: return "OTHER";
    case HR_PUB_GRACE_BLOCKED: return "GRACE_BLOCKED";
    default: return "UNKNOWN";
  }
}

static const char* rrPublishReasonName(uint8_t r) {
  switch (r) {
    case RR_PUB_OK: return "OK";
    case RR_PUB_NO_HUMAN: return "NO_HUMAN";
    case RR_PUB_TRUST_STALE: return "TRUST_STALE";
    case RR_PUB_NO_EVIDENCE: return "NO_EVIDENCE";
    case RR_PUB_STALE: return "STALE";
    case RR_PUB_PQI: return "PQI";
    case RR_PUB_HOLDOFF: return "HOLDOFF";
    case RR_PUB_SOURCE: return "SOURCE";
    case RR_PUB_OTHER: return "OTHER";
    default: return "UNKNOWN";
  }
}

#define RAW_STAB_WIN 12
float rawHRWin[RAW_STAB_WIN]={0};
int rawHRWinIdx=0, rawHRWinCount=0;
static bool ghostSuspect=false, rhcSuspect=false;

#define RAW_LATCH_TIMEOUT_MS 3000UL
static float latchedRawHR=0.0f;
static unsigned long latchedRawHRMs=0;
static inline bool latchedRawHRValidAt(unsigned long now) {
  return latchedRawHR > 0.0f && ((now - latchedRawHRMs) < RAW_LATCH_TIMEOUT_MS);
}
static inline bool latchedRawHRValid() {
  return latchedRawHRValidAt(millis());
}
static inline float latchFreshnessAt(unsigned long now) {
  if (!latchedRawHRValidAt(now)) return 0.0f;
  return 1.0f - (float)(now - latchedRawHRMs) / (float)RAW_LATCH_TIMEOUT_MS;
}
static inline float latchFreshness() {
  return latchFreshnessAt(millis());
}

static inline bool currentRawHRAnchor(unsigned long now, float& anchorOut) {
  if (rawHRValid && isfinite(rawHR) && rawHR >= HR_MIN && rawHR <= HR_MAX) {
    anchorOut = rawHR;
    return true;
  }
  if (latchedRawHRValidAt(now) && isfinite(latchedRawHR) && latchedRawHR >= HR_MIN && latchedRawHR <= HR_MAX) {
    anchorOut = latchedRawHR;
    return true;
  }
  return false;
}

static inline bool isTrustedPhaseHRFreshAt(unsigned long now) {
  return trustedPhaseHR > 0.0f && isfinite(trustedPhaseHR) &&
         ((now - lastTrustedPhaseHRMs) < LOG_VITAL_GRACE_MS);
}

static inline bool isTrackingHRFreshAt(unsigned long now) {
  return trackingHR > 0.0f && isfinite(trackingHR) &&
         ((now - lastTrackingHRMs) < HR_STALE_MS);
}

static inline bool hrLowMotionLockFirst() {
  return !persistedMotion && !dopplerMotionDetected && motionCooldown == 0 &&
         !multiTargetDetected && !clusterAnomaly;
}

static inline bool currentGoverningHRAnchor(unsigned long now,
                                            float arbiterAnchorValue,
                                            float rejectPhaseAnchorValue,
                                            float& anchorOut,
                                            uint8_t& sourceOut) {
  if (isTrustedPhaseHRFreshAt(now)) {
    anchorOut = trustedPhaseHR;
    sourceOut = HR_ANCHOR_TRUSTED_PHASE;
    return true;
  }
  if (isfinite(arbiterAnchorValue) && arbiterAnchorValue >= HR_MIN && arbiterAnchorValue <= HR_MAX) {
    anchorOut = arbiterAnchorValue;
    sourceOut = HR_ANCHOR_ARBITER;
    return true;
  }
  if (isfinite(rejectPhaseAnchorValue) && rejectPhaseAnchorValue >= HR_MIN && rejectPhaseAnchorValue <= HR_MAX) {
    anchorOut = rejectPhaseAnchorValue;
    sourceOut = HR_ANCHOR_REJECTPHASE;
    return true;
  }
  if (isTrackingHRFreshAt(now)) {
    anchorOut = trackingHR;
    sourceOut = HR_ANCHOR_TRACKING_RAW;
    return true;
  }
  if (latchedRawHRValidAt(now) && isfinite(latchedRawHR) && latchedRawHR >= HR_MIN && latchedRawHR <= HR_MAX) {
    anchorOut = latchedRawHR;
    sourceOut = HR_ANCHOR_LATCHED_RAW;
    return true;
  }
  anchorOut = NAN;
  sourceOut = HR_ANCHOR_NONE;
  return false;
}

static inline bool rejectPhaseHRUpdate(float candidateHR, unsigned long now, float pqi,
                                        bool arbiterCorrected = false,
                                        bool rawLooksLikeHalfRate = false,
                                        float arbiterAnchorValue = 0.0f) {
  if (!isfinite(candidateHR) || candidateHR < HR_MIN || candidateHR > HR_MAX) return true;

  float bestErr = NAN;
  float bestAnchor = 0.0f;
  uint8_t bestSource = 0; // 0=none,1=raw,2=latch,3=arbiter,4=smooth,5=stable,6=trusted_phase,7=tracking

  auto considerAnchor = [&](float ref, uint8_t src) {
    if (!isfinite(ref) || ref < HR_MIN || ref > HR_MAX) return;
    float err = fabsf(candidateHR - ref);
    if (!isfinite(bestErr) || err < bestErr) {
      bestErr = err;
      bestAnchor = ref;
      bestSource = src;
    }
  };

  bool freshRawOk = rawHRValid && isfinite(rawHR) && rawHR >= HR_MIN && rawHR <= HR_MAX;
  bool latchRawOk = latchedRawHRValidAt(now) && isfinite(latchedRawHR) && latchedRawHR >= HR_MIN && latchedRawHR <= HR_MAX;
  bool trustedPhaseOk = isTrustedPhaseHRFreshAt(now) && trustedPhaseHR >= HR_MIN && trustedPhaseHR <= HR_MAX;
  bool trackingOk = isTrackingHRFreshAt(now) && trackingHR >= HR_MIN && trackingHR <= HR_MAX;

  // Lock-First: a fresh trusted phase anchor governs reject-phase arbitration whenever it exists.
  if (trustedPhaseOk) {
    bestAnchor = trustedPhaseHR;
    bestSource = 6;
    bestErr = fabsf(candidateHR - trustedPhaseHR);
  }

  // v11.6.14a: corrected windows that were fixed because raw/latch looked half-rate
  // must not be vetoed primarily by that same suspect anchor. Prefer trusted anchors.
  if (!trustedPhaseOk && !(arbiterCorrected && rawLooksLikeHalfRate)) {
    if (freshRawOk) considerAnchor(rawHR, 1);
    else if (latchRawOk) considerAnchor(latchedRawHR, 2);
  }

  if (arbiterCorrected && isfinite(arbiterAnchorValue) && arbiterAnchorValue >= HR_MIN && arbiterAnchorValue <= HR_MAX) {
    considerAnchor(arbiterAnchorValue, 3);
  }
  if (isHRFresh() && isfinite(smoothHR) && smoothHR >= HR_MIN && smoothHR <= HR_MAX) {
    considerAnchor(smoothHR, 4);
  }
  if (isfinite(lastStableHR) && lastStableHR >= HR_MIN && lastStableHR <= HR_MAX) {
    considerAnchor(lastStableHR, 5);
  }
  if (trackingOk) {
    considerAnchor(trackingHR, 7);
  }

  // Fallback only if no trusted anchor was available.
  if (!isfinite(bestErr)) {
    if (freshRawOk) considerAnchor(rawHR, 1);
    else if (latchRawOk) considerAnchor(latchedRawHR, 2);
  }

  if (!isfinite(bestErr)) {
    hrRawAgreementGood = true;
    lastHRAgreementErr = NAN;
    hrRejectPhaseAnchorUsed = 0;
    hrRejectPhaseAnchorValue = 0.0f;
    return false;
  }

  lastHRAgreementErr = bestErr;
  hrRejectPhaseAnchorUsed = bestSource;
  hrRejectPhaseAnchorValue = bestAnchor;

  float effectiveDistance = (radarDistance > 1.0f) ? radarDistance : lastGoodDistance;
  bool nearField = (effectiveDistance > 1.0f && effectiveDistance < 40.0f);
  bool earlySettle = (lastPersonDetectedEventMs > 0UL) && (safeElapsedMs(now, lastPersonDetectedEventMs) < 15000UL);

  float plausMax = HR_SPEC_RAW_PLAUS_MAX_DIFF;
  float hardRejectBpm = HR_RAW_DISAGREE_HARD_BPM;
  float softRejectBpm = HR_RAW_DISAGREE_SOFT_BPM;
  if (nearField) {
    plausMax += 4.0f;
    hardRejectBpm += 2.0f;
    softRejectBpm += 2.0f;
  }
  if (earlySettle) {
    plausMax += 3.0f;
    hardRejectBpm += 2.0f;
    softRejectBpm += 2.0f;
  }

  hrRawAgreementGood = (bestErr <= plausMax);
  if (bestErr > plausMax) return true;

  if (pqi < HR_PQI_WEAK_REJECT && bestErr > hardRejectBpm) return true;
  if (pqi < HR_PQI_SOFT_REJECT && bestErr > softRejectBpm) return true;
  return false;
}
static inline bool acceptPhaseRRUpdate(float candidateRR, float confRR) {
  if (!isfinite(candidateRR) || candidateRR < RR_MIN || candidateRR > RR_MAX) return false;
  if (pqiBreath < RR_PQI_MIN_ACCEPT) return false;
  if (confRR < RR_CONF_MIN_ACCEPT) return false;
  if (rawRRValid && isfinite(rawRR) && fabsf(candidateRR - rawRR) > RR_RAW_DISAGREE_BPM && pqiBreath < 0.25f) {
    return false;
  }
  return true;
}

static const float HR_STABILITY_VAR_SCALE = 100.0f; // BPM^2 scale for raw-HR variance -> [0,1] stability

float rawHRStability() {
  if (rawHRWinCount<3) return 0.0f;
  float mean=0;
  for (int i=0;i<rawHRWinCount;i++) mean+=rawHRWin[i];
  mean/=rawHRWinCount;
  float var=0;
  for (int i=0;i<rawHRWinCount;i++) { float d=rawHRWin[i]-mean; var+=d*d; }
  float denom = (rawHRWinCount > 1) ? (float)(rawHRWinCount - 1) : (float)rawHRWinCount;
  return constrain(1.0f - var / denom / HR_STABILITY_VAR_SCALE, 0.0f, 1.0f);
}

bool rawHRGhostSuspect() {
  if (rawHRWinCount<8) return false;
  float mean=0; for (int i=0; i<rawHRWinCount; i++) mean+=rawHRWin[i];
  mean/=rawHRWinCount;
  float var=0; for (int i=0; i<rawHRWinCount; i++) { float d=rawHRWin[i]-mean; var+=d*d; }
  float stddev = sqrtf(var/rawHRWinCount);

  bool lowVar = stddev < 2.0f;
  bool weakPQI = pqiHeart < 0.40f;
  bool noRaw = (!rawHRValid && !latchedRawHRValid());

  return lowVar && weakPQI && noRaw;
}

bool radarIsPresent=false;
bool radarIsPresentRaw=false;
static uint8_t radarPresentScore = 0;
static uint8_t radarAbsentScore = 0;
static unsigned long falsePresenceStartMs = 0;
static uint8_t falsePresenceAbsenceScore = 0;
static unsigned long noHumanClearStartMs = 0;
static uint8_t rrFundamentalRecoveryCount = 0;
static const uint8_t RAW_PRESENT_VOTES = 4;   // reacquire in ~1 s at the 4 Hz evidence tick
static const uint8_t RAW_ABSENT_VOTES  = 24;  // require ~6 s of sustained absence before dropping
float prevTotalPhase=0, phaseDelta=0;
const unsigned long PRESENCE_KEEPALIVE_TIMEOUT=15000UL;
int consecutiveBadRadar=0;
const int BAD_RADAR_LIMIT=20;
static unsigned long lastRadarPacketMs = 0;
static unsigned long radarWatchdogStartMs = 0;
static unsigned long lastBadRadarTickMs = 0;
static unsigned long lastRadarRecoveryAttemptMs = 0;
const unsigned long RADAR_SILENCE_TIMEOUT_MS = 1000UL;
const unsigned long BAD_RADAR_TICK_MS = 50UL;
const unsigned long RADAR_RECOVERY_COOLDOWN_MS = 500UL;
float prevAmbient=NAN;
static float ambientFastEma=NAN;
static float ambientSlowEma=NAN;
static float ambientDrift = 0.0f;
unsigned long lastMlxRetry=0, lastDisplay=0;
static unsigned long lastHrAutoDebugMs = 0;
static unsigned long lastGateBugLogMs = 0;
static unsigned long lastHrCandidateLogMs = 0;
static unsigned long lastPresenceDebugLogMs = 0;

float coherenceFilter(float newHR) {
  if (!isfinite(newHR)) return NAN;
  if (lastStableHR==0) { lastStableHR=newHR; return newHR; }
  float delta = newHR - lastStableHR;
  float diff=fabsf(delta);
  bool downwardMove = delta < 0.0f;
  float jumpLimit=(motionCooldown>0)?45.0f:25.0f;
  int patienceLimit=(motionCooldown>0)?4:8;
  if (hrDownwardOverrideActive && downwardMove) {
    jumpLimit /= HR_DOWNWARD_COHERENCE_RESIST_MULT;
    patienceLimit = max(1, (int)lroundf((float)patienceLimit * HR_DOWNWARD_COHERENCE_RESIST_MULT));
  }

  if (diff>jumpLimit) {
    rejectionCount++;
    hrCoherenceRejectedThisWindow = true;
    Serial.printf("[COHERENCE_REJECT] new=%.1f stable=%.1f diff=%.1f count=%d/%d motion_cd=%d\n",
                  newHR, lastStableHR, diff, rejectionCount, patienceLimit, motionCooldown);
    if (rejectionCount>=patienceLimit) { lastStableHR=newHR; rejectionCount=0; return newHR; }
    return NAN;
  }
  rejectionCount=0;
  if (diff>10.0f) {
    float blendTowardNew = (hrDownwardOverrideActive && downwardMove) ? 0.75f : 0.50f;
    newHR = lastStableHR + (newHR - lastStableHR) * blendTowardNew;
  }
  lastStableHR=newHR; return newHR;
}
void decayHRState() {
  if (lastHRUpdateMs==0) return;
  unsigned long stale=millis()-lastHRUpdateMs;
  if (stale>=HR_STALE_MS && hrState>1) hrState=1;
  if (stale>=2*HR_STALE_MS && hrState>0) hrState=0;
}

void updateHRDisplayCache(unsigned long now) {
  if (isHRFresh() && hrState >= 2 && smoothHR >= HR_MIN) {
    lastHRDisplayedMs = now;
    lastDisplayedHR = smoothHR;
  }
}

bool nvsNeedsWrite() {
  if (lastSavedGain<0) return true;
  if (!isfinite(radarGain)||!isfinite(kfHR_x)||!isfinite(kfRR_x)) return false;
  return (fabsf(radarGain-lastSavedGain)>0.05f)||(fabsf(kfHR_x-lastSavedHR)>2.0f)||(fabsf(kfRR_x-lastSavedRR)>1.0f);
}

float unwrap(float phase, float& prev, float& offset) {
  float diff = phase - prev;
  if (diff > PI) offset -= 2.0f*PI;
  if (diff < -PI) offset += 2.0f*PI;
  prev = phase;
  return phase + offset;
}

float removeClutter(float s, float& clutter) {
  clutter=(1.0f - currentClutterAlpha)*clutter + currentClutterAlpha*s; return s-clutter;
}

float phaseDiffFn(float phase, float& prev) {
  float diff=phase-prev; prev=phase; return diff;
}

static inline bool v13_rawHRInRestingZone(float hr) {
  return isfinite(hr) && hr >= 48.0f && hr <= 80.0f;
}

static void v13_resetSpatialState() {
  lastPointCloudValid = false;
  lastTargetInfoValid = false;
  lastPointCloudRxMs = 0UL;
  lastTargetInfoRxMs = 0UL;
  numDetectedTargets = -1;
  maxDopplerAbs = 0.0f;
  maxDopplerSpeedCms = 0.0f;
  dopplerMotionDetected = false;
  clusterAnomaly = false;
  multiTargetDetected = false;
  primaryTargetX = 0.0f;
  primaryTargetY = 0.0f;
  primaryTargetDop = 0;
  primaryTargetSpeedCms = 0.0f;
  primaryTargetCluster = -1;
  spatialSource = 0;
  spatialFreshAgeMs = 999999UL;
  useDirectRawHR = false;
  directRawHR_consecWindows = 0;
}

static float v13_refinedDistanceCm(float uartDistanceCm, bool uartDistOk) {
  if (lastTargetInfoValid && !lastTargetInfo.targets.empty()) {
    float pcDist = sqrtf(primaryTargetX * primaryTargetX + primaryTargetY * primaryTargetY);
    if (isfinite(pcDist) && pcDist > 0.01f) {
      float pcDistCm = pcDist * 100.0f;
      if (uartDistOk && isfinite(uartDistanceCm) && uartDistanceCm > 1.0f) {
        if (fabsf(uartDistanceCm - pcDistCm) > 50.0f) return pcDistCm;
      } else {
        return pcDistCm;
      }
    }
  }
  return uartDistanceCm;
}

static void v13_pollSpatialFrames(unsigned long now) {
  bool pointCloudHit = mmWave.getPeopleCountingPointCloud(lastPointCloud);
  bool targetInfoHit = mmWave.getPeopleCountingTargetInfo(lastTargetInfo);
  if (pointCloudHit) lastPointCloudRxMs = now;
  if (targetInfoHit) lastTargetInfoRxMs = now;

  lastPointCloudValid = (lastPointCloudRxMs > 0UL) && (safeElapsedMs(now, lastPointCloudRxMs) <= POINT_CLOUD_FRESH_MS);
  lastTargetInfoValid = (lastTargetInfoRxMs > 0UL) && (safeElapsedMs(now, lastTargetInfoRxMs) <= TARGET_INFO_FRESH_MS);

  if (!fwVersionCheckedThisBoot) {
    FirmwareInfo fwTmp;
    if (mmWave.getFirmwareInfo(fwTmp)) {
      moduleVersion = fwTmp;
      moduleVersionValid = true;
      fwVersionCheckedThisBoot = true;
      Serial.printf("[FW_VER] Module: %u.%u.%u.%u\n",
                    moduleVersion.firmware_verson.project_name,
                    moduleVersion.firmware_verson.major_version,
                    moduleVersion.firmware_verson.sub_version,
                    moduleVersion.firmware_verson.modified_version);
    }
  }

  numDetectedTargets = lastTargetInfoValid ? (int)lastTargetInfo.targets.size() : -1;
  multiTargetDetected = (numDetectedTargets > 1);
  maxDopplerAbs = 0.0f;
  maxDopplerSpeedCms = 0.0f;
  dopplerMotionDetected = false;
  clusterAnomaly = false;
  primaryTargetX = 0.0f;
  primaryTargetY = 0.0f;
  primaryTargetDop = 0;
  primaryTargetSpeedCms = 0.0f;
  primaryTargetCluster = -1;
  spatialSource = 0;
  spatialFreshAgeMs = 999999UL;

  if (lastTargetInfoValid && !lastTargetInfo.targets.empty()) {
    spatialSource = 1;
    spatialFreshAgeMs = safeElapsedMs(now, lastTargetInfoRxMs);
    float minDist = 1e9f;
    int distinctClusters = 0;
    int32_t seenClusters[MAX_TARGET_NUM] = {0};
    for (const auto& t : lastTargetInfo.targets) {
      wdtReset();
      float dopAbs = fabsf((float)t.dop_index);
      float dopSpeedCms = dopAbs * RADAR_SPEED_STEP_CM_S;
      if (dopAbs > maxDopplerAbs) maxDopplerAbs = dopAbs;
      if (dopSpeedCms > maxDopplerSpeedCms) maxDopplerSpeedCms = dopSpeedCms;
      if ((int)dopAbs >= DOPPLER_MOTION_THRESHOLD_INDEX) dopplerMotionDetected = true;
      bool clusterSeen = false;
      for (int i = 0; i < distinctClusters; ++i) {
        if (seenClusters[i] == t.cluster_index) {
          clusterSeen = true;
          break;
        }
      }
      if (!clusterSeen && distinctClusters < MAX_TARGET_NUM) {
        seenClusters[distinctClusters++] = t.cluster_index;
      }
      float dist = sqrtf(t.x_point * t.x_point + t.y_point * t.y_point);
      if (dist < minDist) {
        minDist = dist;
        primaryTargetX = t.x_point;
        primaryTargetY = t.y_point;
        primaryTargetDop = t.dop_index;
        primaryTargetSpeedCms = fabsf((float)primaryTargetDop) * RADAR_SPEED_STEP_CM_S;
        primaryTargetCluster = t.cluster_index;
      }
    }
    if (distinctClusters > 1 && numDetectedTargets >= 1) {
      clusterAnomaly = true;
    }
  }

  if (!lastTargetInfoValid && lastPointCloudValid && !lastPointCloud.targets.empty()) {
    spatialSource = 2;
    spatialFreshAgeMs = safeElapsedMs(now, lastPointCloudRxMs);
    int distinctClusters = 0;
    int32_t seenClusters[MAX_TARGET_NUM] = {0};
    numDetectedTargets = (int)lastPointCloud.targets.size();
    multiTargetDetected = (numDetectedTargets > 1);
    float minDist = 1e9f;
    for (const auto& t : lastPointCloud.targets) {
      wdtReset();
      float dopAbs = fabsf((float)t.dop_index);
      float dopSpeedCms = dopAbs * RADAR_SPEED_STEP_CM_S;
      if (dopAbs > maxDopplerAbs) maxDopplerAbs = dopAbs;
      if (dopSpeedCms > maxDopplerSpeedCms) maxDopplerSpeedCms = dopSpeedCms;
      if ((int)dopAbs >= DOPPLER_MOTION_THRESHOLD_INDEX) dopplerMotionDetected = true;
      bool clusterSeen = false;
      for (int i = 0; i < distinctClusters; ++i) {
        if (seenClusters[i] == t.cluster_index) {
          clusterSeen = true;
          break;
        }
      }
      if (!clusterSeen && distinctClusters < MAX_TARGET_NUM) {
        seenClusters[distinctClusters++] = t.cluster_index;
      }
      float dist = sqrtf(t.x_point * t.x_point + t.y_point * t.y_point);
      if (dist < minDist) {
        minDist = dist;
        primaryTargetX = t.x_point;
        primaryTargetY = t.y_point;
        primaryTargetDop = t.dop_index;
        primaryTargetSpeedCms = fabsf((float)primaryTargetDop) * RADAR_SPEED_STEP_CM_S;
        primaryTargetCluster = t.cluster_index;
      }
    }
    if (distinctClusters > 1 && numDetectedTargets >= 1) clusterAnomaly = true;
  }
}


static int v13_presenceBoostPresent() {
  int boost = 1;
  if (numDetectedTargets >= 1) boost += 1;
  if ((int)maxDopplerAbs >= DOPPLER_SUBTLE_THRESHOLD_INDEX && (int)maxDopplerAbs < DOPPLER_MOTION_THRESHOLD_INDEX) boost += 1;
  return boost;
}

static int v13_presenceBoostAbsent() {
  int boost = 1;
  if (numDetectedTargets == 0) boost += 2;
  return boost;
}

static bool v13_motionDetected(float dhSigned, float db, bool roughMotion) {
  bool energyMotion = motionDetected(dhSigned, db) || roughMotion;
  if (lastTargetInfoValid || lastPointCloudValid) return dopplerMotionDetected || (energyMotion && (int)maxDopplerAbs >= DOPPLER_SUBTLE_THRESHOLD_INDEX);
  return energyMotion;
}

static float v13_processHeartPhase(float x, float pqi) {
  if (pqi >= 0.20f && !dopplerMotionDetected && (int)maxDopplerAbs <= DOPPLER_SUBTLE_THRESHOLD_INDEX && !multiTargetDetected && !clusterAnomaly) return x;
  return bpHeartProcess(x);
}

static float v13_processBreathPhase(float x, float pqi) {
  if (pqi >= 0.20f && !dopplerMotionDetected && (int)maxDopplerAbs <= DOPPLER_SUBTLE_THRESHOLD_INDEX && !multiTargetDetected && !clusterAnomaly) return x;
  return bpBreathProcess(x);
}

static bool v13_rawFastPathEligible(unsigned long now, bool inMotionNow, float rawHrValue) {
  bool rawFresh = rawHRValid && isfinite(rawHrValue) && rawHrValue >= HR_MIN && rawHrValue <= HR_MAX;
  if (!rawFresh || inMotionNow || dopplerMotionDetected || multiTargetDetected || clusterAnomaly) return false;
  if ((int)maxDopplerAbs > DOPPLER_SUBTLE_THRESHOLD_INDEX) return false;
  if (!v13_rawHRInRestingZone(rawHrValue)) return false;
  if (rawHR_consecutive_valid < DIRECT_RAW_HR_ENTRY_COUNT) return false;
  if (lastHRUpdateMs > 0 && safeElapsedMs(now, lastHRUpdateMs) > DIRECT_RAW_HR_MAX_STALENESS_MS) return false;
  return true;
}

void resetVitals() {
  smoothHR=70.0f; smoothRR=15.0f; smoothTemp=NAN;
  hrState=0;
  prevHrState=0; hrConfidence=0;
  pqiHeart=0; lastStableHR=0; rejectionCount=0;
  kfHR_x=75.0f; kfHR_P=1.0f; kfRR_x=15.0f; kfRR_P=1.0f;
  bpResetHeart(); bpResetBreath();
  memset(sgBufH,0,sizeof(sgBufH)); memset(sgBufB,0,sizeof(sgBufB));
  sgIdx=0; sgWarmup=0;
  memset(medBufH,0,sizeof(medBufH)); memset(medBufB,0,sizeof(medBufB));
  medIdx=0; rlsReset();
  rawHRWinIdx=0;
  rawHRWinCount=0; memset(rawHRWin,0,sizeof(rawHRWin));
  rawHR_consecutive_valid = 0;
  rawHR_last_seed_ms = 0UL;
  sessionFirstSeedDone = false;
  skipDSP_consecMisses = 0;
  hrArbiterCorrectedThisWindow    = false;
  hrRejectPhaseRejectedThisWindow = false;
  hrCoherenceRejectedThisWindow   = false;
  hrRawSourceThisWindow           = 0;
  hrPreRejectPhase                = 0.0f;
  hrPostRejectPhase               = 0.0f;
  hrPostBlend                     = 0.0f;
  hrPostCoherence                 = 0.0f;
  hrFinalPublishCandidate         = 0.0f;
  hrArbiterAnchorUsed             = false;
  hrArbiterAnchorValue            = 0.0f;
  hrRejectPhaseAnchorUsed         = 0;
  hrRejectPhaseAnchorValue        = 0.0f;
  hrRawLooksLikeHalfRateLogged    = false;
  hrTrustedAnchorValue            = 0.0f;
  hrTrustedPhaseAnchorLogged      = NAN;
  hrAnchorSourceLogged            = HR_ANCHOR_NONE;
  hrAnchorErrBpmLogged            = NAN;
  hrRawHighBiasSuspectLogged      = false;
  rrAnchorFreshLogged             = false;
  rrPreAcceptPhase                = 0.0f;
  rrPostAcceptPhase               = 0.0f;
  rrPostBlend                     = 0.0f;
  rrPostBiasCorrection            = 0.0f;
  rrPostKalman                    = 0.0f;
  rrFinalPublishCandidate         = 0.0f;
  rrAnchorValueLogged             = 0.0f;
  rrAnchorAgeMsLogged             = 0UL;
  rrAnchorSourceLogged            = 0;
  rrAnchorConfidenceLogged        = 0.0f;
  rrFundamentalRecoveryCountLogged= 0;
  rrFundamentalRecoveryTriggeredLogged = false;
  rrRawSeedConsistentCountLogged  = 0;
  rrMidSessionRawReanchorAllowedLogged = false;
  rrMidSessionRawReanchorBlockedLogged = false;
  rrMidSessionRawReanchorReasonLogged  = 0;
  rrRawAnchorErrBpmLogged         = -1.0f;
  hrBypassPqiOk                   = false;
  hrBypassConfOk                  = false;
  hrBypassGateOk                  = false;
  hrBypassActive                  = false;
  hrGraceEligible                 = false;
  hrGraceActive                   = false;
  ghostSuspect=false; rhcSuspect=false; pqiBreath=0;
  hrHarmonicAmbiguous=false; hrRawAgreementGood=false; lastHRAgreementErr=NAN;
  candidateHR=0.0f; candidateRR=0.0f; candidateHRConf=0.0f; candidateRRConf=0.0f;
  lastCandidateHRMs = 0; lastCandidateRRMs = 0; pqiHeartAtCandidateTime = 0.0f;
  hrAgeMsLogged = 0; candidateHrAgeMsLogged = 0; candidateRrAgeMsLogged = 0;
  hrUpdatedThisCycle = false; hrUpdateSourceThisCycle = HR_PATH_NONE; hrConfidenceSourceThisCycle = HR_CONF_NONE; dspRanThisFrame = false;
  rrPhaseBackedUpdateThisCycle = false;
  hrFirstPhaseSeedConfirmCount = 0; hrFirstPhaseSeedConfirmLastBpm = 0.0f; agcFloorRecoveryStartMs = 0UL;
  phaseValidRunLen = 0; phaseInvalidRunLen = 0;
  hrPhaseBackedUpdateCount = 0; rrPhaseBackedUpdateCount = 0;
  nearFieldReflectorSuspect = false; agcFloorSuspect = false;
  phaseBackedPublishReady = false; hrAnchorDriftSuspect = false;
  phaseGapFillCount = 0; clutterRewarmCount = 0;
  hrPathSourceLatched = HR_PATH_NONE; hrConfidenceSourceLatched = HR_CONF_NONE;
  experimentalProfileEnabled = false;
  hrPublishReason = HR_PUB_TRUST_STALE; rrPublishReason = RR_PUB_TRUST_STALE;
  hrGatePqiUsed=0.0f; rrGatePqiUsed=0.0f; hrGateReason=HR_GATE_NO_AUTO; rrGateReason=RR_GATE_NO_AUTO;
  motionLP=TARGET_ENERGY; motionWarmup=0; motionSpikeCount=0; wasMotion=false;
  motionCooldown=0; persistedMotion=false; lastMotionDetectedMs=0;
  rollingEnergy=1e-5f; recalFrames=0;
  prevHeartPhase=0; prevBreathPhase=0; heartOffset=0; breathOffset=0;
  clutterHeart=0; clutterBreath=0; prevHeartDiff=0;
  prevBreathDiff=0;
  prevStableHeartPhase=0; prevStableBreathPhase=0; firstPhaseSample=true;
  v13_resetSpatialState();
  dspTask=0; lastAutoValid=false;
  lastHrAuto=0; lastConfHR=0; lastRRDSP=0; lastConfRR=0; lastRRValid=false;
  radarIsPresent=false; radarIsPresentRaw=false; radarPresentScore=0; radarAbsentScore=0; radarDistance=0;
  prevTotalPhase=0; phaseDelta=0;
  lastValidRateMs=0; lastTrustedVitalMs=0; lastTrustedHRMs=0; lastTrustedRRMs=0; consecutiveBadRadar=0;
  trustedPhaseHR = 0.0f; lastTrustedPhaseHRMs = 0UL;
  trackingHR = 0.0f; lastTrackingHRMs = 0UL;
  hrLowerPersistWindows = 0; hrUpwardConfirmWindows = 0; hrGraceBlockWindows = 0;
  hrDownwardOverrideActive = false; hrPublishGraceBlocked = false;
  lastRadarPacketMs=0; radarWatchdogStartMs=millis(); lastBadRadarTickMs=0; lastRadarRecoveryAttemptMs=0;
  rawHR=0; rawRR=0; rawHRValid=false;
  rawRRValid=false;
  rawRREffective=0.0f; rawRRLikelyHarmonic=false;
  clutterWarmupCount = 0; phaseWarmupComplete = false; currentClutterAlpha = 0.08f; // v13.7: Reset on major state change
  latchedRawHR=0.0f; latchedRawHRMs=0;
  lastValidDisplayRRMs = 0;
  lastValidDisplayRR = 0.0f;
  rrHoldAgreementCount = 0;
  lastHRUpdateMs=0; lastRRUpdateMs=0; lastDistanceUpdateMs=0; lastPresenceUpdateMs=0;
  lastHRDisplayedMs=0; lastDisplayedHR=0;
  lastValidPublishHRMs = 0UL;
  lastValidPublishHR = 0.0f;
  hrVisibleOnVitalsScreen=false; lastIdleRedrawMs=0;
  lastLcdRescanMs=0;
  bufIndex=0; bufCount=0;
  memset(heartBuf,0,sizeof(heartBuf)); memset(breathBuf,0,sizeof(breathBuf));
  memset(fusedBuf,0,sizeof(fusedBuf)); memset(phaseTimes,0,sizeof(phaseTimes));
  bufCount_snap=0; fs_snap=10.0f;
  memset(linearHeart_snap,0,sizeof(linearHeart_snap));
  memset(linearBreath_snap,0,sizeof(linearBreath_snap));
  memset(linearFused_snap,0,sizeof(linearFused_snap));
  lastCalculatedFs=10.0f; lastLedColor=0xFFFFFFFF; lastLedBrightness = 0xFF;
  welcomeShown=false; heartAnimFrame=false; animPending=false;
  lastHRAlertMs=(unsigned long)(0UL - HR_ALERT_COOLDOWN); hrAlertActive=false; lastHRLockedBeepMs=0; lastMotionBeepMs=0;
  hrVarIdx=0; hrVarCount=0; memset(hrVarBuf,0,sizeof(hrVarBuf));
  medWarmup=0;         
  lastPhaseDataMs=0;   
  lastAmbientJumpMs=0; 
  lastAmbientVoteMs=0; 
  ambientDrift = 0.0f; 
  lastLatchedEvidenceMs = 0;
  skipDSP=false;
  lastLoggedHrBandMin = HR_MIN;
  lastLoggedHrBandMax = HR_MAX;
  lastOutlierRRCandidate = 0.0f;
  rrOutlierPersistCount = 0;

  hrArbiterCorrectedThisWindow    = false;
  hrRejectPhaseRejectedThisWindow = false;
  hrCoherenceRejectedThisWindow   = false;
  hrRawSourceThisWindow           = 0;
  hrPreRejectPhase                = 0.0f;
  hrPostRejectPhase               = 0.0f;
  hrPostBlend                     = 0.0f;
  hrPostCoherence                 = 0.0f;
  hrFinalPublishCandidate         = 0.0f;
  hrArbiterAnchorUsed             = false;
  hrArbiterAnchorValue            = 0.0f;
  hrRejectPhaseAnchorUsed         = 0;
  hrRejectPhaseAnchorValue        = 0.0f;
  hrRawLooksLikeHalfRateLogged    = false;
  hrTrustedAnchorValue            = 0.0f;
  rrAnchorFreshLogged             = false;
  rrPreAcceptPhase                = 0.0f;
  rrPostAcceptPhase               = 0.0f;
  rrPostBlend                     = 0.0f;
  rrPostBiasCorrection            = 0.0f;
  rrPostKalman                    = 0.0f;
  rrFinalPublishCandidate         = 0.0f;
  rrAnchorValueLogged             = 0.0f;
  rrAnchorAgeMsLogged             = 0UL;
  rrAnchorSourceLogged            = 0;
  rrAnchorConfidenceLogged        = 0.0f;
  rrFundamentalRecoveryCountLogged= 0;
  rrFundamentalRecoveryTriggeredLogged = false;
  rrRawSeedConsistentCountLogged  = 0;
  rrMidSessionRawReanchorAllowedLogged = false;
  rrMidSessionRawReanchorBlockedLogged = false;
  rrMidSessionRawReanchorReasonLogged  = 0;
  rrRawAnchorErrBpmLogged         = -1.0f;
  hrBypassPqiOk                   = false;
  hrBypassConfOk                  = false;
  hrBypassGateOk                  = false;
  hrBypassActive                  = false;
  hrGraceEligible                 = false;
  hrGraceActive                   = false;

  ghostSuspectCounter = 0;
  lastTrustedVitalMs = 0;
  lastLivePhaseMs = 0;
  phaseLivenessScore = 0.0f;
  presenceConfidence = 0.0f;
  distanceConfidence = 0.0f;
  lastStrongPresenceMs = 0;
  lastWeakPresenceMs = 0;
  lastEvidenceLogMs = 0;
  lastPresenceTickMs = 0;
  lastPhaseLivenessDecayMs = 0;
  lastConfidenceDecayMs = 0;
  latchedStrongEvidence = false;
  latchedWeakEvidence = false;
  entrySawStrongEvidence = false;
  entryStrongSinceMs = 0;
  weakEntryCooldownMs = 0;
  distVarIdx = 0;
  distVarCount = 0;
  currentDistStdDev = 0.0f;
  staticReflectorConsecutive = 0;
  currentStaticReflector = false;
  memset(distVarBuf,0,sizeof(distVarBuf));

  diagAvgE_Pre  = 0.0f;
  diagAvgE_RLS  = 0.0f;
  diagAvgE_Gate = 0.0f;
}

bool applyStaticReflectorPenalty(bool freshDistanceSample, bool livePhaseEvidence) {
  if (!freshDistanceSample) {
    staticReflectorConsecutive = 0;
    currentDistStdDev = 0.0f;
    return false;
  }

  distVarBuf[distVarIdx] = radarDistance;
  distVarIdx = (distVarIdx + 1) % DIST_VAR_WIN;
  if (distVarCount < DIST_VAR_WIN) distVarCount++;

  if (distVarCount < DIST_VAR_WIN) {
    currentDistStdDev = 0.0f;
    staticReflectorConsecutive = 0;
    return false;
  }

  float mean = 0.0f;
  for (int i=0; i<distVarCount; i++) mean += distVarBuf[i];
  mean /= distVarCount;
  float var = 0.0f;
  for (int i=0; i<distVarCount; i++) {
    float d = distVarBuf[i] - mean;
    var += d*d;
  }
  currentDistStdDev = (distVarCount > 1) ? sqrtf(var / (float)(distVarCount - 1)) : 0.0f;

  if (livePhaseEvidence) {
    staticReflectorConsecutive = 0;
    return false;
  }

  bool noPhysioEvidence = !rawHRValid && !rawRRValid && !latchedRawHRValid() &&
                          (pqiHeart < 0.15f) && (pqiBreath < 0.15f);

  if (currentDistStdDev < STATIC_REFLECTOR_MAX_STD_CM && noPhysioEvidence) {
    if (staticReflectorConsecutive < 255) staticReflectorConsecutive++;
  } else {
    staticReflectorConsecutive = 0;
  }

  return staticReflectorConsecutive >= STATIC_REFLECTOR_REQUIRED_CONSECUTIVE;
}

static inline unsigned long safeElapsedMs(unsigned long now, unsigned long since) {
  return now - since;
}
static inline bool isHRFreshAt(unsigned long now) {
  return lastHRUpdateMs > 0 && (safeElapsedMs(now, lastHRUpdateMs) < HR_STALE_MS);
}
static inline bool isRRFreshAt(unsigned long now) {
  return lastRRUpdateMs > 0 && (safeElapsedMs(now, lastRRUpdateMs) < RR_STALE_MS);
}
static inline bool isDistanceFreshAt(unsigned long now) {
  return lastDistanceUpdateMs > 0 && (safeElapsedMs(now, lastDistanceUpdateMs) < DISTANCE_STALE_MS);
}
static inline bool isPresenceFreshAt(unsigned long now) {
  return lastPresenceUpdateMs > 0 && (safeElapsedMs(now, lastPresenceUpdateMs) < PRESENCE_STALE_MS);
}
static inline bool isPhaseFreshAt(unsigned long now) {
  return lastPhaseDataMs > 0 && (safeElapsedMs(now, lastPhaseDataMs) < PHASE_FRESH_REQ_MS);
}
static inline bool isTrustedVitalFreshAt(unsigned long now) {
  return lastTrustedVitalMs > 0 && (safeElapsedMs(now, lastTrustedVitalMs) < LOG_VITAL_GRACE_MS);
}
static inline bool isTrustedHRFreshAt(unsigned long now) {
  return lastTrustedHRMs > 0 && (safeElapsedMs(now, lastTrustedHRMs) < LOG_VITAL_GRACE_MS);
}
static inline bool isTrustedRRFreshAt(unsigned long now) {
  return lastTrustedRRMs > 0 && (safeElapsedMs(now, lastTrustedRRMs) < LOG_VITAL_GRACE_MS);
}


static const char* presenceStateName(uint8_t s) {
  switch (s) {
    case PRESENCE_ABSENT: return "ABSENT";
    case PRESENCE_ENTERING: return "ENTERING";
    case PRESENCE_PRESENT: return "PRESENT";
    case PRESENCE_SILENT_HOLD: return "SILENT_HOLD";
    case PRESENCE_LEAVING: return "LEAVING";
    default: return "UNKNOWN";
  }
}

static void logEvent(const char* eventName, const char* detail) {
#if (LOG_MODE > 0)
  Serial.printf("EVENT,%lu,%s,%s\n", millis(), eventName, detail ? detail : "");
#else
  (void)eventName; (void)detail;
#endif
}

static float computeDistanceConfidenceScore(float distCm, bool fresh) {
  if (!fresh || distCm <= 0.0f) return 0.0f;
  if (distCm >= 40.0f && distCm <= 100.0f) return 1.0f;
  if (distCm >= 30.0f && distCm <= 140.0f) return 0.78f;
  if (distCm >= 25.0f && distCm <= 180.0f) return 0.50f;
  return 0.15f;
}

static const char* distanceZoneLabel(float distCm, bool fresh) {
  if (!fresh || distCm <= 0.0f) return "Unknown";
  if (distCm < 30.0f) return "TooClose";
  if (distCm <= 100.0f) return "Optimal";
  if (distCm <= 140.0f) return "OK";
  if (distCm <= 180.0f) return "Far";
  return "TooFar";
}

static void sessionReset() {
  sessionStats.active = false;
  sessionStats.startMs = 0;
  sessionStats.endMs = 0;
  sessionStats.lastUpdateMs = 0;
  sessionStats.motionMs = 0;
  sessionStats.lockMs = 0;
  sessionStats.hrSum = 0.0;
  sessionStats.hrTimeMs = 0UL;
  sessionStats.hrCount = 0;
  sessionStats.rrSum = 0.0;
  sessionStats.rrTimeMs = 0UL;
  sessionStats.rrCount = 0;
  sessionStats.tempSum = 0.0;
  sessionStats.tempTimeMs = 0UL;
  sessionStats.tempCount = 0;
  sessionStats.hrMin = 999.0f;
  sessionStats.hrMax = 0.0f;
}

static void sessionStart(unsigned long now) {
  sessionReset();
  sessionStats.active = true;
  sessionStats.startMs = now;
  sessionStats.lastUpdateMs = now;
}

static void sessionUpdate(unsigned long now, bool inMotionNow) {
  if (!sessionStats.active) return;
  unsigned long dt = (sessionStats.lastUpdateMs > 0) ?
                     safeElapsedMs(now, sessionStats.lastUpdateMs) : 0UL;
  sessionStats.lastUpdateMs = now;
  if (inMotionNow) sessionStats.motionMs += dt;
  if (hrState >= 2 && isHRFresh()) sessionStats.lockMs += dt;
  if (dt > 0 && isHRFresh() && smoothHR >= HR_MIN && smoothHR <= HR_MAX) {
    sessionStats.hrSum += (double)smoothHR * (double)dt;
    sessionStats.hrTimeMs += dt;
    sessionStats.hrCount++;
    if (smoothHR < sessionStats.hrMin) sessionStats.hrMin = smoothHR;
    if (smoothHR > sessionStats.hrMax) sessionStats.hrMax = smoothHR;
  }
  if (dt > 0 && isRRFresh() && smoothRR >= RR_MIN && smoothRR <= RR_MAX) {
    sessionStats.rrSum += (double)smoothRR * (double)dt;
    sessionStats.rrTimeMs += dt;
    sessionStats.rrCount++;
  }
  if (dt > 0 && !isnan(smoothTemp) && isfinite(smoothTemp)) {
    sessionStats.tempSum += (double)smoothTemp * (double)dt;
    sessionStats.tempTimeMs += dt;
    sessionStats.tempCount++;
  }
}

static void sessionPrintSummary(unsigned long now) {
  if (!sessionStats.active) return;
  sessionStats.endMs = now;
  unsigned long durMs = sessionStats.endMs - sessionStats.startMs;
  float avgHR = sessionStats.hrTimeMs > 0 ? (float)(sessionStats.hrSum / (double)sessionStats.hrTimeMs) : 0.0f;
  float avgRR = sessionStats.rrTimeMs > 0 ? (float)(sessionStats.rrSum / (double)sessionStats.rrTimeMs) : 0.0f;
  float avgTemp = sessionStats.tempTimeMs > 0 ? (float)(sessionStats.tempSum / (double)sessionStats.tempTimeMs) : NAN;
  float lockPct = durMs > 0 ? (100.0f * (float)sessionStats.lockMs / (float)durMs) : 0.0f;
  float motionPct = durMs > 0 ? (100.0f * (float)sessionStats.motionMs / (float)durMs) : 0.0f;

  Serial.println("SESSION_SUMMARY_BEGIN");
  Serial.printf("SESSION,duration_s,%lu\n", durMs/1000UL);
  Serial.printf("SESSION,avg_hr,%.2f\n", avgHR);
  Serial.printf("SESSION,min_hr,%.2f\n", (sessionStats.hrCount>0 ? sessionStats.hrMin : 0.0f));
  Serial.printf("SESSION,max_hr,%.2f\n", (sessionStats.hrCount>0 ? sessionStats.hrMax : 0.0f));
  Serial.printf("SESSION,avg_rr,%.2f\n", avgRR);
  Serial.printf("SESSION,avg_temp,%.2f\n", (isnan(avgTemp) ? 0.0f : avgTemp));
  Serial.printf("SESSION,lock_pct,%.1f\n", lockPct);
  Serial.printf("SESSION,motion_pct,%.1f\n", motionPct);
  Serial.println("SESSION_SUMMARY_END");
}

static void enterPresenceState(uint8_t next, unsigned long now, const char* reason) {
  if (presenceState == next) return;
  if (next != PRESENCE_ENTERING) {
    entrySawStrongEvidence = false;
    entryStrongSinceMs = 0;
  }
  presenceState = next;
  presenceStateSinceMs = now;
  char detail[64];
  snprintf(detail, sizeof(detail), "%s", reason ? reason : "");
  logEvent(presenceStateName(next), detail);
}

static void forceClearAllVitalState() {
  smoothHR = 0.0f;
  smoothRR = 0.0f;
  candidateHR = 0.0f;
  candidateRR = 0.0f;
  candidateHRConf = 0.0f;
  candidateRRConf = 0.0f;
  rawHR = 0.0f; rawRR = 0.0f;
  rawHRValid = false; rawRRValid = false;
  rawRREffective = 0.0f; rawRRLikelyHarmonic = false;
  latchedRawHR = 0.0f; latchedRawHRMs = 0;
  lastValidDisplayRR = 0.0f; lastValidDisplayRRMs = 0;
  lastDisplayedHR = 0.0f; lastHRDisplayedMs = 0;
  lastHRUpdateMs = 0; lastRRUpdateMs = 0;
  lastTrustedVitalMs = 0; lastTrustedHRMs = 0; lastTrustedRRMs = 0;
  trustedPhaseHR = 0.0f; lastTrustedPhaseHRMs = 0UL;
  trackingHR = 0.0f; lastTrackingHRMs = 0UL;
  lastValidPublishHRMs = 0; lastValidPublishHR = 0.0f;
  hrLowerPersistWindows = 0; hrUpwardConfirmWindows = 0; hrGraceBlockWindows = 0;
  hrDownwardOverrideActive = false; hrPublishGraceBlocked = false;
  hrState = 0; prevHrState = 0;
  hrConfidence = 0.0f; pqiHeart = 0.0f; pqiBreath = 0.0f;
  lastStableHR = 0.0f; rejectionCount = 0;
  rrPhaseBackedUpdateThisCycle = false;
  hrFirstPhaseSeedConfirmCount = 0; hrFirstPhaseSeedConfirmLastBpm = 0.0f; agcFloorRecoveryStartMs = 0UL;
  phaseValidRunLen = 0; phaseInvalidRunLen = 0;
  hrPhaseBackedUpdateCount = 0; rrPhaseBackedUpdateCount = 0;
  nearFieldReflectorSuspect = false; agcFloorSuspect = false;
  phaseBackedPublishReady = false; hrAnchorDriftSuspect = false;
  phaseGapFillCount = 0; clutterRewarmCount = 0;
  hrPathSourceLatched = HR_PATH_NONE; hrConfidenceSourceLatched = HR_CONF_NONE;
  experimentalProfileEnabled = false;
  kfHR_x = 75.0f; kfHR_P = 1.0f;
  kfRR_x = 15.0f; kfRR_P = 1.0f;
  rrOutlierPersistCount = 0; lastOutlierRRCandidate = 0.0f; rrFundamentalRecoveryCount = 0;
  rrSeedConsistentCount = 0; rrSeedLastRaw = NAN;
  bufIndex = 0; bufCount = 0;
  memset(heartBuf,0,sizeof(heartBuf)); memset(breathBuf,0,sizeof(breathBuf));
  memset(fusedBuf,0,sizeof(fusedBuf)); memset(phaseTimes,0,sizeof(phaseTimes));
  memset(linearHeart_snap,0,sizeof(linearHeart_snap));
  memset(linearBreath_snap,0,sizeof(linearBreath_snap));
  memset(linearFused_snap,0,sizeof(linearFused_snap));
  bpResetHeart(); bpResetBreath(); rlsReset();
  sgIdx = 0; sgWarmup = 0; medIdx = 0; medWarmup = 0;
  rawHRWinIdx = 0; rawHRWinCount = 0; memset(rawHRWin,0,sizeof(rawHRWin));
  hrColdStartWindowsRemaining = 0;
  motionCooldown = 0;
}

static void handlePersonDetected(unsigned long now) {
  bool seedRawHRConsistent = (rawHR_consecutive_valid >= 2);
  bool seedRawHR = rawHRValid && seedRawHRConsistent && isfinite(rawHR) && rawHR >= HR_MIN && rawHR <= HR_MAX;
  bool seedRawRRConsistent = rrSeedConsistentCount >= RR_SEED_MIN_CONSISTENT_READS;
  bool seedRawRRPlausible = rawRRValid && isfinite(rawRREffective) && rawRREffective >= RR_MIN && rawRREffective <= RR_SEED_MAX_BPM;
  bool seedRawRR = seedRawRRConsistent && seedRawRRPlausible;
  float seedHR = seedRawHR ? rawHR : 75.0f;
  float seedRR = seedRawRR ? rawRREffective : 15.0f;

  sessionStart(now);
  lastStrongPresenceMs = now;
  lastWeakPresenceMs = now;
  presentVotes = 5;
  absentVotes = 0;
  falsePresenceStartMs = 0;
  falsePresenceAbsenceScore = 0;
  noHumanClearStartMs = 0;

  // Reset transient/session state first, then apply any validated warm-start seed.
  rrFundamentalRecoveryCount = 0;
  hrColdStartWindowsRemaining = HR_COLD_START_MAX_WINDOWS;
  lastPersonDetectedEventMs = now;
  motionLP = TARGET_ENERGY;
  motionWarmup = 0;
  motionSpikeCount = 0;
  wasMotion = false;
  motionCooldown = 0;
  persistedMotion = false;
  bufIndex = 0;
  bufCount = 0;
  bpResetHeart(); bpResetBreath();
  rlsReset();
  memset(sgBufH,0,sizeof(sgBufH)); memset(sgBufB,0,sizeof(sgBufB));
  sgIdx = 0; sgWarmup = 0; medIdx = 0; medWarmup = 0;
  clutterHeart = 0.0f; clutterBreath = 0.0f;
  prevHeartDiff = 0.0f; prevBreathDiff = 0.0f;
  prevHeartPhase = 0.0f; prevBreathPhase = 0.0f;
  prevStableHeartPhase = 0.0f; prevStableBreathPhase = 0.0f;
  heartOffset = 0.0f; breathOffset = 0.0f;
  firstPhaseSample = true;
  skipDSP = false; dspTask = 0; lastAutoValid = false; lastRRValid = false;
  phaseDelta = 0.0f; prevTotalPhase = 0.0f;
  pqiHeart = 0.0f; pqiBreath = 0.0f; hrConfidence = 0.0f;
  candidateHR = 0.0f; candidateRR = 0.0f; candidateHRConf = 0.0f; candidateRRConf = 0.0f;
  lastCandidateHRMs = 0; lastCandidateRRMs = 0; pqiHeartAtCandidateTime = 0.0f;
  lastHRUpdateMs = 0; lastRRUpdateMs = 0;
  lastTrustedVitalMs = 0; lastTrustedHRMs = 0; lastTrustedRRMs = 0; lastValidRateMs = 0;
  trustedPhaseHR = 0.0f; lastTrustedPhaseHRMs = 0UL;
  trackingHR = 0.0f; lastTrackingHRMs = 0UL;
  lastHRDisplayedMs = 0; lastDisplayedHR = 0.0f;
  lastValidPublishHRMs = 0UL; lastValidPublishHR = 0.0f;
  hrLowerPersistWindows = 0; hrUpwardConfirmWindows = 0; hrGraceBlockWindows = 0;
  hrDownwardOverrideActive = false; hrPublishGraceBlocked = false;
  rrPhaseBackedUpdateThisCycle = false;
  hrFirstPhaseSeedConfirmCount = 0; hrFirstPhaseSeedConfirmLastBpm = 0.0f; agcFloorRecoveryStartMs = 0UL;
  phaseValidRunLen = 0; phaseInvalidRunLen = 0;
  hrPhaseBackedUpdateCount = 0; rrPhaseBackedUpdateCount = 0;
  nearFieldReflectorSuspect = false; agcFloorSuspect = false;
  phaseBackedPublishReady = false; hrAnchorDriftSuspect = false;
  phaseGapFillCount = 0; clutterRewarmCount = 0;
  hrPathSourceLatched = HR_PATH_NONE; hrConfidenceSourceLatched = HR_CONF_NONE;
  experimentalProfileEnabled = false;
  hrVisibleOnVitalsScreen = false;
  hrAlertActive = false;
  currentStaticReflector = false; staticReflectorConsecutive = 0; currentDistStdDev = 0.0f;
  distVarIdx = 0; distVarCount = 0; memset(distVarBuf, 0, sizeof(distVarBuf));
  ghostSuspect = false; rhcSuspect = false; ghostSuspectCounter = 0; dbEnvLP = 0.0f;
  hrHarmonicAmbiguous = false; hrRawAgreementGood = false; lastHRAgreementErr = NAN;
  hrPublishReason = HR_PUB_TRUST_STALE; rrPublishReason = RR_PUB_TRUST_STALE;
  entrySawStrongEvidence = false;
  entryStrongSinceMs = 0UL;
  weakEntryCooldownMs = 0UL;
  rawHR_last_seed_ms = 0UL;
  sessionFirstSeedDone = false;
  skipDSP_consecMisses = 0;
  hrArbiterCorrectedThisWindow    = false;
  hrRejectPhaseRejectedThisWindow = false;
  hrCoherenceRejectedThisWindow   = false;
  hrRawSourceThisWindow           = 0;
  hrPreRejectPhase                = 0.0f;
  hrPostRejectPhase               = 0.0f;
  hrPostBlend                     = 0.0f;
  hrPostCoherence                 = 0.0f;
  hrFinalPublishCandidate         = 0.0f;
  hrArbiterAnchorUsed             = false;
  hrArbiterAnchorValue            = 0.0f;
  hrRejectPhaseAnchorUsed         = 0;
  hrRejectPhaseAnchorValue        = 0.0f;
  hrRawLooksLikeHalfRateLogged    = false;
  hrTrustedPhaseAnchorLogged      = NAN;
  hrAnchorSourceLogged            = HR_ANCHOR_NONE;
  hrAnchorErrBpmLogged            = NAN;
  hrRawHighBiasSuspectLogged      = false;
  hrTrustedAnchorValue            = 0.0f;
  rrAnchorFreshLogged             = false;
  rrPreAcceptPhase                = 0.0f;
  rrPostAcceptPhase               = 0.0f;
  rrPostBlend                     = 0.0f;
  rrPostBiasCorrection            = 0.0f;
  rrPostKalman                    = 0.0f;
  rrFinalPublishCandidate         = 0.0f;
  rrAnchorValueLogged             = 0.0f;
  rrAnchorAgeMsLogged             = 0UL;
  rrAnchorSourceLogged            = 0;
  rrAnchorConfidenceLogged        = 0.0f;
  rrFundamentalRecoveryCountLogged= 0;
  rrFundamentalRecoveryTriggeredLogged = false;
  rrRawSeedConsistentCountLogged  = 0;
  rrMidSessionRawReanchorAllowedLogged = false;
  rrMidSessionRawReanchorBlockedLogged = false;
  rrMidSessionRawReanchorReasonLogged  = 0;
  rrRawAnchorErrBpmLogged         = -1.0f;
  hrBypassPqiOk                   = false;
  hrBypassConfOk                  = false;
  hrBypassGateOk                  = false;
  hrBypassActive                  = false;
  hrGraceEligible                 = false;
  hrGraceActive                   = false;

  // Apply validated warm-start seeds after reset.
  kfHR_x = seedHR;
  smoothHR = seedHR;
  lastStableHR = seedRawHR ? seedHR : 0.0f;
  if (seedRawHR) {
    lastHRUpdateMs = now;
    trackingHR = seedHR;
    lastTrackingHRMs = now;
  }

  kfRR_x = seedRR;
  smoothRR = seedRR;
  if (seedRawRR) {
    lastRRUpdateMs = now;
    lastTrustedRRMs = now;
    lastTrustedVitalMs = now;
  }
  Serial.printf("[RR_SEED] raw=%.1f accepted=%.1f consistent=%u\n", rawRR, seedRR, (unsigned)rrSeedConsistentCount);
}

static void handlePersonLeft(unsigned long now, const char* reason) {
  reentryLockoutMs = now;
  presentVotes = 0;
  absentVotes = 0;
  sessionPrintSummary(now);
  if (!buzzerIsBusy()) buzzerPlay(BUZZ_PERSON_LEFT);

  if ((nvsEverWritten==false || (now-lastNvsWriteMs>=NVS_WRITE_MIN_INTERVAL_MS)) &&
      nvsNeedsWrite() && isfinite(radarGain) && isfinite(kfHR_x) && isfinite(kfRR_x)) {
    wdtReset(); buzzerUpdate();
    prefs.end();
    if (!prefs.begin("rvital",false)) {
      Serial.println("[NVS] ERROR: Could not open namespace for write");
      prefs.end();
      if (!buzzerIsBusy()) buzzerPlay(BUZZ_SENSOR_ERROR);
    } else {
      bool okGain = prefs.putFloat("gain",radarGain);
      wdtReset(); buzzerUpdate();
      bool okHR = prefs.putFloat("kfHR",kfHR_x);
      wdtReset(); buzzerUpdate();
      bool okRR = prefs.putFloat("kfRR",kfRR_x);
      wdtReset(); buzzerUpdate();
      prefs.end();
      wdtReset(); buzzerUpdate();
      if (okGain && okHR && okRR) {
        lastNvsWriteMs=now;
        nvsEverWritten=true;
        lastSavedGain=radarGain;
        lastSavedHR=kfHR_x;
        lastSavedRR=kfRR_x;
        Serial.printf("[NVS] Saved gain=%.3f HR=%.1f RR=%.1f\n", radarGain, kfHR_x, kfRR_x);
      } else {
        Serial.println("[NVS] ERROR: putFloat failed");
        if (!buzzerIsBusy()) buzzerPlay(BUZZ_SENSOR_ERROR);
      }
    }
  }

  goodbyeStartMs = now;
  dispState = DISP_GOODBYE;
  lastDisplay = 0;
  char detail[64];
  snprintf(detail, sizeof(detail), "%s", reason ? reason : "session_end");
  logEvent("PERSON_LEFT", detail);
  Serial.println("[PRESENCE] Left");
  sessionReset();
  resetVitals();
}

// =========================================================================
// LED CONTROL
// =========================================================================
static inline uint8_t ledBrightnessFromLux(float lux) {
  const float LED_BRIGHT_LUX_MIN = 1.0f;
  const float LED_BRIGHT_LUX_MAX = 120.0f;
  float x = (lux - LED_BRIGHT_LUX_MIN) / (LED_BRIGHT_LUX_MAX - LED_BRIGHT_LUX_MIN);
  x = constrain(x, 0.0f, 1.0f);
  x = sqrtf(x);
  return (uint8_t)lroundf(8.0f + x * (40.0f - 8.0f));
}

void updateLED(bool inMotion) {
  static unsigned long ledMillis=0; static bool flashState=false;
  uint8_t brite = ledBrightnessFromLux(luxLevel);
  uint32_t colour; unsigned long now=millis();

  if (!humanDetected) {
    float t=(now%4000)/4000.0f;
    uint8_t b=(uint8_t)(10.0f+25.0f*(0.5f+0.5f*sinf(t*2.0f*PI)));
    colour=pixel.Color(0,0,b);
  } else if (hrAlertActive) {
    if (now-ledMillis>=200) { ledMillis=now; flashState=!flashState; }
    colour=flashState?pixel.Color(80,0,0):pixel.Color(20,0,0);
  } else if (inMotion) {
    if (now-ledMillis>=150) { ledMillis=now; flashState=!flashState; }
    colour=flashState?pixel.Color(60,40,0):pixel.Color(20,15,0);
  } else if (hrState==2) {
    float hr=constrain(smoothHR,HR_MIN,HR_MAX);
    if (!isfinite(hr)) hr=72.0f;
    unsigned long period=constrain((unsigned long)(60000.0f/hr),200UL,1500UL);
    float phase=(float)(now%period)/(float)period;
    float pulse=powf(sinf(phase*PI),2.0f);
    colour=pixel.Color((uint8_t)(2+8*pulse),(uint8_t)(15+50*pulse),(uint8_t)(2+5*pulse));
  } else if (hrState==1) {
    float t=(now%3000)/3000.0f;
    uint8_t v=(uint8_t)(10.0f + 20.0f * (0.5f + 0.5f * sinf(t*2.0f*PI)));
    colour=pixel.Color(0,v,v);
  } else { colour=pixel.Color(10,10,10); }

  bool colourChanged = (colour != lastLedColor);
  bool brightnessChanged = (brite != lastLedBrightness);
  if (colourChanged || brightnessChanged) {
    pixel.setBrightness(brite);
    pixel.setPixelColor(0, colour);
    pixel.show();
    lastLedColor = colour;
    lastLedBrightness = brite;
  }
}

// =========================================================================
// HEART ANIMATION
// =========================================================================
void updateHeartAnimation() {
  if (!lcdPtr||!lcdConnected||!customCharsValid||dispState!=DISP_VITALS) return;
  unsigned long now=millis();
  bool hrShowable = hrVisibleOnVitalsScreen;
  if (!hrShowable) { if (heartAnimFrame) { heartAnimFrame=false; setHeartChar(false); } return; }

  float hr=constrain(isHRFresh()?smoothHR:lastDisplayedHR,HR_MIN,HR_MAX);
  if (!isfinite(hr)) hr=72.0f;
  unsigned long beatPeriod=constrain((unsigned long)(60000.0f/hr),300UL,1500UL);
  bool systole=((now%beatPeriod)<(beatPeriod/5));
  if (systole!=heartAnimFrame) { heartAnimFrame=systole; setHeartChar(systole); }
}

// =========================================================================
// DISPLAY RENDERING
// =========================================================================
void renderInitScreen(int step, int total, const char* sensorName) {
  if (!lcdPtr) return;
  lcdPrintCentered(0,"Starting Up...");
  lcdPrintCentered(1,"");
  char stepStr[21]; snprintf(stepStr,sizeof(stepStr),"Init: %.12s",sensorName);
  lcdPrintCentered(2,stepStr);
  drawProgressBar(3,(step*100)/total,true);
}

void renderCalibrationScreen(bool forceRedraw) {
  if (!lcdPtr||!lcdConnected) return;
  int pct=calibrationCount>0?constrain((calibrationCount*100)/CALIBRATION_FRAMES,0,100):0;
  unsigned long now=millis();
  if (!forceRedraw&&pct==lastCalibPct&&now-lastCalibDisplay<250) return;
  lastCalibDisplay=now; lastCalibPct=pct;
  int elapsed=(int)((now-calibStartMs)/1000);

  lcdWriteCachedCenteredRow(0,"Calibrating Sensor", forceRedraw);
  lcdWriteCachedCenteredRow(1,(calibrationCount==0&&elapsed>2)?"Waiting for radar...":(pct<50)?"Please stand back":"Almost ready...", forceRedraw);
  drawProgressBar(2,pct,true);
  char row3[21];
  snprintf(row3,sizeof(row3),"Time left: %d sec",max(0,(int)(CALIB_TIMEOUT_MS/1000)-elapsed));
  lcdWriteCachedCenteredRow(3,row3, forceRedraw);
}

void startCalibResult(bool complete) {
  buzzerPlay(complete?BUZZ_CALIBRATION_DONE:BUZZ_CALIBRATION_FAIL);
  calibResultStartMs=millis(); dispState=DISP_CALIB_RESULT;
  prevDispState=DISP_CALIB_RESULT; lastCalibPct=-1;
  if (!lcdConnected||!lcdPtr) return;
  lcdSmoothTransition(); lcdPtr->clear();
  lcdPrintCentered(0,complete?"Calibration Done!":"Using Defaults");
  lcdPrintCentered(1,complete?"Sensor ready":"Saved settings");
  char gainStr[21]; snprintf(gainStr,sizeof(gainStr),"Gain: %.2f",radarGain);
  lcdPrintCentered(2,gainStr);
  lcdPrintCentered(3,"Starting...");
}

void renderWelcomeScreen() {
  if (!lcdPtr) return;
  lcdWriteCachedCenteredRow(0,"");
  lcdWriteCachedCenteredRow(1,"Hello!");
  lcdWriteCachedCenteredRow(2,"Measuring vitals...");
  lcdWriteCachedCenteredRow(3,"Please stay still");
}

void renderGoodbyeScreen() {
  if (!lcdPtr) return;
  lcdWriteCachedCenteredRow(0,"");
  lcdWriteCachedCenteredRow(1,"Goodbye!");
  lcdWriteCachedCenteredRow(2,"Session ended");
  lcdWriteCachedCenteredRow(3,"");
}

void renderIdleScreen(unsigned long now) {
  if (!lcdPtr) return;
  if (safeElapsedMs(now, lastIdleRedrawMs) < 500UL) return;
  lastIdleRedrawMs = now;

  bool scanPhase=((now/500)%2)==0;
  uint8_t row[LCD_COLS];
  lcdBuildBlankRow(row);
  row[0] = customCharsValid ? (uint8_t)(scanPhase ? CHAR_SCAN : CHAR_SCAN2)
                            : (uint8_t)(scanPhase ? '|' : '/');
  lcdCopyTextToRow(row, 1, 19, " Vital Monitor    ");
  lcdWriteCachedRowRaw(0, row);

  int dotCount=(int)((now/400)%4)+1;
  char scanLine[21]; snprintf(scanLine,sizeof(scanLine),"  Scanning%.*s",dotCount,"....");
  lcdWriteCachedTextRow(1, scanLine);

  char envLine[21];
  bool hasAmb=mlxReady&&!isnan(prevAmbient)&&prevAmbient>-40.0f;
  bool hasLux=bh1750Ready;

  if (hasAmb&&hasLux) {
    int ambI=constrain((int)lroundf(prevAmbient),-9,99);
    int luxI=constrain((int)lroundf(luxLevel),0,9999);
    snprintf(envLine,sizeof(envLine),"Room:%d%cC  Lux:%d",ambI,byte(CHAR_DEGREE),luxI);
  } else if (hasAmb) {
    int ambI=constrain((int)lroundf(prevAmbient),-9,99);
    snprintf(envLine,sizeof(envLine),"Room Temp: %d%cC",ambI,byte(CHAR_DEGREE));
  } else if (hasLux) {
    int luxI=constrain((int)lroundf(luxLevel),0,9999);
    snprintf(envLine,sizeof(envLine),"Light: %d lux",luxI);
  } else { snprintf(envLine,sizeof(envLine),"Sensors warming up.."); }
  lcdWriteCachedTextRow(2, envLine);

  int hintIdx=(int)((now/4000UL)%(unsigned long)NUM_HINTS);
  lcdBuildCenteredRow(row, HINTS[hintIdx]);
  lcdWriteCachedRowRaw(3, row);
}

// =========================================================================
// SETUP
// =========================================================================
void setup() {
  Serial.begin(115200); delay(100);
  Serial.println("\n[BOOT] RVital " FW_VERSION); 

  buzzerInit();
  buildSinLUT(); i2cRecover();
  pixel.begin(); pixel.setBrightness(40);
  pixel.setPixelColor(0,pixel.Color(0,0,80)); pixel.show();

#ifdef ESP32
  mmWaveSerial.setRxBufferSize(MMWAVE_RX_BUFFER_SIZE);
#endif
  mmWaveSerial.begin(115200); mmWave.begin(&mmWaveSerial);
  lastPointCloud.targets.reserve(MAX_TARGET_NUM);
  lastTargetInfo.targets.reserve(MAX_TARGET_NUM);

  Wire.setTimeOut(50);
  uint8_t quickScanAddrs[]={0x27,0x3F};
  for (int i=0;i<2;i++) {
    wdtReset();
    Wire.beginTransmission(quickScanAddrs[i]);
    if (Wire.endTransmission()==0) {
      if (lcdObjAllocated&&lcdPtr) { lcdPtr->~LiquidCrystal_I2C();
        lcdPtr=nullptr; lcdObjAllocated=false; }
      lcdPtr=new(lcdObjBuf) LiquidCrystal_I2C(quickScanAddrs[i],20,4);
      lcdObjAllocated=true; lcdPtr->init(); lcdPtr->backlight();
      invalidateLcdRowCache();
      customCharsValid=false; lcdCreateChars();
      lcdAddr=quickScanAddrs[i]; lcdConnected=true; break;
    }
  }
  Wire.setTimeOut(100);

  int initStep=0; const int totalSteps=6;

  if (lcdConnected) renderInitScreen(++initStep,totalSteps,"Buzzer");
  pixel.setPixelColor(0,pixel.Color(40,40,40)); pixel.show();
  Serial.println("[SENSOR] Buzzer: OK"); wdtReset();

  if (lcdConnected) renderInitScreen(++initStep,totalSteps,"Temperature");
  pixel.setPixelColor(0,pixel.Color(40,0,40)); pixel.show();
  mlxReady=mlx.begin();
  Serial.printf("[SENSOR] MLX90614: %s\n",mlxReady?"OK":"Not found");
  delay(100); wdtReset();

  if (lcdConnected) renderInitScreen(++initStep,totalSteps,"Light");
  pixel.setPixelColor(0,pixel.Color(40,40,0));
  pixel.show();
  bh1750Ready=false; bh1750Addr=0;
  tryInitBH1750();
  if (bh1750Ready) { Serial.printf("[SENSOR] BH1750: OK at 0x%02X\n",bh1750Addr);
    delay(180); i2cSafeReadLux(millis());
  }
  else Serial.println("[SENSOR] BH1750: Not found");
  wdtReset();

  if (lcdConnected) renderInitScreen(++initStep,totalSteps,"Display");
  else { pixel.setPixelColor(0,pixel.Color(0,60,60));
    pixel.show(); lcdConnected=scanForLCD(); initStep++;
  }
  Serial.printf("[SENSOR] LCD: %s\n",lcdConnected?"OK":"Not found"); wdtReset();

  if (lcdConnected) renderInitScreen(++initStep,totalSteps,"Settings");
  pixel.setPixelColor(0,pixel.Color(0,40,40)); pixel.show();

  {
    if (!prefs.begin("rvital", false)) {
      Serial.println("[NVS] ERROR: Could not open namespace for migration");
    } else {
      int nvsVer = prefs.getInt("nvsVer", 0);
      if (nvsVer < 2) {
        prefs.clear();
        prefs.putInt("nvsVer", 2);
        Serial.println("[NVS] Migrated v2: cleared stale settings");
      }
      prefs.end();
    }
  }

  if (!prefs.begin("rvital",true)) {
    Serial.println("[NVS] ERROR: Could not open namespace for read; using defaults");
    if (!buzzerIsBusy()) buzzerPlay(BUZZ_SENSOR_ERROR);
  } else {
    float savedGain=prefs.getFloat("gain",0);
    if (savedGain>=MIN_GAIN&&savedGain<=MAX_GAIN) { radarGain=savedGain; lastSavedGain=savedGain;
      nvsGainValid=true; nvsRestoredGain=savedGain; Serial.printf("[SETTINGS] Gain: %.3f\n",radarGain); }
    float savedKfHR=prefs.getFloat("kfHR",0);
    if (savedKfHR>=HR_MIN&&savedKfHR<=HR_MAX) { kfHR_x=savedKfHR; lastSavedHR=savedKfHR; }
    float savedKfRR=prefs.getFloat("kfRR",0);
    if (savedKfRR>=RR_MIN&&savedKfRR<=RR_MAX) { kfRR_x=savedKfRR; lastSavedRR=savedKfRR; }
    buzzerEnabled=prefs.getBool("buzzer",true); buzzerSilentMode=prefs.getBool("silent",false);
    prefs.end();
  }
  wdtReset();

  if (buzzerEnabled && !buzzerSilentMode) {
      buzzerOn(); delay(50); buzzerOff();
  }

  if (lcdConnected) renderInitScreen(++initStep,totalSteps,"Radar");
  pixel.setPixelColor(0,pixel.Color(0,60,0)); pixel.show();

  { int safe=0;
    while (mmWaveSerial.available()&&safe++<1024) mmWaveSerial.read(); }
  if (mlxReady) { float amb=mlx.readAmbientTempC(); if (isfinite(amb)) { prevAmbient=amb; ambientFastEma=amb; ambientSlowEma=amb; ambientDrift=0.0f; } }
  wdtReset();

  if (lcdConnected) {
    lcdSmoothTransition(); lcdPtr->clear();
    lcdPrintCentered(0,"RVital " FW_VERSION);  
    lcdPrintCentered(1,"Signal + Distance");
    char statusLine[21];
    snprintf(statusLine,sizeof(statusLine),"T:%s L:%s B:%s",mlxReady?"OK":"--",bh1750Ready?"OK":"--",buzzerEnabled?"ON":"--");
    lcdPrintCentered(2,statusLine); lcdPrintCentered(3,"Starting...");
    for (int i=0;i<12;i++) { delay(100); wdtReset(); }
    lcdPtr->clear(); prevDispState=DISP_NONE; lastDisplay=0;
  }
  buzzerPlay(BUZZ_STARTUP);

  { esp_task_wdt_config_t wdt_config={}; wdt_config.timeout_ms=8000;
    wdt_config.idle_core_mask=0; wdt_config.trigger_panic=true;
    esp_err_t err=esp_task_wdt_init(&wdt_config);
    if (err==ESP_ERR_INVALID_STATE) err=esp_task_wdt_reconfigure(&wdt_config);
    if (err==ESP_OK) { err=esp_task_wdt_add(NULL);
      if (err==ESP_OK||err==ESP_ERR_INVALID_ARG) wdtActive=true; }
  }
  calibStartMs=millis(); radarWatchdogStartMs = calibStartMs; lastValidRateMs=0; sessionReset(); presenceState = PRESENCE_ABSENT; presenceStateSinceMs = millis();
  Serial.println("[BOOT] Setup complete");
  Serial.println("=========================================================");
}

// =========================================================================
// MAIN LOOP
// =========================================================================
void loop() {
  wdtReset();
  unsigned long now=millis();
  buzzerUpdate();

  { static unsigned long lastDecayCheckMs=0;
    if (now-lastDecayCheckMs>=1000UL) { lastDecayCheckMs=now; decayHRState(); } }

  if (buzzPinState&&currentBuzzEvent==BUZZ_NONE) { Serial.println("[BUZZER] Safety: forcing off");
    buzzerForceOff(); }
  if (persistedMotion&&(now-lastMotionDetectedMs>=MOTION_PERSIST_MS)) persistedMotion=false;
  bool inMotion=persistedMotion;

  if (!mlxReady&&now-lastMlxRetry>MLX_RETRY_INTERVAL) { mlxReady=mlx.begin(); lastMlxRetry=now; }
  rawHRValid = false;
  rawRRValid = false;
  hrUpdatedThisCycle = false;
  hrUpdateSourceThisCycle = HR_PATH_NONE;
  hrConfidenceSourceThisCycle = HR_CONF_NONE;
  dspRanThisFrame = false;
  rrPhaseBackedUpdateThisCycle = false;
  hrGateReason = HR_GATE_NO_AUTO;
  rrGateReason = RR_GATE_NO_AUTO;
  bool newData=mmWave.update(5);
  bool badRadarPacketThisFrame = false;
  bool goodRadarPacketThisFrame = false;
  bool freshDistanceSampleThisFrame = false;
  bool staticReflectorUpdatedThisFrame = false;
  bool livePhaseThisFrame = false;
  nearFieldReflectorSuspect = false;
  agcFloorSuspect = false;
  hrAnchorDriftSuspect = false;
  experimentalProfileEnabled = false;

  // Calibration timeout
  if (!calibrationDone&&now-calibStartMs>=CALIB_TIMEOUT_MS) {
    radarGain=nvsGainValid?nvsRestoredGain:1.0f;
    rollingEnergy=TARGET_ENERGY; motionLP=TARGET_ENERGY;
    calibrationDone=true; presentVotes=0; absentVotes=0; presenceState = PRESENCE_ABSENT; presenceStateSinceMs = now;
    startCalibResult(false);
  }

  // Display state timeouts
  if (dispState==DISP_CALIB_RESULT&&now-calibResultStartMs>=CALIB_RESULT_DISPLAY_MS) { dispState=DISP_NONE; lastDisplay=0; }
  if (dispState==DISP_WELCOME&&now-welcomeStartMs>=WELCOME_DISPLAY_MS) {
    dispState=DISP_VITALS; lastDisplay=0;
  }
  if (dispState==DISP_GOODBYE&&now-goodbyeStartMs>=GOODBYE_DISPLAY_MS) {
    dispState=DISP_IDLE; lastDisplay=0;
    lastIdleRedrawMs = 0;
    if (!humanDetected) { presentVotes = 0; absentVotes = 0; }
  }

  // ==== RADAR DATA PROCESSING ====
  if (newData) {
    lastRadarPacketMs = now;
    rawHRValid=false; rawRRValid=false;
    bool hrOk=mmWave.getHeartRate(rawHR);
    bool rrOk=mmWave.getBreathRate(rawRR);
    bool radarIsPresentInstant=mmWave.isHumanDetected();
    radarIsPresentRaw = radarIsPresentInstant;
    bool distOk=mmWave.getDistance(radarDistance);
    v13_pollSpatialFrames(now);
    if (distOk && radarDistance > 1.0f) {
      radarDistance = v13_refinedDistanceCm(radarDistance, distOk);
      if (radarDistance > MODULE_MAX_DISTANCE_CM) {
        radarDistance = 0.0f;
        distOk = false;
        radarIsPresentInstant = false;
        radarIsPresentRaw = false;
        numDetectedTargets = 0;
      }
    }

    if (hrOk&&rawHR>=HR_MIN&&rawHR<=HR_MAX) rawHRValid=true;
    if (rrOk&&rawRR>=RR_MIN&&rawRR<=RR_MAX) rawRRValid=true;
    if (!hrOk) rawHR=0.0f; if (!rrOk) rawRR=0.0f;
    // v13.7 P1: Anchor-aware RR harmonic guard. Only correct if rawRR is in the 18-26 band AND near 1.6x the current anchor.
    bool rrInSuspiciousBand = rawRRValid && isfinite(rawRR) && rawRR >= 18.0f && rawRR <= 26.0f;
    bool rrNearAnchorHarmonic = (smoothRR >= 8.0f && smoothRR <= 17.0f) && (fabsf(rawRR - smoothRR * RR_MODULE_RAW_HARMONIC_RATIO) <= 4.5f);
    rawRRLikelyHarmonic = rrInSuspiciousBand && rrNearAnchorHarmonic;
    rawRREffective = rawRRValid ? (rawRRLikelyHarmonic ? (rawRR / RR_MODULE_RAW_HARMONIC_RATIO) : rawRR) : 0.0f;

    if (rawRRValid) {
      if (isfinite(rrSeedLastRaw) && fabsf(rawRR - rrSeedLastRaw) <= RR_SEED_CONSISTENT_MAX_DIFF_BPM) {
        rrSeedConsistentCount = (uint8_t)min(255, (int)rrSeedConsistentCount + 1);
      } else {
        rrSeedConsistentCount = 1;
      }
      rrSeedLastRaw = rawRR;
    } else {
      rrSeedConsistentCount = 0;
      rrSeedLastRaw = NAN;
    }

    if (radarIsPresentInstant) {
      radarPresentScore = min((int)RAW_PRESENT_VOTES, (int)radarPresentScore + v13_presenceBoostPresent());
      radarAbsentScore = 0;
    } else {
      radarAbsentScore = min((int)RAW_ABSENT_VOTES, (int)radarAbsentScore + v13_presenceBoostAbsent());
      radarPresentScore = 0;
    }
    if (!radarIsPresent && radarPresentScore >= RAW_PRESENT_VOTES) {
      radarIsPresent = true;
      if (safeElapsedMs(now, lastPresenceDebugLogMs) >= 250UL) {
        lastPresenceDebugLogMs = now;
        Serial.printf("[RADAR_PRES] raw_confirm present | inst=%d pScore=%u aScore=%u human=%d pv=%d av=%d dist=%.2f\n",
                      (int)radarIsPresentInstant, (unsigned)radarPresentScore, (unsigned)radarAbsentScore,
                      (int)humanDetected, presentVotes, absentVotes, lastGoodDistance);
      }
    } else if (radarIsPresent && radarAbsentScore >= RAW_ABSENT_VOTES) {
      radarIsPresent = false;
      if (safeElapsedMs(now, lastPresenceDebugLogMs) >= 250UL) {
        lastPresenceDebugLogMs = now;
        Serial.printf("[RADAR_PRES] raw_confirm absent | inst=%d pScore=%u aScore=%u human=%d pv=%d av=%d dist=%.2f\n",
                      (int)radarIsPresentInstant, (unsigned)radarPresentScore, (unsigned)radarAbsentScore,
                      (int)humanDetected, presentVotes, absentVotes, lastGoodDistance);
      }
    }

    if (radarIsPresent) lastPresenceUpdateMs=now;
    if (distOk&&radarDistance>1.0f) {
      lastDistanceUpdateMs=now;
      lastGoodDistance=radarDistance;
      freshDistanceSampleThisFrame = true;
    } else {
      radarDistance = 0.0f;
    }

    bool badRadarThisFrame = (!hrOk && !rrOk && !distOk);

    if (rawHRValid && !wasMotion && motionCooldown == 0) { 
      rawHRWin[rawHRWinIdx]=rawHR; 
      rawHRWinIdx=(rawHRWinIdx+1)%RAW_STAB_WIN; 
      if (rawHRWinCount<RAW_STAB_WIN) rawHRWinCount++; 
      latchedRawHR=rawHR; latchedRawHRMs=now;
      if (rawHR_consecutive_valid < 255) rawHR_consecutive_valid++;
      if (directRawHR_consecWindows < 255) directRawHR_consecWindows++;
    } else {
      rawHR_consecutive_valid = 0;
      directRawHR_consecWindows = 0;
    }
    // v11.6.10: retain hardened cold-start HR seeding while relaxing downstream publish gates
    float rawHRStab = rawHRStability();
    float rawHRSeedMinStability = v13_rawHRInRestingZone(rawHR) ? (RAW_HR_SEED_MIN_STABILITY * 0.70f) : RAW_HR_SEED_MIN_STABILITY;
    bool seedCandidateAvailable = isfinite(candidateHR) && candidateHR > 0.0f;
    bool seedCandidateAgrees = seedCandidateAvailable && (fabsf(rawHR - candidateHR) <= RAW_HR_SEED_MAX_DIFF_WITH_CAND);
    unsigned long rawHRSeedCooldownNeed = sessionFirstSeedDone ? RAW_HR_SEED_COOLDOWN_MS : RAW_HR_FIRST_SEED_COOLDOWN_MS;
    bool rawHRSeedCooldownOk = (rawHR_last_seed_ms == 0UL) ||
                               (safeElapsedMs(now, rawHR_last_seed_ms) >= rawHRSeedCooldownNeed);
    // v13.7 P1: Force first session seed to wait for a phase-derived candidate (autoValid) to prevent bias poisoning.
    bool allowBlindRawSeed = sessionFirstSeedDone && !seedCandidateAvailable;
    if (rawHR_consecutive_valid >= RAW_HR_SEED_MIN_COUNT &&
        calibrationDone &&
        !wasMotion &&
        !isHRFresh() &&
        rawHRSeedCooldownOk &&
        isfinite(rawHR) &&
        rawHR >= HR_MIN && rawHR <= HR_MAX &&
        rawHRStab >= rawHRSeedMinStability &&
        (humanDetected || radarIsPresent) &&
        (seedCandidateAgrees || allowBlindRawSeed)) {
      float seededHR = constrain(rawHR - (allowBlindRawSeed ? CHIP_HR_BIAS_CORRECTION_BPM : 0.0f), HR_MIN, HR_MAX);
      float rawWeight = sessionFirstSeedDone ? 0.40f : 0.80f;
      float kfWeight = 1.0f - rawWeight;
      float seededBlend = (isfinite(kfHR_x) && kfHR_x >= HR_MIN && kfHR_x <= HR_MAX)
                        ? (rawWeight * seededHR + kfWeight * kfHR_x)
                        : seededHR;
      kfHR_x = seededBlend;
      kfHR_P = 4.0f;
      smoothHR = safe_float(kfHR_x);
      lastStableHR = smoothHR;
      lastHRUpdateMs = now;
      trackingHR = smoothHR;
      lastTrackingHRMs = now;
      hrState = max(hrState, 1);
      hrConfidence = fmaxf(hrConfidence, 0.20f);
      latchedRawHR = seededHR;
      latchedRawHRMs = now;
      rawHR_last_seed_ms = now;
      sessionFirstSeedDone = true;
      Serial.printf("[HR_SEED] raw=%.1f stab=%.2f count=%d cand=%.1f agree=%d seed=%.1f\n",
                    rawHR, rawHRStab, rawHR_consecutive_valid,
                    seedCandidateAvailable ? candidateHR : 0.0f,
                    seedCandidateAgrees ? 1 : 0,
                    smoothHR);
      rawHR_consecutive_valid = 0;
    }

    float totalPhase,breath_phase,heart_phase;
    bool phaseDataValid=mmWave.getHeartBreathPhases(totalPhase,breath_phase,heart_phase);
    if (phaseDataValid) {
        if (!isfinite(totalPhase) || !isfinite(breath_phase) || !isfinite(heart_phase)) {
            phaseDataValid = false;
            badRadarThisFrame = true;
        }
    }
    if (phaseDataValid) lastPhaseDataMs = now;
    badRadarPacketThisFrame = (!phaseDataValid && !hrOk && !rrOk && !distOk);
    goodRadarPacketThisFrame = (phaseDataValid || hrOk || rrOk || distOk);

    if (phaseDataValid) {
      if (firstPhaseSample) {
        prevHeartPhase = heart_phase;
        prevBreathPhase = breath_phase;
        prevStableHeartPhase = heart_phase;
        prevStableBreathPhase = breath_phase;
        prevTotalPhase = totalPhase;
        heartOffset = 0.0f;
        breathOffset = 0.0f;
        clutterHeart = heart_phase;
        clutterBreath = breath_phase;
        prevHeartDiff = 0.0f;
        prevBreathDiff = 0.0f;
        phaseDelta = 0.0f;
        firstPhaseSample = false;
      } else {
        float rawPD=totalPhase-prevTotalPhase;
        if (rawPD>PI) rawPD-=2.0f*PI; if (rawPD<-PI) rawPD+=2.0f*PI;
        phaseDelta=fabsf(rawPD); prevTotalPhase=totalPhase;
      }

      float uh = unwrap(heart_phase, prevHeartPhase, heartOffset);
      if (fabsf(heartOffset) > 100.0f) {
          float delta = roundf(heartOffset / (2.0f * PI)) * (2.0f * PI);
          heartOffset -= delta;
          prevStableHeartPhase -= delta;
          clutterHeart -= delta;
          uh -= delta;
      }

      float ub = unwrap(breath_phase, prevBreathPhase, breathOffset);
      if (fabsf(breathOffset) > 100.0f) {
          float delta = roundf(breathOffset / (2.0f * PI)) * (2.0f * PI);
          breathOffset -= delta;
          prevStableBreathPhase -= delta;
          clutterBreath -= delta;
          ub -= delta;
      }

      uh=stabilizePhase(uh,prevStableHeartPhase);
      ub=stabilizePhase(ub,prevStableBreathPhase);
      float ch=removeClutter(uh,clutterHeart);
      float cb=removeClutter(ub,clutterBreath);

      {
        sgBufH[sgIdx]=ch;
        sgBufB[sgIdx]=cb; sgIdx=(sgIdx+1)%SG_WIN;
        float smoothedH,smoothedB;

        if (sgWarmup<SG_WIN) { sgWarmup++; smoothedH=ch; smoothedB=cb; }
        else { smoothedH=savitzkyGolay(sgBufH); smoothedB=savitzkyGolay(sgBufB); }

        float dh=phaseDiffFn(smoothedH,prevHeartDiff);
        float db=phaseDiffFn(smoothedB,prevBreathDiff);

        medBufH[medIdx]=dh; medBufB[medIdx]=db; medIdx=(medIdx+1)%MED_WIN;
        if (medWarmup >= MED_WIN) {
            dh=medianOf5(medBufH); 
            db=medianOf5(medBufB);
        } else {
            medWarmup++;
        }

        float dhSigned = dh;
        float dhRectified = fabsf(dhSigned);  // v11.6: remove alternating-sign beat cancellation before HR spectral extraction

        dh = v13_processHeartPhase(dhRectified, pqiHeart);
        db = v13_processBreathPhase(db, pqiBreath);

        float preGainEnergy = dh*dh + db*db;
        dh *= radarGain; db *= radarGain;

        float rawEnergy = dh*dh + db*db;
        nearFieldReflectorSuspect = isfinite(radarDistance) && radarDistance > 0.0f && radarDistance < 10.0f;
        float energyFloor = fmaxf(rollingEnergy * 0.01f, MIN_PHASE_ENERGY);
        float phaseMotionNorm = constrain((phaseDelta - 0.008f) / 0.08f, 0.0f, 1.0f);
        float phaseEnergyNorm = constrain((rawEnergy - energyFloor) / fmaxf(energyFloor * 4.0f, 1e-6f), 0.0f, 1.0f);
        float instantPhaseLiveness = phaseDataValid ? (0.20f + 0.45f * phaseMotionNorm + 0.35f * phaseEnergyNorm) : 0.0f;
        phaseLivenessScore = constrain(0.88f * phaseLivenessScore + 0.12f * instantPhaseLiveness, 0.0f, 1.0f);
        livePhaseThisFrame = phaseDataValid && (phaseMotionNorm > 0.10f) && (phaseEnergyNorm > 0.08f);
        if (livePhaseThisFrame) {
          lastLivePhaseMs = now;
          // v13.7 P3: Clutter experiment - dynamic alpha gated on valid phase
          if (clutterWarmupCount < 80) {
            clutterWarmupCount++;
            currentClutterAlpha = 0.08f;
          } else {
            phaseWarmupComplete = true;
            currentClutterAlpha = 0.005f;
          }
        }

        bool livePhaseEvidencePreDSP = (lastLivePhaseMs > 0) &&
                                      (safeElapsedMs(now, lastLivePhaseMs) < LIVE_PHASE_HOLD_MS) &&
                                      (lastPhaseDataMs > 0) &&
                                      (safeElapsedMs(now, lastPhaseDataMs) < LIVE_PHASE_PACKET_MAX_MS) &&
                                      (phaseLivenessScore >= 0.12f);
        if (freshDistanceSampleThisFrame) {
          currentStaticReflector = applyStaticReflectorPenalty(true, livePhaseEvidencePreDSP);
          staticReflectorUpdatedThisFrame = true;
        }

        if (rawEnergy < energyFloor && calibrationDone) {
            skipDSP_consecMisses++;
            bool agcAtFloorNow = isfinite(radarGain) && radarGain <= (MIN_GAIN + 0.01f);
            if (agcAtFloorNow) {
              if (agcFloorRecoveryStartMs == 0UL) agcFloorRecoveryStartMs = now;
              if (safeElapsedMs(now, agcFloorRecoveryStartMs) >= AGC_SKIP_RECOVERY_MS) {
                radarGain = 1.0f;
                rollingEnergy = TARGET_ENERGY;
                motionLP = TARGET_ENERGY;
                recalFrames = 0;
                skipDSP_consecMisses = 0;
                agcFloorRecoveryStartMs = 0UL;
              }
            } else {
              agcFloorRecoveryStartMs = 0UL;
            }
            if (skipDSP_consecMisses >= SKIP_DSP_MISS_THRESH) {
              pqiHeart = 0.0f;
              pqiBreath = 0.0f;
              skipDSP = true;
              dspTask = 0;
              lastAutoValid = false;
              lastRRValid = false;

              heartBuf[bufIndex] = 0.0f;
              breathBuf[bufIndex] = 0.0f;
              fusedBuf[bufIndex] = 0.0f;
              phaseTimes[bufIndex] = now;

              bufIndex = (bufIndex + 1) % BUF_SIZE;
              if (bufCount < BUF_SIZE) bufCount++;
            } else {
              skipDSP = false;
            }
        } else {
            skipDSP_consecMisses = 0;
            skipDSP = false;
            agcFloorRecoveryStartMs = 0UL;
        }

        if (!skipDSP) {
            float energy=dh*dh+db*db;

            if (!calibrationDone) {
              baselineEnergy += preGainEnergy;
              calibrationCount++;

              if (calibrationCount>=CALIBRATION_FRAMES) {
                baselineEnergy/=calibrationCount; baselineEnergy=fmaxf(baselineEnergy,1e-6f);
                radarGain=constrain(TARGET_ENERGY/baselineEnergy,MIN_GAIN,MAX_GAIN);
                rollingEnergy=TARGET_ENERGY; motionLP=TARGET_ENERGY;
                calibrationDone=true;
                presentVotes=0; absentVotes=0; presenceState = PRESENCE_ABSENT; presenceStateSinceMs = now; startCalibResult(true);
              }
            } else {
              bool roughMotionBase=(rawEnergy>fmaxf(motionLP*8.0f,1e-6f));
              bool roughMotion=roughMotionBase || dopplerMotionDetected;
              if (!roughMotion && !nearFieldReflectorSuspect) rollingEnergy=0.99f*rollingEnergy+0.01f*rawEnergy;

              if (!nearFieldReflectorSuspect &&
                  (rollingEnergy>TARGET_ENERGY*RECAL_RATIO_HI||rollingEnergy<TARGET_ENERGY*RECAL_RATIO_LO)) {
                recalFrames++;
                if (recalFrames>RECAL_HOLD_FRAMES) {
                  rollingEnergy=fmaxf(rollingEnergy,1e-6f);
                  radarGain=constrain(TARGET_ENERGY/rollingEnergy,MIN_GAIN,MAX_GAIN);
                  rollingEnergy=0.5f*rollingEnergy+0.5f*TARGET_ENERGY; recalFrames=0;
                }
              } else { if (recalFrames>0) recalFrames--; }

              float dh_preRLS = dh; 

              float safeGain = (isfinite(radarGain) && radarGain >= MIN_GAIN) ? radarGain : MIN_GAIN;
              float db_norm = db / safeGain;
              float dh_norm = dh / safeGain;

              bool rlsFreeze = roughMotion || (fabsf(db_norm) >= 0.3f);
              float e_norm = rlsUpdate(db_norm, dh_norm, rlsFreeze);

              dh = rlsWarmedUp ? (e_norm * safeGain) : dh_preRLS;

              float dh_postRLS = dh;

              bool localMotion=v13_motionDetected(dhSigned,db,roughMotion);
              if (localMotion) {
                bool physicalMotionLikely = roughMotion || dopplerMotionDetected || multiTargetDetected || (motionSpikeCount >= 2) || (currentDistStdDev > 0.25f);
                if (physicalMotionLikely) lastMotionDetectedMs=now;
                persistedMotion=true;
                inMotion=true;
    #if BUZZER_MOTION_WARNING_ENABLED
                if (now-lastMotionBeepMs>DEBOUNCE_MOTION&&humanDetected&&!buzzerIsBusy()) { buzzerPlay(BUZZ_MOTION_WARNING);
                  lastMotionBeepMs=now; }
    #endif
              }

              dbEnvLP = 0.95f * dbEnvLP + 0.05f * fabsf(db);
              float breathGate = constrain(fabsf(db) / fmaxf(dbEnvLP * 2.0f, 1e-6f), 0.0f, 1.0f);
              float dhGated=dh*(0.5f+0.5f*breathGate);

              diagAvgE_Pre  = 0.99f * diagAvgE_Pre  + 0.01f * (dh_preRLS  * dh_preRLS);
              diagAvgE_RLS  = 0.99f * diagAvgE_RLS  + 0.01f * (dh_postRLS * dh_postRLS);
              diagAvgE_Gate = 0.99f * diagAvgE_Gate + 0.01f * (dhGated    * dhGated);

              float w1=pqiHeart,w2=pqiBreath*0.7f,w3=0.3f,weightSum=w1+w2+w3;
              float fusedSig=(weightSum>0.01f)?(w1*dhGated+w2*db+w3*(dhGated+db)*0.5f)/weightSum:dhGated;

              heartBuf[bufIndex]=dhGated; fusedBuf[bufIndex]=fusedSig;
              breathBuf[bufIndex]=db; phaseTimes[bufIndex]=now;
              bufIndex=(bufIndex+1)%BUF_SIZE;
              if (bufCount<BUF_SIZE) bufCount++;

              float fs=10.0f;
              if (bufCount>1) {
                unsigned long span=phaseTimes[(bufIndex-1+BUF_SIZE)%BUF_SIZE]-phaseTimes[(bufIndex-bufCount+BUF_SIZE)%BUF_SIZE];
                if (span>0) fs=(bufCount-1)*1000.0f/span;
              }
              if (fs>5.0f&&fs<20.0f) lastCalculatedFs=fs;

              int detectWindow=0;
              if (bufCount>=WIN_FULL) detectWindow=3;
              else if (bufCount>=96) detectWindow=2;
              else if (bufCount>=WIN_FAST) detectWindow=1;
              if (detectWindow==0) dspTask=0;

              if (detectWindow>0) {
                if (inMotion&&!wasMotion) {
                  motionCooldown=15;
                  rejectionCount=0; rawHRWinIdx=0; rawHRWinCount=0; memset(rawHRWin,0,sizeof(rawHRWin));
                  ghostSuspect=false; rhcSuspect=false; ghostSuspectCounter=0;
                  rlsReset(); 
                  bpResetHeart(); bpResetBreath();
                  memset(sgBufH,0,sizeof(sgBufH)); memset(sgBufB,0,sizeof(sgBufB));
                  sgIdx=0; sgWarmup=0;
                  memset(medBufH,0,sizeof(medBufH)); memset(medBufB,0,sizeof(medBufB));
                  medIdx=0; medWarmup=0;
                  firstPhaseSample=true;
                  clutterHeart=0.0f; clutterBreath=0.0f;
                  heartOffset=0.0f; breathOffset=0.0f;
                  prevHeartPhase=0.0f; prevBreathPhase=0.0f;
                  prevStableHeartPhase=0.0f; prevStableBreathPhase=0.0f;
                  phaseDelta=0.0f; dbEnvLP=0.0f;
                  skipDSP_consecMisses = 0;
                  bufIndex = 0;
                  bufCount = 0;
                  clutterWarmupCount = 0; phaseWarmupComplete = false; currentClutterAlpha = 0.08f; // v13.7: Reset on motion

                  if (rawHRValid) { kfHR_x=rawHR; kfHR_P=4.0f; latchedRawHR=rawHR; latchedRawHRMs=now; }
                  else if (latchedRawHRValidAt(now)) { kfHR_x=latchedRawHR; kfHR_P=4.0f; }
                  lastStableHR=0;
                }

                if (inMotion) { 
                    dspTask=0;
                    pqiHeart *= 0.90f; 
                    pqiBreath *= 0.90f;
                }
                else {
                  bool latchedValidNow = latchedRawHRValidAt(now);
                  float hrAnchor = isTrustedPhaseHRFreshAt(now) ? trustedPhaseHR :
                                   (isTrackingHRFreshAt(now) ? trackingHR :
                                   (latchedValidNow ? latchedRawHR :
                                   ((lastStableHR > 0) ? lastStableHR : 0.0f)));
                  float bandHalf = (rawHRValid && isfinite(rawHR) && rawHR >= HR_MIN && rawHR <= HR_MAX)
                                   ? HR_BAND_HALF_RAW : HR_BAND_HALF;
                  float effectiveAnchor = hrAnchor;
                  if (!(effectiveAnchor > 0.0f) && rawHRValid && isfinite(rawHR) && rawHR >= HR_MIN && rawHR <= HR_MAX) {
                    effectiveAnchor = rawHR;
                  }
                  float hrMinAdaptive=(effectiveAnchor>0)?fmaxf(HR_MIN,effectiveAnchor-bandHalf):HR_MIN;
                  float hrMaxAdaptive=(effectiveAnchor>0)?fminf(HR_MAX,effectiveAnchor+bandHalf):HR_MAX;
                  lastLoggedHrBandMin = hrMinAdaptive;
                  lastLoggedHrBandMax = hrMaxAdaptive;

                  dspRanThisFrame = true;
                  if (dspTask==0) {
                    // v11.6.14: reset per-window funnel telemetry
                    hrArbiterCorrectedThisWindow    = false;
                    hrRejectPhaseRejectedThisWindow = false;
                    hrCoherenceRejectedThisWindow   = false;
                    hrGateReason                    = HR_GATE_NO_AUTO; // v13.7 P0: Truthfulness reset
                    rrGateReason                    = RR_GATE_NO_AUTO; // v13.7 P0: Truthfulness reset
                    hrRawSourceThisWindow           = rawHRValid ? 1 :
                                                      (latchedRawHRValidAt(now) ? 2 : 0);
                    hrPreRejectPhase                = 0.0f;
                    hrPostRejectPhase               = 0.0f;
                    hrPostBlend                     = 0.0f;
                    hrPostCoherence                 = 0.0f;
                    hrFinalPublishCandidate         = 0.0f;
                    hrArbiterAnchorUsed             = false;
                    hrArbiterAnchorValue            = 0.0f;
                    hrRejectPhaseAnchorUsed         = 0;
                    hrRejectPhaseAnchorValue        = 0.0f;
                    hrRawLooksLikeHalfRateLogged    = false;
                    hrTrustedAnchorValue            = 0.0f;
                    hrTrustedPhaseAnchorLogged      = NAN;
                    hrAnchorSourceLogged            = HR_ANCHOR_NONE;
                    hrAnchorErrBpmLogged            = NAN;
                    hrRawHighBiasSuspectLogged      = false;
                    rrAnchorFreshLogged             = false;
                    hrBypassPqiOk                   = false;
                    hrBypassConfOk                  = false;
                    hrBypassGateOk                  = false;
                    hrBypassActive                  = false;
                    hrGraceEligible                 = false;
                    hrGraceActive                   = false;
                    hrUpdatedThisCycle              = false;
                    hrUpdateSourceThisCycle         = 0;
                    bufCount_snap=bufCount;
                    fs_snap = (fs > 5.0f && fs < 20.0f) ? fs : lastCalculatedFs; 
                    detectWindow_snap = detectWindow;

                    for (int i=0;i<bufCount;i++) {
                      wdtResetEvery(i, 64);
                      int idx=(bufIndex-bufCount+i+BUF_SIZE)%BUF_SIZE;
                      linearHeart_snap[i]=heartBuf[idx];
                      linearBreath_snap[i]=breathBuf[idx];
                      linearFused_snap[i]=fusedBuf[idx];
                    }

                    pqiHeart = computePQI(linearHeart_snap, bufCount_snap, fs_snap, HR_MIN, HR_MAX);
                    pqiBreath = computePQI(linearBreath_snap, bufCount_snap, fs_snap, RR_MIN, RR_MAX); 
                    hrZcBpmLogged = 0.0f;
                    hrZcConfLogged = 0.0f;
                    hrSpecBpmLogged = 0.0f;
                    hrSpecMagLogged = 0.0f;
                    hrTripleAgreeLogged = false;

                    {
                      float w1f = pqiHeart, w2f = pqiBreath * 0.7f, w3f = 0.3f;
                      float wsf = w1f + w2f + w3f;
                      if (wsf > 0.01f) {
                        for (int li = 0; li < bufCount_snap; li++) {
                          wdtResetEvery(li, 64);
                          linearFused_snap[li] = (w1f * linearHeart_snap[li] +
                                                  w2f * linearBreath_snap[li] +
                                                  w3f * (linearHeart_snap[li] + linearBreath_snap[li]) * 0.5f) / wsf;
                        }
                      }
                    }

                    float hrZc = 0.0f, hrZcConf = 0.0f;
                    bool hrZcValid = false;
                    if (fs_snap >= 8.0f) {
                      hrZcValid = detectRateZeroCrossing(linearHeart_snap, bufCount_snap, fs_snap,
                                                         hrMinAdaptive, hrMaxAdaptive, hrZc, hrZcConf);
                    }
                    if (hrZcValid) { hrZcBpmLogged = hrZc; hrZcConfLogged = hrZcConf; }

                    float hrSpecDiag = 0.0f, hrSpecMagDiag = 0.0f;
                    bool hrSpecDiagValid = false;
                    if (fs_snap >= 8.0f) {
                      detectSpectral(linearFused_snap, bufCount_snap, fs_snap,
                                     hrMinAdaptive/60.0f, hrMaxAdaptive/60.0f,
                                     hrSpecDiag, hrSpecMagDiag);
                      hrSpecDiagValid = isfinite(hrSpecDiag) && (hrSpecDiag >= hrMinAdaptive) &&
                                        (hrSpecDiag <= hrMaxAdaptive) && isfinite(hrSpecMagDiag) &&
                                        (hrSpecMagDiag > 0.0f);
                    }
                    if (hrSpecDiagValid) { hrSpecBpmLogged = hrSpecDiag; hrSpecMagLogged = hrSpecMagDiag; }

                    float hrAuto=0,confHR=0;
                    float confThreshold = (detectWindow_snap == 1) ? HR_CONF_THRESHOLD_FAST : HR_CONF_THRESHOLD;

                    bool autoValid = false;
                    if (fs_snap >= 8.0f) {
                        autoValid = detectRate(linearFused_snap,bufCount_snap,fs_snap,
                                              hrMinAdaptive,hrMaxAdaptive,hrAuto,confHR,confThreshold);
                    }
                    if (autoValid && hrZcValid && hrSpecDiagValid) {
                      hrTripleAgreeLogged = (fabsf(hrAuto - hrZc) <= HR_TRIPLE_AGREE_BPM) &&
                                            (fabsf(hrAuto - hrSpecDiag) <= HR_TRIPLE_AGREE_BPM) &&
                                            (fabsf(hrZc - hrSpecDiag) <= HR_TRIPLE_AGREE_BPM);
                    }
                    bool hrTwoOfThreeAgree =
                      (hrZcValid && fabsf(hrAuto - hrZc) <= HR_TRIPLE_AGREE_BPM) ||
                      (hrSpecDiagValid && fabsf(hrAuto - hrSpecDiag) <= HR_TRIPLE_AGREE_BPM) ||
                      (hrZcValid && hrSpecDiagValid && fabsf(hrZc - hrSpecDiag) <= HR_TRIPLE_AGREE_BPM);

                    float heartPqiGate = currentHeartPQIGate();
                    hrGatePqiUsed = heartPqiGate;
                    rrGatePqiUsed = RR_PQI_MIN_ACCEPT;
                    hrHarmonicAmbiguous = false;
                    hrRawAgreementGood = false;
                    lastHRAgreementErr = NAN;
                    candidateHR = autoValid ? hrAuto : 0.0f;
                    candidateHRConf = autoValid ? confHR : 0.0f;
                    if (autoValid) { lastCandidateHRMs = now; pqiHeartAtCandidateTime = pqiHeart; }
                    if (!autoValid) hrGateReason = inMotion ? HR_GATE_MOTION : HR_GATE_NO_AUTO;
                    else if (pqiHeart < HR_PQI_UPDATE_FLOOR) hrGateReason = HR_GATE_PQI;

                    bool hrUpdateEligible = autoValid && (pqiHeart >= HR_PQI_UPDATE_FLOOR);
                    bool hrCandidateEvaluated = autoValid;
                    if (hrColdStartWindowsRemaining > 0 && detectWindow_snap > 0 && hrCandidateEvaluated) {
                      bool rawConfirmsColdStartHigh = rawHRValid && isfinite(rawHR) && rawHR >= HR_COLD_START_CONFIRM_BPM;
                      if (hrUpdateEligible && hrAuto > HR_COLD_START_CEILING_BPM && !rawConfirmsColdStartHigh) {
                        hrUpdateEligible = false;
                        autoValid = false;
                        hrGateReason = HR_GATE_SPEC_REJECT;
                        confHR *= 0.30f;
                      }
                      hrColdStartWindowsRemaining--;
                    }
                    bool trustedAutoPreview = autoValid && (pqiHeart >= heartPqiGate);
                    if (safeElapsedMs(now, lastHrAutoDebugMs) >= 1000UL) {
                      lastHrAutoDebugMs = now;
                      Serial.printf("[HR_AUTO] autoValid=%d updOK=%d trustedEvid=%d pqi=%.3f conf=%.2f dist=%.1f cand=%.1f raw=%.1f motion=%d\n",
                                    (int)autoValid, (int)hrUpdateEligible, (int)trustedAutoPreview, pqiHeart, confHR,
                                    (radarDistance > 1.0f ? radarDistance : lastGoodDistance),
                                    hrAuto, rawHRValid ? rawHR : (latchedRawHRValidAt(now) ? latchedRawHR : 0.0f),
                                    (int)inMotion);
                    }

                    if (hrUpdateEligible) {
                      float doubled=hrAuto*2.0f;
                      float halved = hrAuto * 0.5f;

                      if (doubled<=HR_MAX&&motionCooldown==0) {
                        float sO=harmonicSpectrumScore(linearFused_snap,bufCount_snap,fs_snap,hrAuto);
                        float sD=harmonicSpectrumScore(linearFused_snap,bufCount_snap,fs_snap,doubled);
                        bool fftVote=(sD>sO*1.5f);
                        bool rawVeto=rawHRValid&&(fabsf(rawHR-hrAuto)<=fabsf(rawHR-doubled));

                        ghostSuspect = currentStaticReflector || rawHRGhostSuspect();
                        if (fftVote&&!rawVeto&&!ghostSuspect) hrAuto=doubled;
                      }

                      if (hrAuto >= 2.0f * HR_MIN && motionCooldown == 0) {
                        halved = hrAuto * 0.5f;
                        if (halved >= HR_MIN) {
                          float sO = harmonicSpectrumScore(linearFused_snap, bufCount_snap, fs_snap, hrAuto);
                          float sH = harmonicSpectrumScore(linearFused_snap, bufCount_snap, fs_snap, halved);
                          bool rawVoteHalf = rawHRValid && (fabsf(rawHR - halved) < fabsf(rawHR - hrAuto));
                          if (sH > sO * 1.3f && rawVoteHalf) hrAuto = halved;
                        }
                      }

                      // Harmonic ambiguity veto: if half-rate or double-rate remain comparably plausible
                      // and raw/trusted HR cannot clearly arbitrate, reject the auto-HR rather than force a mode.
                      bool rawArbAvailable = rawHRValid || latchedRawHRValidAt(now);
                      float rawArbValue = rawHRValid ? rawHR : (latchedRawHRValidAt(now) ? latchedRawHR : 0.0f);
                      float trustedHrAnchor = isTrustedPhaseHRFreshAt(now) ? trustedPhaseHR :
                                              ((lastStableHR >= 65.0f) ? lastStableHR :
                                              ((smoothHR >= 65.0f) ? smoothHR : 0.0f));
                      bool rawLooksLikeHalfRate = rawArbAvailable && (rawArbValue < 55.0f) &&
                                                  (trustedHrAnchor > 0.0f) &&
                                                  (rawArbValue < (trustedHrAnchor * 0.65f));
                      hrTrustedAnchorValue         = trustedHrAnchor;
                      hrRawLooksLikeHalfRateLogged = rawLooksLikeHalfRate;
                      float arbiterAnchor = (rawArbAvailable && !rawLooksLikeHalfRate) ? rawArbValue : trustedHrAnchor;
                      hrArbiterAnchorUsed  = (arbiterAnchor > 0.0f);
                      hrArbiterAnchorValue = arbiterAnchor;
                      bool arbiterCorrectionApplied = false;

                      if (!inMotion && arbiterAnchor > 55.0f && hrAuto < (arbiterAnchor * 0.65f)) {
                        float doubledCand = hrAuto * 2.0f;
                        bool doubledPlausible = (doubledCand <= HR_MAX) &&
                                                (fabsf(doubledCand - arbiterAnchor) <= HR_HARMONIC_RAW_ARBITER_BPM);
                        if (doubledPlausible) {
                          hrAuto = doubledCand;
                          arbiterCorrectionApplied = true;
                          hrArbiterCorrectedThisWindow = true;
                        }
                      }

                      float chosenScore = harmonicSpectrumScore(linearFused_snap, bufCount_snap, fs_snap, hrAuto);
                      bool rawSupportsChosen = (arbiterAnchor > 0.0f) && fabsf(arbiterAnchor - hrAuto) <= HR_HARMONIC_RAW_ARBITER_BPM;

                      bool altHalfStrong = false, altDoubleStrong = false;
                      bool rawSupportsHalf = false, rawSupportsDouble = false;
                      if (hrAuto * 0.5f >= HR_MIN) {
                        float halfCand = hrAuto * 0.5f;
                        float halfScore = harmonicSpectrumScore(linearFused_snap, bufCount_snap, fs_snap, halfCand);
                        altHalfStrong = (halfScore >= chosenScore * HR_HARMONIC_ALT_RATIO);
                        rawSupportsHalf = (arbiterAnchor > 0.0f) && fabsf(arbiterAnchor - halfCand) <= HR_HARMONIC_RAW_ARBITER_BPM;
                      }
                      if (hrAuto * 2.0f <= HR_MAX) {
                        float doubleCand = hrAuto * 2.0f;
                        float doubleScore = harmonicSpectrumScore(linearFused_snap, bufCount_snap, fs_snap, doubleCand);
                        altDoubleStrong = (doubleScore >= chosenScore * HR_HARMONIC_ALT_RATIO);
                        rawSupportsDouble = (arbiterAnchor > 0.0f) && fabsf(arbiterAnchor - doubleCand) <= HR_HARMONIC_RAW_ARBITER_BPM;
                      }

                      float halfCandVal = hrAuto * 0.5f;
                      bool halfPhysPlausible = (halfCandVal >= HR_HARMONIC_HALF_PLAUS_MAX) ||
                                               (arbiterAnchor <= 0.0f) ||
                                               (arbiterAnchor < HR_HARMONIC_RAW_PLAUS_MIN);
                      bool halfVetoEnabled = altHalfStrong && halfPhysPlausible && !arbiterCorrectionApplied;
                      if ((halfVetoEnabled && !rawSupportsChosen && !rawSupportsHalf) ||
                          (altDoubleStrong && !rawSupportsChosen && !rawSupportsDouble) ||
                          (halfVetoEnabled && altDoubleStrong && !rawSupportsChosen)) {
                        hrHarmonicAmbiguous = true;
                        hrGateReason = HR_GATE_HARMONIC;
                        confHR *= 0.35f;
                      }

                      bool upwardConfirmNeeded = isTrustedPhaseHRFreshAt(now) &&
                                                 (hrAuto - trustedPhaseHR) > HR_BIAS_DELTA_BPM;
                      if (upwardConfirmNeeded) {
                        if (hrLowMotionLockFirst() && pqiHeart >= heartPqiGate && hrTwoOfThreeAgree) {
                          if (hrUpwardConfirmWindows < 255) hrUpwardConfirmWindows++;
                        } else {
                          hrUpwardConfirmWindows = 0;
                        }
                        if (hrUpwardConfirmWindows < HR_UPWARD_CONFIRM_WINDOWS) {
                          hrGateReason = HR_GATE_UPWARD_CONFIRM;
                          hrAuto = trustedPhaseHR;
                          confHR *= 0.60f;
                          hrHarmonicAmbiguous = false;
                        } else {
                          hrUpwardConfirmWindows = 0;
                        }
                      } else {
                        hrUpwardConfirmWindows = 0;
                      }

                      rhcSuspect=false;
                      if (smoothRR>=RR_MIN&&smoothRR<=20.0f) {
                        for (int nn=2;nn<=5;nn++) { if (fabsf(hrAuto-nn*smoothRR)<=3.0f) { confHR*=0.6f;
                          rhcSuspect=true; break; } }
                      }

                      if (!hrHarmonicAmbiguous || rawSupportsChosen) {
                        lastAutoValid=true;
                        lastHrAuto=hrAuto; lastConfHR=confHR;
                        candidateHR = hrAuto;
                        candidateHRConf = confHR;
                        hrGateReason = HR_GATE_OK;
                      } else {
                        lastAutoValid=false;
                        lastConfHR=0.0f;
                        candidateHR = hrAuto;
                        candidateHRConf = confHR;
                        if (hrGateReason == HR_GATE_OK) hrGateReason = HR_GATE_HARMONIC;
                      }
                    } else { 
                        lastAutoValid=false;
                        ghostSuspect = currentStaticReflector; 
                        rhcSuspect=false; 
                        lastConfHR = 0.0f; 
                        candidateHRConf = 0.0f;
                        hrConfidence *= 0.90f; 
                    }

                    #if (DIAG_PLOTTER > 0)
                    Serial.print("E_Pre:");      Serial.print(diagAvgE_Pre  * 10000.0f, 4);
                    Serial.print(",E_RLS:");     Serial.print(diagAvgE_RLS  * 10000.0f, 4);
                    Serial.print(",E_Gate:");    Serial.print(diagAvgE_Gate * 10000.0f, 4);
                    Serial.print(",PQI_Heart:"); Serial.print(pqiHeart, 4);
                    Serial.print(",Gain:");      Serial.print(radarGain, 4);
                    Serial.println();
                    #endif

                    dspTask=1;

                  } else if (dspTask==1) {
                    hrGateReason = HR_GATE_NO_AUTO;
                    if (lastAutoValid) {
                      float finalHR = lastHrAuto;

                      bool latchedValidBlend = latchedRawHRValidAt(now);
                      float rawHR_use = rawHRValid ? rawHR : (latchedValidBlend ? latchedRawHR : 0.0f);
                      bool rawUse = rawHRValid || latchedValidBlend;

                      hrPreRejectPhase = finalHR;
                      bool directRawModeNow = v13_rawFastPathEligible(now, inMotion, rawHR_use);
                      useDirectRawHR = directRawModeNow;
                      bool phaseHrRejected = false;
                      if (!directRawModeNow) {
                        phaseHrRejected = rejectPhaseHRUpdate(finalHR, now, pqiHeartAtCandidateTime > 0.0f ? pqiHeartAtCandidateTime : pqiHeart,
                                                              hrArbiterCorrectedThisWindow,
                                                              hrRawLooksLikeHalfRateLogged,
                                                              hrArbiterAnchorValue);
                      } else {
                        finalHR = rawHR_use;
                        lastConfHR = fmaxf(lastConfHR, fminf(1.0f, pqiHeart * (v13_rawHRInRestingZone(rawHR_use) ? 1.35f : 1.0f) + 0.15f));
                        hrGateReason = HR_GATE_OK;
                      }
                      hrRejectPhaseRejectedThisWindow = phaseHrRejected;
                      hrPostRejectPhase = phaseHrRejected ? 0.0f : finalHR;
                      candidateHR = finalHR;
                      candidateHRConf = lastConfHR;
                      float governingAnchorNow = NAN;
                      uint8_t governingAnchorSourceNow = HR_ANCHOR_NONE;
                      currentGoverningHRAnchor(now, hrArbiterAnchorValue, hrRejectPhaseAnchorValue,
                                               governingAnchorNow, governingAnchorSourceNow);
                      bool lowerPhaseBiasTrigger = hrLowMotionLockFirst() &&
                                                   isfinite(hrPreRejectPhase) && hrPreRejectPhase >= HR_MIN &&
                                                   (governingAnchorSourceNow == HR_ANCHOR_TRACKING_RAW ||
                                                    governingAnchorSourceNow == HR_ANCHOR_LATCHED_RAW) &&
                                                   isfinite(governingAnchorNow) &&
                                                   ((governingAnchorNow - hrPreRejectPhase) >= HR_BIAS_DELTA_BPM);
                      if (lowerPhaseBiasTrigger) {
                        if (hrLowerPersistWindows < 255) hrLowerPersistWindows++;
                      } else {
                        hrLowerPersistWindows = 0;
                      }
                      hrDownwardOverrideActive = (hrLowerPersistWindows >= HR_DOWNWARD_OVERRIDE_WINDOWS);
                      hrRawHighBiasSuspectLogged = hrDownwardOverrideActive;
                      if (phaseHrRejected) {
                        hrGateReason = HR_GATE_RAW_DISAGREE;
                        if (rawUse && !hrDownwardOverrideActive) {
                          finalHR = rawHR_use;
                          lastConfHR *= 0.55f;
                        } else if (hrDownwardOverrideActive && isfinite(hrPreRejectPhase) && hrPreRejectPhase >= HR_MIN) {
                          finalHR = hrPreRejectPhase;
                          lastConfHR *= 0.75f;
                        } else {
                          lastAutoValid = false;
                          lastConfHR *= 0.45f;
                        }
                      }

                      if (lastAutoValid) {
                        hrConfidence = lastConfHR * pqiHeart;
                        if (pqiBreath < 0.20f) hrConfidence *= 0.70f;
                        if (ghostSuspect) hrConfidence *= 0.70f;
                        if (radarDistance > 1.0f && (radarDistance < 30.0f || radarDistance > 200.0f)) hrConfidence *= 0.70f;
                        hrConfidence = constrain(hrConfidence, 0.0f, 1.0f);

                        prevHrState = hrState;
                        hrState = (detectWindow_snap >= 3) ? 2 : 1;

                        if (rhcSuspect) hrState = min(hrState, 1);
                        if (ghostSuspect) hrState = min(hrState, 1);
                        if (phaseHrRejected) hrState = min(hrState, 1);

                        if (rawUse) {
                          float stab = rawHRStability();
                          float fresh = rawHRValid ? 1.0f : latchFreshnessAt(now);
                          float wR_raw = (stab >= 0.3f) ? fmaxf(stab, 0.5f * fresh) : stab;
                          float wD = hrConfidence;

                          if (phaseHrRejected) {
                            wD *= 0.20f;
                            wR_raw = fmaxf(wR_raw, 0.85f);
                          }
                          if (hrDownwardOverrideActive && rawHR_use > finalHR && (rawHR_use - finalHR) >= HR_BIAS_DELTA_BPM) {
                            wR_raw *= HR_DOWNWARD_RAW_BLEND_MULT;
                          }

                          if (isfinite(smoothHR) && smoothHR >= HR_MIN && smoothHR <= HR_MAX) {
                            float rawSmoothErr = fabsf(rawHR_use - smoothHR);
                            bool rawAbove = (rawHR_use > smoothHR);
                            if (rawSmoothErr > HR_RAW_DISAGREE_STRONG_BPM) wR_raw *= rawAbove ? 0.10f : 0.30f;
                            else if (rawSmoothErr > HR_RAW_DISAGREE_MED_BPM) wR_raw *= rawAbove ? 0.30f : 0.60f;
                          }

                          float wS = wD + wR_raw;
                          if (wS > 0.01f) finalHR = (wD * finalHR + wR_raw * rawHR_use) / wS;
                        }

                        hrPostBlend = finalHR;
                        finalHR = constrain(finalHR, HR_MIN, HR_MAX);
                        float preCoherenceHR = finalHR;
                        finalHR = coherenceFilter(finalHR);
                        if (!isfinite(finalHR) && isfinite(preCoherenceHR)) {
                          finalHR = preCoherenceHR;
                          hrConfidence *= 0.40f;
                          hrCoherenceRejectedThisWindow = true;
                        }
                        hrPostCoherence = isfinite(finalHR) ? finalHR : 0.0f;

                        if (isfinite(finalHR)) {
                          float hrTrustPqiGate = currentHeartPQIGate();
                          bool hrDriftGuardNow = (hrState >= 2) &&
                                                (lastTrackingHRMs > 0) &&
                                                (safeElapsedMs(now, lastTrackingHRMs) <= 10000UL) &&
                                                isfinite(trackingHR) &&
                                                (pqiHeart < 0.55f) &&
                                                (fabsf(finalHR - trackingHR) >= 8.0f);
                          float savedR = kfHR_R * (hrDriftGuardNow ? 4.0f : 1.0f);
                          if (hrDriftGuardNow) hrAnchorDriftSuspect = true;
                          float pqiScale = constrain(pqiHeart / fmaxf(hrTrustPqiGate, 0.01f), 0.1f, 2.0f);
                          kfHR_R = savedR / fmaxf(pqiScale * pqiScale, 0.01f);
                          if (pqiHeart < HR_PQI_UPDATE_FLOOR) {
                            kfHR_R = savedR * 16.0f;
                          }
                          smoothHR = safe_float(kalmanHR(finalHR));
                          kfHR_R = savedR;
                          pushHRVar(smoothHR);
                          lastHRUpdateMs = now;
                          hrUpdatedThisCycle = true;
                          hrUpdateSourceThisCycle = useDirectRawHR ? HR_PATH_DIRECT_RAW_FAST : ((phaseHrRejected && rawUse && !hrDownwardOverrideActive) ? HR_PATH_RAW_BLEND : HR_PATH_AUTO);
                          trackingHR = smoothHR;
                          lastTrackingHRMs = now;

                          bool hrKalmanUpdateAllowed = (pqiHeart >= hrTrustPqiGate) ||
                                                       (rawUse && hrRawAgreementGood && lastConfHR >= HR_LOG_CONF_BYPASS_MIN);
                          bool trustedHrEvidence = ((hrUpdateSourceThisCycle == HR_PATH_AUTO || hrUpdateSourceThisCycle == HR_PATH_SPECTRAL) && pqiHeart >= hrTrustPqiGate && hrConfidence >= 0.09f);
                          if (trustedHrEvidence && lastTrustedPhaseHRMs == 0UL) {
                            bool lowFirstSeedSuspect = smoothHR < HR_FIRST_PHASE_SEED_MIN_BPM &&
                                                       pqiHeart >= HR_FIRST_PHASE_SEED_PQI_SUSPECT;
                            if (lowFirstSeedSuspect) {
                              if (fabsf(smoothHR - hrFirstPhaseSeedConfirmLastBpm) <= HR_FIRST_PHASE_SEED_CONFIRM_DELTA_BPM) {
                                if (hrFirstPhaseSeedConfirmCount < 255) hrFirstPhaseSeedConfirmCount++;
                              } else {
                                hrFirstPhaseSeedConfirmCount = 1;
                                hrFirstPhaseSeedConfirmLastBpm = smoothHR;
                              }
                              if (hrFirstPhaseSeedConfirmCount < HR_FIRST_PHASE_SEED_CONFIRM_CYCLES) {
                                trustedHrEvidence = false;
                              }
                            } else {
                              hrFirstPhaseSeedConfirmCount = HR_FIRST_PHASE_SEED_CONFIRM_CYCLES;
                              hrFirstPhaseSeedConfirmLastBpm = smoothHR;
                            }
                          }
                          if (trustedHrEvidence && hrKalmanUpdateAllowed) {
                            lastValidRateMs = now;
                            lastTrustedHRMs = now;
                            lastTrustedVitalMs = now;
                            trustedPhaseHR = smoothHR;
                            lastTrustedPhaseHRMs = now;
                            sessionFirstSeedDone = true;
                          }
                          if (!hrKalmanUpdateAllowed && hrGateReason == HR_GATE_OK) {
                            hrGateReason = HR_GATE_PQI;
                          }
                        }

                        if (hrState == 2 && prevHrState < 2 && now - lastHRLockedBeepMs > DEBOUNCE_HR_LOCKED && !buzzerIsBusy()) {
                          buzzerPlay(BUZZ_HR_LOCKED);
                          lastHRLockedBeepMs = now;
                        }
                      }
                    }
                    dspTask = 2;

                  } else if (dspTask==2) {
                    if (!lastAutoValid && fs_snap >= 8.0f) {
                      float hrSpec = 0.0f;
                      float spectralMag = 0.0f;
                      detectSpectral(linearFused_snap, bufCount_snap, fs_snap, hrMinAdaptive/60.0f, hrMaxAdaptive/60.0f, hrSpec, spectralMag);
                      if (isfinite(hrSpec) && hrSpec > 0.0f && isfinite(spectralMag)) { hrSpecBpmLogged = hrSpec; hrSpecMagLogged = spectralMag; }

                      bool latchedValidSpec = latchedRawHRValidAt(now);
                      bool rawSp = rawHRValid || latchedValidSpec;
                      float rawHR_sp = rawSp ? (rawHRValid ? rawHR : latchedRawHR) : 0.0f;

                      float effectiveDistanceSpec = (radarDistance > 1.0f) ? radarDistance : lastGoodDistance;
                      bool nearFieldSpec = (effectiveDistanceSpec > 1.0f && effectiveDistanceSpec < 40.0f);
                      float spectralRawTol = nearFieldSpec ? 36.0f : 30.0f;
                      float spectralMagMin = nearFieldSpec ? 0.00035f : 0.001f;
                      float spectralPqiGate = currentHeartPQIGate();
                      bool spectralPlausible = (rawSp && fabsf(hrSpec - rawHR_sp) < spectralRawTol) ||
                                              (!rawSp && spectralMag > spectralMagMin && pqiHeart > spectralPqiGate);

                      hrPreRejectPhase = hrSpec;
                      bool phaseHrRejected = rejectPhaseHRUpdate(hrSpec, now, pqiHeartAtCandidateTime > 0.0f ? pqiHeartAtCandidateTime : pqiHeart,
                                                                  false, false, 0.0f);
                      hrRejectPhaseRejectedThisWindow = phaseHrRejected;
                      hrPostRejectPhase = phaseHrRejected ? 0.0f : hrSpec;
                      candidateHR = hrSpec;
                      candidateHRConf = spectralMag;
                      if (isfinite(hrSpec) && hrSpec > 0.0f) { lastCandidateHRMs = now; pqiHeartAtCandidateTime = pqiHeart; }
                      float governingAnchorSpec = NAN;
                      uint8_t governingAnchorSourceSpec = HR_ANCHOR_NONE;
                      currentGoverningHRAnchor(now, hrArbiterAnchorValue, hrRejectPhaseAnchorValue,
                                               governingAnchorSpec, governingAnchorSourceSpec);
                      bool lowerSpectralBiasTrigger = hrLowMotionLockFirst() &&
                                                      isfinite(hrSpec) && hrSpec >= HR_MIN &&
                                                      (governingAnchorSourceSpec == HR_ANCHOR_TRACKING_RAW ||
                                                       governingAnchorSourceSpec == HR_ANCHOR_LATCHED_RAW) &&
                                                      isfinite(governingAnchorSpec) &&
                                                      ((governingAnchorSpec - hrSpec) >= HR_BIAS_DELTA_BPM);
                      if (lowerSpectralBiasTrigger) {
                        if (hrLowerPersistWindows < 255) hrLowerPersistWindows++;
                      } else {
                        hrLowerPersistWindows = 0;
                      }
                      hrDownwardOverrideActive = (hrLowerPersistWindows >= HR_DOWNWARD_OVERRIDE_WINDOWS);
                      hrRawHighBiasSuspectLogged = hrDownwardOverrideActive;

                      if (!spectralPlausible) hrGateReason = HR_GATE_SPEC_REJECT;
                      else if (phaseHrRejected) hrGateReason = HR_GATE_RAW_DISAGREE;

                      if (spectralPlausible && !phaseHrRejected) {
                        float finalHR = hrSpec;

                        if (rawSp) {
                          float rawHR_u = rawHRValid ? rawHR : latchedRawHR;
                          hrConfidenceSourceThisCycle = HR_CONF_SPECTRAL_RAW_BLEND;
                          hrConfidence = (1.0f + rawHRStability() * 4.0f) * fmaxf(pqiHeart, 0.2f);
                          hrConfidence = constrain(hrConfidence, 0.0f, 1.0f);

                          if (hrState < 1) hrState = 1;
                          if (radarDistance > 1.0f && (radarDistance < 30.0f || radarDistance > 200.0f)) hrConfidence *= 0.70f;
                          float stab2 = rawHRStability();
                          float fresh2 = rawHRValid ? 1.0f : latchFreshnessAt(now);
                          float wR2 = (stab2 >= 0.3f) ? fmaxf(stab2, 0.5f * fresh2) : stab2;
                          if (hrDownwardOverrideActive && rawHR_u > finalHR && (rawHR_u - finalHR) >= HR_BIAS_DELTA_BPM) {
                            wR2 *= HR_DOWNWARD_RAW_BLEND_MULT;
                          }
                          if (isfinite(smoothHR) && smoothHR >= HR_MIN && smoothHR <= HR_MAX) {
                            float rawSmoothErr2 = fabsf(rawHR_u - smoothHR);
                            if (rawSmoothErr2 > HR_RAW_DISAGREE_STRONG_BPM) wR2 *= 0.20f;
                            else if (rawSmoothErr2 > HR_RAW_DISAGREE_MED_BPM) wR2 *= 0.50f;
                          }
                          float wD = hrConfidence;
                          float wS = wD + wR2;
                          if (wS > 0.01f) finalHR = (wD * finalHR + wR2 * rawHR_u) / wS;
                        } else {
                          hrConfidenceSourceThisCycle = HR_CONF_SPECTRAL_ONLY;
                          hrConfidence = pqiHeart * 0.5f;
                          if (pqiHeart < currentHeartPQIGate()) hrConfidence *= 0.3f;
                        }

                        if (hrState > 0) {
                          finalHR = constrain(finalHR, HR_MIN, HR_MAX);
                          float preCoherenceHR = finalHR;
                          finalHR = coherenceFilter(finalHR);
                          if (!isfinite(finalHR) && isfinite(preCoherenceHR)) {
                            finalHR = preCoherenceHR;
                            hrConfidence *= 0.40f;
                            hrCoherenceRejectedThisWindow = true;
                          }
                          if (isfinite(finalHR)) {
                            bool hrKalmanUpdateAllowed = (pqiHeart >= spectralPqiGate) ||
                                                         (rawSp && hrRawAgreementGood && spectralMag >= spectralMagMin && hrConfidence >= HR_LOG_CONF_BYPASS_MIN);
                            hrGateReason = HR_GATE_OK;
                            {
                              bool hrDriftGuardNow = (hrState >= 2) &&
                                                    (lastTrackingHRMs > 0) &&
                                                    (safeElapsedMs(now, lastTrackingHRMs) <= 10000UL) &&
                                                    isfinite(trackingHR) &&
                                                    (pqiHeart < 0.55f) &&
                                                    (fabsf(finalHR - trackingHR) >= 8.0f);
                              float savedR_s = kfHR_R * (hrDriftGuardNow ? 4.0f : 1.0f);
                              if (hrDriftGuardNow) hrAnchorDriftSuspect = true;
                              float pqiScale_s = constrain(pqiHeart / fmaxf(spectralPqiGate, 0.01f), 0.1f, 2.0f);
                              kfHR_R = savedR_s / fmaxf(pqiScale_s * pqiScale_s, 0.01f);
                              if (pqiHeart < HR_PQI_UPDATE_FLOOR) kfHR_R = savedR_s * 16.0f;
                              smoothHR = safe_float(kalmanHR(finalHR));
                              kfHR_R = savedR_s;
                              pushHRVar(smoothHR);
                              lastHRUpdateMs = now;
                              hrUpdatedThisCycle = true;
                              hrUpdateSourceThisCycle = HR_PATH_SPECTRAL;
                              trackingHR = smoothHR;
                              lastTrackingHRMs = now;
                            }
                            if (hrKalmanUpdateAllowed) {
                              bool trustedSpectralEvidence = (spectralPlausible && spectralMag > spectralMagMin && (rawSp || pqiHeart >= spectralPqiGate));
                              if (trustedSpectralEvidence && lastTrustedPhaseHRMs == 0UL) {
                                bool lowFirstSeedSuspect = smoothHR < HR_FIRST_PHASE_SEED_MIN_BPM &&
                                                           pqiHeart >= HR_FIRST_PHASE_SEED_PQI_SUSPECT;
                                if (lowFirstSeedSuspect) {
                                  if (fabsf(smoothHR - hrFirstPhaseSeedConfirmLastBpm) <= HR_FIRST_PHASE_SEED_CONFIRM_DELTA_BPM) {
                                    if (hrFirstPhaseSeedConfirmCount < 255) hrFirstPhaseSeedConfirmCount++;
                                  } else {
                                    hrFirstPhaseSeedConfirmCount = 1;
                                    hrFirstPhaseSeedConfirmLastBpm = smoothHR;
                                  }
                                  if (hrFirstPhaseSeedConfirmCount < HR_FIRST_PHASE_SEED_CONFIRM_CYCLES) {
                                    trustedSpectralEvidence = false;
                                  }
                                } else {
                                  hrFirstPhaseSeedConfirmCount = HR_FIRST_PHASE_SEED_CONFIRM_CYCLES;
                                  hrFirstPhaseSeedConfirmLastBpm = smoothHR;
                                }
                              }
                              if (trustedSpectralEvidence) {
                                lastValidRateMs = now;
                                lastTrustedHRMs = now;
                                lastTrustedVitalMs = now;
                                trustedPhaseHR = smoothHR;
                                lastTrustedPhaseHRMs = now;
                                sessionFirstSeedDone = true;
                              }
                            } else {
                              hrGateReason = HR_GATE_PQI;
                            }
                          }
                        }
                      }
                      bool rawFallbackEligible = !lastAutoValid &&
                                               rawSp && isfinite(rawHR_sp) &&
                                               rawHR_sp >= lastLoggedHrBandMin && rawHR_sp <= lastLoggedHrBandMax &&
                                               rawHRStability() >= (v13_rawHRInRestingZone(rawHR_sp) ? 0.35f : 0.45f) && !inMotion && motionCooldown == 0 &&
                                               !ghostSuspect && !hrRawLooksLikeHalfRateLogged;
                      if (!hrUpdatedThisCycle && rawFallbackEligible) {
                        float finalHR = constrain(rawHR_sp, HR_MIN, HR_MAX);
                        hrGateReason = HR_GATE_RAW_FALLBACK;
                        hrRawAgreementGood = true;
                        lastHRAgreementErr = 0.0f;
                        hrPreRejectPhase = finalHR;
                        hrPostRejectPhase = finalHR;
                        hrPostBlend = finalHR;
                        finalHR = coherenceFilter(finalHR);
                        hrPostCoherence = isfinite(finalHR) ? finalHR : 0.0f;
                        if (isfinite(finalHR)) {
                          smoothHR = safe_float(kalmanHR(finalHR));
                          pushHRVar(smoothHR);
                          lastHRUpdateMs = now;
                          hrUpdatedThisCycle = true;
                          hrUpdateSourceThisCycle = HR_PATH_RAW_FALLBACK;
                          trackingHR = smoothHR;
                          lastTrackingHRMs = now;
                          hrState = max(hrState, 1);
                          hrConfidenceSourceThisCycle = HR_CONF_RAW_FALLBACK;
                          hrConfidence = constrain(rawHRStability() * 0.45f * fmaxf(pqiHeart, 0.12f), 0.0f, 1.0f);
                          candidateHR = finalHR;
                          candidateHRConf = hrConfidence;
                          lastCandidateHRMs = now;
                          pqiHeartAtCandidateTime = pqiHeart;
                          lastValidRateMs = now;
                        }
                      }
                    }
                    dspTask = 3;

                  } else {
                    float rrDSP = 0.0f, confRR = 0.0f, finalRR = 0.0f;
                    bool rrValid = false;
                    bool rrAccepted = false;
                    bool rrPhaseAccepted = false;
                    bool rrPhaseRejected = false;
                    rrGateReason = RR_GATE_NO_AUTO;
                    rrFundamentalRecoveryTriggeredLogged = false;

                    if (fs_snap >= 3.0f) {
                      rrValid = detectRate(linearBreath_snap, bufCount_snap, fs_snap, RR_MIN, RR_MAX, rrDSP, confRR, RR_CONF_THRESHOLD);
                    }

                    rrZcBpmLogged = 0.0f;
                    rrZcConfLogged = 0.0f;
                    rrSpecBpmLogged = 0.0f;
                    rrSpecConfLogged = 0.0f;
                    rrTripleAgreeLogged = false;
                    candidateRR = rrValid ? rrDSP : 0.0f;
                    candidateRRConf = rrValid ? confRR : 0.0f;
                    rrPreAcceptPhase = rrValid ? rrDSP : 0.0f;
                    rrPostAcceptPhase = 0.0f;
                    rrPostBlend = 0.0f;
                    rrPostBiasCorrection = 0.0f;
                    rrPostKalman = 0.0f;
                    rrFinalPublishCandidate = 0.0f;
                    rrRawSeedConsistentCountLogged = rrSeedConsistentCount;
                    rrMidSessionRawReanchorAllowedLogged = false;
                    rrMidSessionRawReanchorBlockedLogged = false;
                    rrMidSessionRawReanchorReasonLogged = 0;
                    rrRawAnchorErrBpmLogged = -1.0f;
                    if (rrValid) lastCandidateRRMs = now;

                    if (!rrValid) {
                      rrGateReason = inMotion ? RR_GATE_MOTION : RR_GATE_NO_AUTO;
                    } else if (pqiBreath < RR_PQI_MIN_ACCEPT) {
                      rrGateReason = RR_GATE_PQI;
                    } else if (confRR < RR_CONF_MIN_ACCEPT) {
                      rrGateReason = RR_GATE_CONF;
                    } else if (rawRRValid && isfinite(rawRR) && fabsf(rrDSP - rawRREffective) > RR_RAW_DISAGREE_BPM && pqiBreath < 0.25f) {
                      rrGateReason = RR_GATE_RAW_DISAGREE;
                    }

                    bool rrHarmonicSuspect = false;
                    bool rrFundamentalRecoveryActive = false;
                    float rrAnchor = 0.0f;
                    bool rrAnchorFresh = false;
                    bool rrAnchorDegraded = false;
                    if (lastRRUpdateMs > 0 && safeElapsedMs(now, lastRRUpdateMs) < RR_SUBHARMONIC_ANCHOR_MAX_AGE_MS &&
                        isfinite(kfRR_x) && kfRR_x >= RR_MIN + 2.0f && kfRR_x <= RR_MAX) {
                      rrAnchor = kfRR_x;
                      rrAnchorFresh = true;
                    } else if (isRRFresh() && isfinite(smoothRR) && smoothRR >= RR_MIN + 2.0f && smoothRR <= RR_MAX) {
                      rrAnchor = smoothRR;
                      rrAnchorFresh = true;
                    } else if (!inMotion && lastValidDisplayRRMs > 0 &&
                               safeElapsedMs(now, lastValidDisplayRRMs) < RR_DISPLAY_GRACE_MS &&
                               isfinite(lastValidDisplayRR) &&
                               lastValidDisplayRR >= RR_MIN + 2.0f &&
                               lastValidDisplayRR <= RR_MAX) {
                      rrAnchor = lastValidDisplayRR;
                      rrAnchorFresh = true;
                      rrAnchorDegraded = true;
                    }
                    rrAnchorFreshLogged = rrAnchorFresh;
                    rrAnchorValueLogged = rrAnchorFresh ? rrAnchor : 0.0f;
                    rrAnchorAgeMsLogged = 0UL;
                    rrAnchorSourceLogged = 0;
                    rrAnchorConfidenceLogged = 0.0f;
                    if (rrAnchorFresh) {
                      unsigned long rrAnchorMaxAge = rrAnchorDegraded ? RR_DISPLAY_GRACE_MS : RR_SUBHARMONIC_ANCHOR_MAX_AGE_MS;
                      if (rrAnchorSourceLogged == 0) {
                        if (lastRRUpdateMs > 0 && isfinite(kfRR_x) && rrAnchor == kfRR_x) {
                          rrAnchorSourceLogged = 1;
                          rrAnchorAgeMsLogged = safeElapsedMs(now, lastRRUpdateMs);
                        } else if (isRRFresh() && isfinite(smoothRR) && rrAnchor == smoothRR) {
                          rrAnchorSourceLogged = 2;
                          rrAnchorAgeMsLogged = (lastRRUpdateMs > 0) ? safeElapsedMs(now, lastRRUpdateMs) : 0UL;
                        } else if (lastValidDisplayRRMs > 0 && isfinite(lastValidDisplayRR) && rrAnchor == lastValidDisplayRR) {
                          rrAnchorSourceLogged = 3;
                          rrAnchorAgeMsLogged = safeElapsedMs(now, lastValidDisplayRRMs);
                        }
                      }
                      rrAnchorConfidenceLogged = constrain(1.0f - ((float)rrAnchorAgeMsLogged / fmaxf((float)rrAnchorMaxAge, 1.0f)), 0.0f, 1.0f);
                    }
                    float rrZc = 0.0f, rrZcConf = 0.0f;
                    bool rrZcValid = false;
                    if (fs_snap >= 3.0f) {
                      rrZcValid = detectRateZeroCrossing(linearBreath_snap, bufCount_snap, fs_snap, RR_MIN, RR_MAX, rrZc, rrZcConf);
                    }
                    if (rrZcValid) { rrZcBpmLogged = rrZc; rrZcConfLogged = rrZcConf; }

                    float rrSpec = 0.0f, rrSpecConf = 0.0f;
                    bool rrSpecValid = false;
                    if (fs_snap >= 3.0f) {
                      float rrSpecFMin = currentRRSpectralFMin(rrAnchorFresh ? rrAnchor : 0.0f);
                      rrSpecValid = detectLowestSpectralFundamental(linearBreath_snap, bufCount_snap, fs_snap, rrSpecFMin, RR_MAX / 60.0f, rrSpec, rrSpecConf, rrAnchorFresh ? rrAnchor : 0.0f);
                    }
                    if (rrSpecValid) { rrSpecBpmLogged = rrSpec; rrSpecConfLogged = rrSpecConf; }
                    if (rrValid && rrZcValid && rrSpecValid) {
                      rrTripleAgreeLogged = (fabsf(rrDSP - rrZc) <= RR_TRIPLE_AGREE_BPM) &&
                                            (fabsf(rrDSP - rrSpec) <= RR_TRIPLE_AGREE_BPM) &&
                                            (fabsf(rrZc - rrSpec) <= RR_TRIPLE_AGREE_BPM);
                    }

                    if (rrValid && rrAnchorFresh) {
                      float rrSubRatio = rrAnchorDegraded ? (RR_SUBHARMONIC_ANCHOR_RATIO - 0.04f) : RR_SUBHARMONIC_ANCHOR_RATIO;
                      float rrHighRatio = rrAnchorDegraded ? (RR_HARMONIC_UPPER_ANCHOR_RATIO - 0.05f) : RR_HARMONIC_UPPER_ANCHOR_RATIO;
                      bool muchLowerThanAnchor = rrDSP < (rrAnchor * rrSubRatio);
                      bool muchHigherThanAnchor = rrDSP > (rrAnchor * rrHighRatio);
                      bool rawDoesNotConfirmLow = (!rawRRValid) || !isfinite(rawRREffective) || (rawRREffective > rrDSP + 2.0f);
                      bool rawDoesNotConfirmHigh = (!rawRRValid) || !isfinite(rawRREffective) || (rawRREffective < rrDSP - 2.0f);
                      bool rrOutlierLikeLast = (lastOutlierRRCandidate > 0.0f) && (fabsf(rrDSP - lastOutlierRRCandidate) <= 1.0f);
                      bool candidateNearHalfAnchor = false;
                      if (rrAnchor > (RR_MIN * 2.0f)) {
                        float recoveryTol = fmaxf(RR_FUNDAMENTAL_RECOVERY_ABS_TOL_BPM, rrAnchor * RR_FUNDAMENTAL_RECOVERY_REL_TOL);
                        candidateNearHalfAnchor = fabsf(rrDSP - (rrAnchor * 0.5f)) <= recoveryTol;
                      }
                      float rrEffectiveDistance = (radarDistance > 1.0f) ? radarDistance : lastGoodDistance;
                      bool rrNearFieldRecovery = (rrEffectiveDistance > 1.0f && rrEffectiveDistance < 40.0f);
                      float rrRecoveryMinPqi = rrNearFieldRecovery ? RR_FUNDAMENTAL_RECOVERY_MIN_PQI_NEAR_FIELD : RR_FUNDAMENTAL_RECOVERY_MIN_PQI;
                      uint8_t rrRecoveryMinWindows = rrNearFieldRecovery ? RR_FUNDAMENTAL_RECOVERY_MIN_WINDOWS_NEAR_FIELD : RR_FUNDAMENTAL_RECOVERY_MIN_WINDOWS;
                      if (candidateNearHalfAnchor && confRR >= RR_FUNDAMENTAL_RECOVERY_MIN_CONF &&
                          pqiBreath >= rrRecoveryMinPqi && !inMotion) {
                        rrFundamentalRecoveryCount = (uint8_t)min(255, (int)rrFundamentalRecoveryCount + 1);
                      } else {
                        rrFundamentalRecoveryCount = 0;
                      }
                      rrFundamentalRecoveryCountLogged = rrFundamentalRecoveryCount;
                      bool allowFundamentalRecovery = rrFundamentalRecoveryCount >= rrRecoveryMinWindows;
                      if (muchLowerThanAnchor || muchHigherThanAnchor) {
                        rrOutlierPersistCount = rrOutlierLikeLast ? (uint8_t)min(255, (int)rrOutlierPersistCount + 1) : 1;
                        lastOutlierRRCandidate = rrDSP;
                      } else {
                        rrOutlierPersistCount = 0;
                        lastOutlierRRCandidate = 0.0f;
                      }
                      bool allowPersistentOutlier = (rrOutlierPersistCount >= RR_PERSISTENT_OUTLIER_MIN_WINDOWS) &&
                                                    (confRR >= (RR_CONF_MIN_ACCEPT + 0.05f)) &&
                                                    (pqiBreath >= (RR_PQI_MIN_ACCEPT + 0.05f)) &&
                                                    !inMotion;
                      if (allowFundamentalRecovery && muchLowerThanAnchor) {
                        muchLowerThanAnchor = false;
                        rawDoesNotConfirmLow = false;
                        rrFundamentalRecoveryActive = true;
                        rrFundamentalRecoveryTriggeredLogged = true;
                      }
                      if (muchLowerThanAnchor && rawDoesNotConfirmLow && !allowPersistentOutlier) {
                        rrHarmonicSuspect = true;
                        rrGateReason = RR_GATE_SUBHARMONIC;
                      } else if (muchHigherThanAnchor && rawDoesNotConfirmHigh && !allowPersistentOutlier) {
                        rrHarmonicSuspect = true;
                        rrGateReason = RR_GATE_HARMONIC;
                      }
                    } else {
                      rrFundamentalRecoveryCount = 0;
                      rrFundamentalRecoveryCountLogged = 0;
                      // Anchor stale: preserve outlier persistence state across brief staleness.
                      // Reset only when the candidate returns to the normal anchor range.
                    }

                    bool rawRRFallbackOk = rawRRValid && isfinite(rawRREffective) && rawRREffective >= RR_MIN && rawRREffective <= RR_MAX;
                    bool rawRRMidSessionGuard = (lastPersonDetectedEventMs > 0UL) &&
                                                (safeElapsedMs(now, lastPersonDetectedEventMs) >= RR_MIDSESSION_RAW_REANCHOR_GRACE_MS);
                    if (rawRRFallbackOk && rawRRMidSessionGuard && rrSeedConsistentCount < RR_MIDSESSION_RAW_REANCHOR_MIN_CONSISTENT_READS) {
                      rawRRFallbackOk = false;
                      rrMidSessionRawReanchorBlockedLogged = true;
                      rrMidSessionRawReanchorReasonLogged = 1;
                    }
                    if (rawRRFallbackOk && rrAnchorFresh) {
                      float rawAnchorErr = fabsf(rawRREffective - rrAnchor);
                      rrRawAnchorErrBpmLogged = rawAnchorErr;
                      if (rawAnchorErr > RR_RAW_FALLBACK_MAX_ERR_BPM ||
                          (rawRRMidSessionGuard && rawAnchorErr > RR_MIDSESSION_RAW_REANCHOR_MAX_ANCHOR_ERR_BPM)) {
                        rawRRFallbackOk = false;
                        rrMidSessionRawReanchorBlockedLogged = true;
                        rrMidSessionRawReanchorReasonLogged = 2;
                      }
                    }
                    if (rawRRFallbackOk && rrValid && isfinite(rrDSP)) {
                      float rawCandErr = fabsf(rawRREffective - rrDSP);
                      if (rawCandErr > RR_RAW_FALLBACK_MAX_ERR_BPM) {
                        rawRRFallbackOk = false;
                        rrMidSessionRawReanchorBlockedLogged = true;
                        rrMidSessionRawReanchorReasonLogged = 3;
                      }
                    }
                    rrMidSessionRawReanchorAllowedLogged = rawRRMidSessionGuard && rawRRFallbackOk;

                    if (rrValid && !rrHarmonicSuspect && acceptPhaseRRUpdate(rrDSP, confRR)) {
                      finalRR = rrDSP;
                      rrAccepted = true;
                      rrPhaseAccepted = true;
                      rrPhaseBackedUpdateThisCycle = true;
                      rrGateReason = RR_GATE_OK;
                      rrPostAcceptPhase = finalRR;
                    } else if (rawRRFallbackOk && (!rrValid || rrGateReason == RR_GATE_NO_AUTO || rrGateReason == RR_GATE_MOTION)) {
                      finalRR = rawRREffective;
                      rrAccepted = true;
                      rrPhaseRejected = rrValid;
                      rrPostAcceptPhase = finalRR;
                      if (rrGateReason == RR_GATE_NO_AUTO) rrGateReason = RR_GATE_DISPLAY_ONLY;
                    }

                    if (rrAccepted) {
                      if (rrPhaseAccepted && rawRRFallbackOk) {
                        float rr_diff = fabsf(rrDSP - rawRREffective);
                        if (rr_diff <= 2.0f) {
                          float rawW = (rawRREffective < rrDSP) ? RR_MODULE_RAW_CORRECTION_BLEND : 0.12f;
                          finalRR = (1.0f - rawW) * rrDSP + rawW * rawRREffective;
                        } else if (rr_diff <= 4.0f) {
                          float rawW = (rawRREffective < rrDSP) ? 0.20f : 0.06f;
                          finalRR = (1.0f - rawW) * rrDSP + rawW * rawRREffective;
                        } else {
                          finalRR = rrDSP;
                        }
                      }
                      rrPostBlend = finalRR;

                      if (rrPhaseAccepted) {
                        float rrBiasAdj = constrain(RR_BIAS_CORRECTION_BPM, -RR_BIAS_CORRECTION_MAX_ABS, RR_BIAS_CORRECTION_MAX_ABS);
                        finalRR = constrain(finalRR + rrBiasAdj, RR_MIN, RR_MAX);
                      }
                      rrPostBiasCorrection = finalRR;
                      if (rrFundamentalRecoveryActive && rrPhaseAccepted) {
                        kfRR_x = finalRR;
                        smoothRR = finalRR;
                        rrFundamentalRecoveryCount = 0;
                        rrOutlierPersistCount = 0;
                        lastOutlierRRCandidate = 0.0f;
                      } else {
                        float rrConfScale = constrain(confRR * (pqiBreath / fmaxf(RR_PQI_MIN_ACCEPT, 0.01f)), 0.2f, 1.5f);
                        smoothRR = safe_float(kalmanRR(finalRR, rrConfScale));
                      }
                      rrPostKalman = smoothRR;
                      rrFinalPublishCandidate = smoothRR;
                      lastRRUpdateMs = now;

                      if (rrPhaseAccepted && confRR >= RR_CONF_MIN_ACCEPT && pqiBreath >= RR_PQI_MIN_ACCEPT) {
                        lastValidDisplayRRMs = now;
                        lastValidDisplayRR = smoothRR;
                      }

                      bool trustedRREvidence = (rrPhaseAccepted && confRR >= RR_CONF_MIN_ACCEPT && pqiBreath >= RR_PQI_MIN_ACCEPT) || (rawRRValid && isfinite(rawRREffective) && !rrPhaseRejected);
                      if (trustedRREvidence) {
                        lastValidRateMs = now;
                        lastTrustedRRMs = now;
                        lastTrustedVitalMs = now;
                      }
                    }

                    lastRRDSP = rrDSP;
                    lastConfRR = confRR;
                    lastRRValid = (rrPhaseAccepted && pqiBreath >= RR_PQI_MIN_ACCEPT);

                    pqiHeart *= 0.995f;   // v11.6.7: lighter decay in active DSP path so weak signal can accumulate
                    pqiBreath *= 0.995f;
                    dspTask = 0;
                  }
                }
                if (!inMotion && motionCooldown > 0 && dspTask == 0) {
                  motionCooldown--;
                }
              }
            } 
          }
        }
      }

      if (calibrationDone&&!phaseDataValid) {
        if (rawHRValid) {
          float finalHR=rawHR;
          if (smoothHR>=HR_MIN) {
            float rawBlendW = (fabsf(rawHR - smoothHR) > HR_RAW_DISAGREE_STRONG_BPM) ? 0.10f : 0.30f;
            finalHR = rawBlendW*rawHR + (1.0f-rawBlendW)*smoothHR;
          }
          finalHR=constrain(finalHR,HR_MIN,HR_MAX);
          hrPreRejectPhase = finalHR;
          hrPostRejectPhase = finalHR;
          hrPostBlend = finalHR;
          finalHR=coherenceFilter(finalHR);
          hrPostCoherence = isfinite(finalHR) ? finalHR : 0.0f;
          if (isfinite(finalHR) && rawHRStability() >= 0.35f) {   // v11.6.7: cold-start fallback reachable
            smoothHR=safe_float(kalmanHR(finalHR)); pushHRVar(smoothHR);
            lastHRUpdateMs=now;
            hrUpdatedThisCycle = true;
            hrUpdateSourceThisCycle = HR_PATH_RAW_ONLY_NO_PHASE;
            hrConfidenceSourceThisCycle = HR_CONF_RAW_ONLY;
            trackingHR = smoothHR;
            lastTrackingHRMs = now;
            hrGateReason = HR_GATE_RAW_FALLBACK;
            candidateHR = finalHR;
            candidateHRConf = fmaxf(hrConfidence, 0.30f);
            lastCandidateHRMs = now;
            hrFinalPublishCandidate = finalHR;
            if ((presenceState == PRESENCE_PRESENT || presenceState == PRESENCE_SILENT_HOLD || presenceState == PRESENCE_LEAVING) && hrState<1) hrState=1; 
            hrConfidence=fmaxf(hrConfidence,0.3f);
          }
        }
        if (rawRRValid) {
          float finalRR=rawRREffective;
          rrPreAcceptPhase = finalRR;
          rrPostAcceptPhase = finalRR;
          if (smoothRR>=RR_MIN && fabsf(rawRREffective - smoothRR) <= RR_RAW_UPDATE_MAX_DIFF_BPM) finalRR=0.3f*rawRREffective+0.7f*smoothRR;
          rrPostBlend = finalRR;
          rrPostBiasCorrection = finalRR;
          if (!(smoothRR>=RR_MIN) || fabsf(rawRREffective - smoothRR) <= RR_RAW_UPDATE_MAX_DIFF_BPM) {
            smoothRR=safe_float(kalmanRR(constrain(finalRR,RR_MIN,RR_MAX)));
            rrPostKalman = smoothRR;
            rrFinalPublishCandidate = smoothRR;
            lastRRUpdateMs=now; 
            candidateRR = constrain(finalRR, RR_MIN, RR_MAX);
            candidateRRConf = fmaxf(lastConfRR, 0.25f);
            lastCandidateRRMs = now;
          }
        }
      }
  } 

  unsigned long radarLastSeenMs = (lastRadarPacketMs > 0) ? lastRadarPacketMs : radarWatchdogStartMs;
  bool silenceArmed = safeElapsedMs(now, radarWatchdogStartMs) > RADAR_RECOVERY_GRACE_MS;
  bool packetSilent = silenceArmed && (safeElapsedMs(now, radarLastSeenMs) > RADAR_SILENCE_TIMEOUT_MS);
  bool badRadarCondition = packetSilent || badRadarPacketThisFrame;
  if (badRadarCondition) {
    if (lastBadRadarTickMs == 0 || safeElapsedMs(now, lastBadRadarTickMs) >= BAD_RADAR_TICK_MS) {
      lastBadRadarTickMs = now;
      if (consecutiveBadRadar < 255) consecutiveBadRadar++;
    }
  } else if (newData && goodRadarPacketThisFrame) {
    consecutiveBadRadar = 0;
    lastBadRadarTickMs = now;
  }
  if (consecutiveBadRadar >= BAD_RADAR_LIMIT &&
      (lastRadarRecoveryAttemptMs == 0 || safeElapsedMs(now, lastRadarRecoveryAttemptMs) >= RADAR_RECOVERY_COOLDOWN_MS)) {
    lastRadarRecoveryAttemptMs = now;
    int safe = 0;
    while (mmWaveSerial.available() && safe++ < 1024) {
      mmWaveSerial.read();
      wdtReset(); buzzerUpdate();
    }
    mmWaveSerial.end();
    delay(20);
    mmWaveSerial.setRxBufferSize(MMWAVE_RX_BUFFER_SIZE);
    mmWaveSerial.begin(115200);
    mmWave.begin(&mmWaveSerial);
    radarWatchdogStartMs = now;
    lastRadarPacketMs = 0;
    consecutiveBadRadar = BAD_RADAR_LIMIT - 1;
    lastBadRadarTickMs = now;
  }

  if (lastPhaseDataMs > 0 && now - lastPhaseDataMs > 5000UL) motionCooldown = 0;
  wasMotion=inMotion;

  bool ambientEvidenceLatched = lastAmbientJumpMs > 0 && (safeElapsedMs(now, lastAmbientJumpMs) < AMBIENT_EVIDENCE_HOLD_MS);
  bool radarPresenceEvidence = lastPresenceUpdateMs > 0 && (safeElapsedMs(now, lastPresenceUpdateMs) < RADAR_PRESENCE_EVIDENCE_MS);
  bool radarEvidence = newData && (rawHRValid || rawRRValid);
  bool dspPresenceEvidence = lastTrustedVitalMs > 0 && (safeElapsedMs(now, lastTrustedVitalMs) < DSP_PRESENCE_EVIDENCE_MS);
  bool livePhaseEvidence = (lastLivePhaseMs > 0) && (safeElapsedMs(now, lastLivePhaseMs) < LIVE_PHASE_HOLD_MS) &&
                           (lastPhaseDataMs > 0) && (safeElapsedMs(now, lastPhaseDataMs) < LIVE_PHASE_PACKET_MAX_MS) &&
                           (phaseLivenessScore >= 0.10f);
  bool weakDistanceEvidence = (lastDistanceUpdateMs > 0) && (safeElapsedMs(now, lastDistanceUpdateMs) < DISTANCE_PRESENCE_EVIDENCE_MS) &&
                              lastGoodDistance >= 25.0f && lastGoodDistance <= 200.0f;
  bool strongPresenceEvidence = radarEvidence || radarPresenceEvidence || livePhaseEvidence;
  bool weakPresenceEvidence = dspPresenceEvidence || weakDistanceEvidence;
  bool departureMotionEvidence = (lastMotionDetectedMs > 0) && (safeElapsedMs(now, lastMotionDetectedMs) < 2000UL);

  if (strongPresenceEvidence) { latchedStrongEvidence = true; lastLatchedEvidenceMs = now; }
  if (weakPresenceEvidence) { latchedWeakEvidence = true; lastLatchedEvidenceMs = now; }
  if (lastLatchedEvidenceMs > 0 && safeElapsedMs(now, lastLatchedEvidenceMs) > EVIDENCE_LATCH_MAX_MS) {
    latchedStrongEvidence = false;
    latchedWeakEvidence = false;
  }

  if (now - lastPhaseLivenessDecayMs >= PHASE_LIVENESS_DECAY_MS) {
    lastPhaseLivenessDecayMs = now;
    if (lastLivePhaseMs > 0 && safeElapsedMs(now, lastLivePhaseMs) > PHASE_LIVENESS_DECAY_MS) phaseLivenessScore *= 0.97f;
  }

  if (freshDistanceSampleThisFrame && !staticReflectorUpdatedThisFrame) {
    currentStaticReflector = applyStaticReflectorPenalty(true, livePhaseEvidence);
    staticReflectorUpdatedThisFrame = true;
  } else if (!isDistanceFresh()) {
    currentStaticReflector = false;
    staticReflectorConsecutive = 0;
    currentDistStdDev = 0.0f;
  }
  if (livePhaseEvidence || rawHRValid || rawRRValid || latchedRawHRValidAt(now)) {
    currentStaticReflector = false;
    staticReflectorConsecutive = 0;
    currentDistStdDev = 0.0f;
  }
  bool staticReflector = currentStaticReflector;
  ghostSuspect = staticReflector || rawHRGhostSuspect();
  if (ghostSuspect) {
    if (ghostSuspectCounter < 255) ghostSuspectCounter++;
  } else {
    ghostSuspectCounter = 0;
  }

  if (now - lastConfidenceDecayMs >= CONFIDENCE_DECAY_MS) {
    lastConfidenceDecayMs = now;
    if (humanDetected && !livePhaseEvidence && !rawHRValid && !rawRRValid) {
      hrConfidence *= 0.85f;
      pqiHeart *= 0.85f;
      pqiBreath *= 0.85f;
      if (!isTrustedVitalFreshAt(now)) hrState = 0;
    }
  }


  distanceConfidence = computeDistanceConfidenceScore(lastGoodDistance, isDistanceFreshAt(now));
  float stateBase = 0.05f;
  switch (presenceState) {
    case PRESENCE_ENTERING: stateBase = 0.35f; break;
    case PRESENCE_PRESENT: stateBase = 0.80f; break;
    case PRESENCE_SILENT_HOLD: stateBase = 0.60f; break;
    case PRESENCE_LEAVING: stateBase = 0.25f; break;
    default: stateBase = 0.05f; break;
  }
  presenceConfidence = constrain(
      stateBase +
      (rawHRValid ? 0.12f : 0.0f) +
      (rawRRValid ? 0.10f : 0.0f) +
      (radarPresenceEvidence ? 0.10f : 0.0f) +
      (livePhaseEvidence ? 0.18f : 0.0f) +
      (weakDistanceEvidence ? 0.08f : 0.0f) +
      (ambientEvidenceLatched ? 0.02f : 0.0f),
      0.0f, 1.0f);

  if (!(reentryLockoutMs > 0 && (safeElapsedMs(now, reentryLockoutMs) < REENTRY_LOCKOUT_MS))) {
    if (strongPresenceEvidence) lastStrongPresenceMs = now;
    if (strongPresenceEvidence || weakPresenceEvidence) lastWeakPresenceMs = now;
  }

  if (departureMotionEvidence) lastEvidenceLogMs = now;

  // Time-based presence FSM (foundation fix from corrected v10.8.9c)
  if (calibrationDone && now - lastPresenceTickMs >= 250UL) {
    lastPresenceTickMs = now;

    bool tickStrongEvidence = latchedStrongEvidence || strongPresenceEvidence;
    bool tickWeakEvidence = latchedWeakEvidence || weakPresenceEvidence;
    latchedStrongEvidence = false;
    latchedWeakEvidence = false;

    bool exitLivePhaseEvidence = livePhaseEvidence && (phaseLivenessScore >= FALSE_PRESENCE_PHASE_MIN) && !currentStaticReflector;
    bool hardNoPresenceEvidence = !radarIsPresentRaw && !radarEvidence && !radarPresenceEvidence &&
                                  !dspPresenceEvidence && !weakDistanceEvidence &&
                                  !rawHRValid && !rawRRValid && !exitLivePhaseEvidence;
    if (presenceState == PRESENCE_PRESENT || presenceState == PRESENCE_SILENT_HOLD || presenceState == PRESENCE_LEAVING) {
      if (hardNoPresenceEvidence) {
        falsePresenceStartMs = (falsePresenceStartMs == 0) ? now : falsePresenceStartMs;
        falsePresenceAbsenceScore = (uint8_t)min((int)FALSE_PRESENCE_ABSENCE_SCORE_MAX, (int)falsePresenceAbsenceScore + 2);
      } else {
        if (falsePresenceAbsenceScore > 0) falsePresenceAbsenceScore--;
        if (falsePresenceAbsenceScore == 0) falsePresenceStartMs = 0;
      }
    } else {
      falsePresenceStartMs = 0;
      falsePresenceAbsenceScore = 0;
    }

    switch (presenceState) {
      case PRESENCE_ABSENT: {
        presentVotes = 0;
        absentVotes = 0;
        if (reentryLockoutMs > 0 && (safeElapsedMs(now, reentryLockoutMs) < REENTRY_LOCKOUT_MS)) {
          break;
        }
        bool weakEntryCooldownActive = (weakEntryCooldownMs > 0UL) &&
                                       (safeElapsedMs(now, weakEntryCooldownMs) < WEAK_ENTRY_RETRY_COOLDOWN_MS);
        if ((tickStrongEvidence || (tickWeakEvidence && !weakEntryCooldownActive)) && !staticReflector) {
          entrySawStrongEvidence = tickStrongEvidence;
          entryStrongSinceMs = tickStrongEvidence ? now : 0UL;
          enterPresenceState(PRESENCE_ENTERING, now, tickStrongEvidence ? "strong_evidence" : "weak_evidence");
        }
        break;
      }

      case PRESENCE_ENTERING: {
        unsigned long enteringElapsed = safeElapsedMs(now, presenceStateSinceMs);
        if (tickStrongEvidence) {
          if (entryStrongSinceMs == 0UL) entryStrongSinceMs = now;
        } else {
          entryStrongSinceMs = 0UL;
        }
        entrySawStrongEvidence = (entryStrongSinceMs > 0UL);
        unsigned long strongElapsed = entrySawStrongEvidence ? safeElapsedMs(now, entryStrongSinceMs) : 0UL;
        if (staticReflector && !radarEvidence && !livePhaseEvidence) {
          entrySawStrongEvidence = false;
          entryStrongSinceMs = 0UL;
          enterPresenceState(PRESENCE_ABSENT, now, "reflector_gate");
          break;
        }
        if (!(tickStrongEvidence || tickWeakEvidence)) {
          entrySawStrongEvidence = false;
          entryStrongSinceMs = 0UL;
          enterPresenceState(PRESENCE_ABSENT, now, "entry_lost");
          break;
        }
        unsigned long voteElapsed = entrySawStrongEvidence ? strongElapsed : enteringElapsed;
        unsigned long confirmMs = entrySawStrongEvidence ? ENTER_CONFIRM_STRONG_MS : ENTER_CONFIRM_MS;
        presentVotes = (int)min(5UL, (voteElapsed * 5UL) / max(1UL, confirmMs));
        if (tickStrongEvidence && entrySawStrongEvidence && strongElapsed >= ENTER_CONFIRM_STRONG_MS) {
          weakEntryCooldownMs = 0UL;
          enterPresenceState(PRESENCE_PRESENT, now, "confirmed");
          handlePersonDetected(now);
        } else if (!entrySawStrongEvidence && enteringElapsed >= ENTER_CONFIRM_MS) {
          weakEntryCooldownMs = now;
          enterPresenceState(PRESENCE_ABSENT, now, "entry_weak_only");
        }
        break;
      }

      case PRESENCE_PRESENT:
        presentVotes = 5;
        absentVotes = 0;
        if (!(tickStrongEvidence || tickWeakEvidence)) {
          bool likelyRadarSilence = !departureMotionEvidence &&
                                    (lastStrongPresenceMs > 0) && (safeElapsedMs(now, lastStrongPresenceMs) <= 2000UL);
          enterPresenceState(likelyRadarSilence ? PRESENCE_SILENT_HOLD : PRESENCE_LEAVING,
                             now,
                             likelyRadarSilence ? "sudden_blackout" : "evidence_faded");
        }
        break;

      case PRESENCE_SILENT_HOLD:
        presentVotes = 5;
        absentVotes = (int)min(20UL, safeElapsedMs(now, presenceStateSinceMs) / 250UL);
        if (tickStrongEvidence || tickWeakEvidence) {
          enterPresenceState(PRESENCE_PRESENT, now, "evidence_resumed");
        } else if (departureMotionEvidence || (safeElapsedMs(now, presenceStateSinceMs) >= SILENT_HOLD_MS)) {
          enterPresenceState(PRESENCE_LEAVING, now,
                             departureMotionEvidence ? "departure_motion" : "silent_hold_timeout");
        }
        break;

      case PRESENCE_LEAVING:
        presentVotes = 0;
        absentVotes = (int)min(20UL, safeElapsedMs(now, presenceStateSinceMs) / 250UL);
        if (tickStrongEvidence || tickWeakEvidence) {
          enterPresenceState(PRESENCE_PRESENT, now, "recovered");
        } else if (safeElapsedMs(now, presenceStateSinceMs) >= LEAVE_CONFIRM_MS) {
          enterPresenceState(PRESENCE_ABSENT, now, "departure_confirmed");
          handlePersonLeft(now, departureMotionEvidence ? "motion_departure" : "timeout_departure");
        }
        break;
    }
    if ((presenceState == PRESENCE_PRESENT || presenceState == PRESENCE_SILENT_HOLD || presenceState == PRESENCE_LEAVING) &&
        falsePresenceStartMs > 0 && safeElapsedMs(now, falsePresenceStartMs) >= FALSE_PRESENCE_FORCE_EXIT_MS &&
        falsePresenceAbsenceScore >= FALSE_PRESENCE_ABSENCE_SCORE_EXIT) {
      enterPresenceState(PRESENCE_ABSENT, now, "hard_absence");
      handlePersonLeft(now, "hard_absence");
      falsePresenceStartMs = 0;
    }
  }

  // Backward-compatible boolean used by the rest of the firmware/UI/logging
  humanDetected = (presenceState == PRESENCE_PRESENT ||
                   presenceState == PRESENCE_SILENT_HOLD ||
                   presenceState == PRESENCE_LEAVING);

  // Claude P0-A fallback: if the higher-level presence FSM is confidently present,
  // force radarIsPresent to confirm so downstream publication logic can see it.
  if (!radarIsPresent && humanDetected && presentVotes >= 3) {
    radarIsPresent = true;
    radarPresentScore = (radarPresentScore > RAW_PRESENT_VOTES) ? radarPresentScore : RAW_PRESENT_VOTES;
    radarAbsentScore = 0;
    lastPresenceUpdateMs = now;
    if (safeElapsedMs(now, lastPresenceDebugLogMs) >= 250UL) {
      lastPresenceDebugLogMs = now;
      Serial.printf("[RADAR_PRES] fsm_fallback present | human=%d pv=%d av=%d state=%s dist=%.2f\n",
                    (int)humanDetected, presentVotes, absentVotes, presenceStateName(presenceState), lastGoodDistance);
    }
  } else if (radarIsPresent && !humanDetected && absentVotes >= 3) {
    radarIsPresent = false;
    radarAbsentScore = (radarAbsentScore > 3) ? radarAbsentScore : 3;
    if (safeElapsedMs(now, lastPresenceDebugLogMs) >= 250UL) {
      lastPresenceDebugLogMs = now;
      Serial.printf("[RADAR_PRES] fsm_fallback absent | human=%d pv=%d av=%d state=%s dist=%.2f\n",
                    (int)humanDetected, presentVotes, absentVotes, presenceStateName(presenceState), lastGoodDistance);
    }
  }
  // Preserve lastGoodDistance across brief absence so weak-distance evidence and
  // distance-based stabilization can recover cleanly on re-entry.

  if (presenceState == PRESENCE_PRESENT || presenceState == PRESENCE_SILENT_HOLD || presenceState == PRESENCE_LEAVING) {
    sessionUpdate(now, inMotion);
  }

  static unsigned long lastTempRead=0;
  static unsigned long lastLuxRead=0;
  bool allowSlowI2C = !radarSerialBacklogged();
  if (allowSlowI2C) {
    if (mlxReady && now-lastTempRead>=1000) {
      lastTempRead=now;
      float amb=NAN, obj=NAN;
      if (i2cSafeReadMLX(now, amb, obj)) {
        float obj_K=obj+273.15f,amb_K=amb+273.15f;
        float obj_K4=obj_K*obj_K*obj_K*obj_K,amb_K4=amb_K*amb_K*amb_K*amb_K;
        float corr_K4=obj_K4/MLX_EMISSIVITY+amb_K4*(1.0f-1.0f/MLX_EMISSIVITY);
        if (corr_K4>0.0f) obj=sqrtf(sqrtf(corr_K4))-273.15f; else obj=NAN;
        if (isfinite(obj)&&obj>-40.0f&&obj<85.0f) {
          if (isnan(smoothTemp)) smoothTemp=obj;
          else smoothTemp=safe_float(0.9f*smoothTemp+0.1f*obj);
        }
        if (isfinite(amb)) {
          if (isnan(ambientFastEma) || isnan(ambientSlowEma)) {
            ambientFastEma = amb;
            ambientSlowEma = amb;
            ambientDrift = 0.0f;
          } else {
            ambientFastEma = 0.70f * ambientFastEma + 0.30f * amb;
            ambientSlowEma = 0.95f * ambientSlowEma + 0.05f * amb;
            ambientDrift = fabsf(ambientFastEma - ambientSlowEma);
          }
          prevAmbient = amb;
          if (ambientDrift > 0.20f) lastAmbientJumpMs = now;
        }
      }
    }

    if (!bh1750Ready && (lastBh1750RetryMs == 0 || safeElapsedMs(now, lastBh1750RetryMs) >= BH1750_RETRY_INTERVAL_MS)) {
      lastBh1750RetryMs = now;
      i2cRecover();
      tryInitBH1750();
    }

    if (bh1750Ready&&now-lastLuxRead>=500) {
      lastLuxRead=now;
      i2cSafeReadLux(now);
    }
  }

  float rawReflectorDist = freshDistanceSampleThisFrame ? radarDistance : 0.0f; 

  if (!humanDetected) {
      if (noHumanClearStartMs == 0) noHumanClearStartMs = now;
      hrState = 0;
      radarDistance = 0.0f;
      hrConfidence *= 0.90f;
      pqiHeart *= 0.90f;
      pqiBreath *= 0.90f;
      if (hrConfidence < 0.01f) hrConfidence = 0.0f;
      if (safeElapsedMs(now, noHumanClearStartMs) >= NO_HUMAN_CLEAR_STATE_MS) {
        forceClearAllVitalState();
      }
  } else {
      noHumanClearStartMs = 0;
  }

  bool skipDSPForcedAbsent = calibrationDone && humanDetected &&
                             (skipDSP_consecMisses >= SKIPDSP_FORCED_ABSENT_THRESH) &&
                             (safeElapsedMs(now, lastPersonDetectedEventMs) >= SKIPDSP_FORCED_ABSENT_MIN_MS) &&
                             !livePhaseEvidence && !rawHRValid && !rawRRValid && !radarPresenceEvidence;
  if (skipDSPForcedAbsent) {
      enterPresenceState(PRESENCE_ABSENT, now, "skipdsp_starved");
      handlePersonLeft(now, "skipdsp_starved");
      humanDetected = false;
      radarIsPresent = false;
      radarIsPresentRaw = false;
      falsePresenceStartMs = 0;
      falsePresenceAbsenceScore = 0;
      noHumanClearStartMs = 0;
  }

  // =========================================================================
  // [v11.0.0] DUAL-PIPELINE VITAL GATING & INVALIDATION
  // =========================================================================

  bool phaseFreshNow = isPhaseFreshAt(now);
  bool trustedVitalFreshNow = isTrustedVitalFreshAt(now);
  bool trustedHrFreshNow = isTrustedHRFreshAt(now);
  bool trustedRrFreshNow = isTrustedRRFreshAt(now);
  bool hrFreshNow = isHRFreshAt(now);
  bool rrFreshNow = isRRFreshAt(now);
  bool distFreshNow = isDistanceFreshAt(now);
  if (livePhaseThisFrame) {
    if (phaseValidRunLen < 65535U) phaseValidRunLen++;
    phaseInvalidRunLen = 0;
  } else {
    if (phaseInvalidRunLen < 65535U) phaseInvalidRunLen++;
    phaseValidRunLen = 0;
  }
  if (hrUpdatedThisCycle && (hrUpdateSourceThisCycle == HR_PATH_AUTO || hrUpdateSourceThisCycle == HR_PATH_SPECTRAL)) {
    if (hrPhaseBackedUpdateCount < 65535U) hrPhaseBackedUpdateCount++;
  }
  if (rrPhaseBackedUpdateThisCycle && rrPhaseBackedUpdateCount < 65535U) {
    rrPhaseBackedUpdateCount++;
  }
  if (hrUpdateSourceThisCycle != HR_PATH_NONE) hrPathSourceLatched = hrUpdateSourceThisCycle;
  if (hrConfidenceSourceThisCycle != HR_CONF_NONE) hrConfidenceSourceLatched = hrConfidenceSourceThisCycle;
  agcFloorSuspect = calibrationDone &&
                    isfinite(radarGain) &&
                    (radarGain <= (MIN_GAIN + 0.01f)) &&
                    (skipDSP_consecMisses >= SKIP_DSP_MISS_THRESH);
  phaseBackedPublishReady = (hrState >= 2) && trustedHrFreshNow && (hrPhaseBackedUpdateCount >= 3);
  hrAnchorDriftSuspect = (hrState >= 2) &&
                         trustedHrFreshNow &&
                         (pqiHeart < 0.50f) &&
                         isfinite(smoothHR) &&
                         isfinite(trustedPhaseHR) &&
                         (trustedPhaseHR > 0.0f) &&
                         (fabsf(smoothHR - trustedPhaseHR) >= 8.0f);

  // 1. The Display Gate (10-second grace period)
  bool allowDisplayVitals = humanDetected && lastTrustedVitalMs > 0 && (safeElapsedMs(now, lastTrustedVitalMs) < 10000UL);

  // 2. The Logging Gate (Strict 2.5s grace + type-specific fresh evidence)
  bool inPublishWarmup = sessionStats.active && sessionStats.startMs > 0 && (safeElapsedMs(now, sessionStats.startMs) < PUBLISH_WARMUP_MS);
  if (lastCandidateHRMs > 0 && safeElapsedMs(now, lastCandidateHRMs) > CANDIDATE_MAX_AGE_MS) {
    candidateHR = 0.0f;
    candidateHRConf = 0.0f;
    lastCandidateHRMs = 0UL;
  }
  if (lastCandidateRRMs > 0 && safeElapsedMs(now, lastCandidateRRMs) > CANDIDATE_MAX_AGE_MS) {
    candidateRR = 0.0f;
    candidateRRConf = 0.0f;
    lastCandidateRRMs = 0UL;
  }

  bool candidateHRFresh = (lastCandidateHRMs > 0) && (safeElapsedMs(now, lastCandidateHRMs) < CANDIDATE_FRESH_MS) &&
                         (candidateHR > 0.0f && isfinite(candidateHR));
  bool candidateRRFresh = (lastCandidateRRMs > 0) && (safeElapsedMs(now, lastCandidateRRMs) < CANDIDATE_FRESH_MS) &&
                         (candidateRR > 0.0f && isfinite(candidateRR));
  bool haveFreshPhysioEvidence = livePhaseEvidence || rawHRValid || rawRRValid;
  bool haveFreshHREvidence = livePhaseEvidence || rawHRValid || latchedRawHRValidAt(now) || candidateHRFresh;
  bool haveFreshRREvidence = livePhaseEvidence || rawRRValid || candidateRRFresh;
  bool allowLoggedVitals = humanDetected && trustedVitalFreshNow && haveFreshPhysioEvidence && !inPublishWarmup;
  bool allowLoggedHRVitals = humanDetected && trustedHrFreshNow && haveFreshHREvidence && !inPublishWarmup;
  bool allowLoggedRRVitals = humanDetected && trustedRrFreshNow && haveFreshRREvidence && !inPublishWarmup;
  bool rrPostMotionHold = (lastMotionDetectedMs > 0) && (safeElapsedMs(now, lastMotionDetectedMs) < RR_POST_MOTION_HOLDOFF_MS);
  if (inMotion || rrPostMotionHold) {
    hrPhaseBackedUpdateCount = 0;
    rrPhaseBackedUpdateCount = 0;
    phaseBackedPublishReady = false;
  }

  // 3. State Downgrade (Aggressive UI invalidation if phase freezes)
  if (humanDetected && lastPhaseDataMs > 0 && (now - lastPhaseDataMs > PHASE_STALE_DOWNGRADE_MS)) {
      if (hrState > 1) hrState = 1;
  }

  // 4. Compute Separate Display vs Logged Output Variables
  bool displayHRValid = allowDisplayVitals && (hrState > 0) && hrFreshNow;
  bool displayRRValid = allowDisplayVitals && rrFreshNow;
  bool displayDistValid = allowDisplayVitals && distFreshNow;
  bool displayRRGrace = !displayRRValid &&
                        (lastValidDisplayRRMs > 0) &&
                        (safeElapsedMs(now, lastValidDisplayRRMs) < RR_DISPLAY_GRACE_MS);

  float displayHR = displayHRValid ? smoothHR : 0.0f;
  float displayRR = displayRRValid ? smoothRR : (displayRRGrace ? lastValidDisplayRR : 0.0f);
  float displayDist = displayDistValid ? lastGoodDistance : 0.0f;

  bool hrRawAvailableNow = rawHRValid || latchedRawHRValidAt(now);
  // v11.6.14: bypass subcondition telemetry — pure refactor, result identical
  hrBypassPqiOk  = (pqiHeart >= HR_LOG_PQI_BYPASS_MIN);
  hrBypassConfOk = (hrConfidence >= HR_LOG_CONF_BYPASS_MIN);
  hrBypassGateOk = (hrGateReason == HR_GATE_OK);
  bool hrBypassCandOk = (candidateHR > 0.0f && isfinite(candidateHR) &&
                         candidateHRConf >= HR_LOG_CAND_CONF_BYPASS_MIN);
  bool hrBypassRawOk  = (!hrRawAvailableNow || hrRawAgreementGood);

  bool hrHighConfidencePqiBypass = hrBypassPqiOk && hrBypassConfOk &&
                                   hrBypassGateOk && hrBypassCandOk &&
                                   hrBypassRawOk;
  hrBypassActive = hrHighConfidencePqiBypass;
  bool hrTrustedPublishFresh = (lastValidPublishHRMs > 0UL) && (safeElapsedMs(now, lastValidPublishHRMs) < HR_PUBLISH_GRACE_MS);
  bool hrPhaseBackedCandidateNow = ((hrUpdateSourceThisCycle == HR_PATH_AUTO) || (hrUpdateSourceThisCycle == HR_PATH_SPECTRAL)) &&
                                   (candidateHR > 0.0f && isfinite(candidateHR));
  if (!hrPhaseBackedCandidateNow) {
    hrLowerPersistWindows = 0;
    hrDownwardOverrideActive = false;
    hrRawHighBiasSuspectLogged = false;
  }
  bool hrGraceBlockCondition = hrPhaseBackedCandidateNow &&
                               hrLowMotionLockFirst() &&
                               hrTrustedPublishFresh &&
                               ((lastValidPublishHR - candidateHR) >= HR_BIAS_DELTA_BPM);
  if (hrGraceBlockCondition) {
    if (hrGraceBlockWindows < 255) hrGraceBlockWindows++;
  } else {
    hrGraceBlockWindows = 0;
  }
  hrPublishGraceBlocked = (hrGraceBlockWindows >= HR_PUBLISH_GRACE_BLOCK_WINDOWS);
  bool hrTrustedPublishNear = hrTrustedPublishFresh && (candidateHR > 0.0f && isfinite(candidateHR)) &&
                              (fabsf(candidateHR - lastValidPublishHR) <= HR_PUBLISH_GRACE_MAX_DELTA_BPM);
  bool hrPublishRawOk = (!hrRawAvailableNow || hrRawAgreementGood || hrUpdateSourceThisCycle == 4 || hrGateReason == HR_GATE_RAW_FALLBACK);
  bool hrPublishGrace = hrTrustedPublishFresh && !inMotion && hrFreshNow &&
                        (pqiHeart >= HR_PUBLISH_GRACE_MIN_PQI) &&
                        (hrConfidence >= HR_LOG_CONF_BYPASS_MIN) &&
                        (candidateHRConf >= HR_LOG_CAND_CONF_BYPASS_MIN) &&
                        !hrHarmonicAmbiguous && hrTrustedPublishNear &&
                        !hrPublishGraceBlocked &&
                        hrPublishRawOk;
  hrGraceEligible = hrTrustedPublishFresh;
  hrGraceActive   = hrPublishGrace;
  bool loggedHRQualityGate = ((pqiHeart >= HR_LOG_PQI_MIN) || hrHighConfidencePqiBypass || hrPublishGrace) &&
                             (hrConfidence >= 0.10f) &&
                             !hrHarmonicAmbiguous &&
                             hrPublishRawOk;

  bool rrEstimatorAgreeNow = rawRRValid && isfinite(rawRREffective) &&
                             candidateRR > 0.0f && isfinite(candidateRR) &&
                             fabsf(candidateRR - rawRREffective) <= 2.0f;
  if (rrPostMotionHold && rrEstimatorAgreeNow) {
    if (rrHoldAgreementCount < 255) rrHoldAgreementCount++;
  } else if (!rrPostMotionHold || !rrEstimatorAgreeNow) {
    rrHoldAgreementCount = 0;
  }
  bool rrPostMotionHoldActive = rrPostMotionHold && (rrHoldAgreementCount < 2);
  bool rrSourceLogOk = (rrGateReason == RR_GATE_OK);
  bool loggedRRQualityGate = (pqiBreath >= RR_PQI_LOG_THRESHOLD) &&
                             !rrPostMotionHoldActive &&
                             rrSourceLogOk;

  unsigned long hrAgeMs = (lastHRUpdateMs > 0) ? safeElapsedMs(now, lastHRUpdateMs) : 999999UL;
  bool hrFreshForLog = hrFreshNow && (hrAgeMs <= HR_PUBLISH_MAX_AGE_MS);
  bool loggedHRValid = allowLoggedHRVitals && phaseBackedPublishReady && hrFreshForLog && loggedHRQualityGate;
  bool loggedRRValid = allowLoggedRRVitals && phaseBackedPublishReady && rrFreshNow && loggedRRQualityGate;
  bool loggedDistValid = allowLoggedVitals && distFreshNow;

  if (loggedHRValid) hrPublishReason = HR_PUB_OK;
  else if (!humanDetected) hrPublishReason = HR_PUB_NO_HUMAN;
  else if (!trustedHrFreshNow) hrPublishReason = HR_PUB_TRUST_STALE;
  else if (inPublishWarmup) hrPublishReason = HR_PUB_STATE;
  else if (!haveFreshHREvidence) hrPublishReason = HR_PUB_NO_EVIDENCE;
  else if (!phaseBackedPublishReady) hrPublishReason = HR_PUB_STATE;
  else if (!hrFreshForLog) hrPublishReason = HR_PUB_AGE;
  else if (!hrFreshNow) hrPublishReason = HR_PUB_STALE;
  else if (hrPublishGraceBlocked && hrPhaseBackedCandidateNow) hrPublishReason = HR_PUB_GRACE_BLOCKED;
  else if (!((pqiHeart >= HR_LOG_PQI_MIN) || hrHighConfidencePqiBypass)) hrPublishReason = HR_PUB_PQI;
  else if (!(hrConfidence >= 0.10f)) hrPublishReason = HR_PUB_CONF;
  else if (hrHarmonicAmbiguous) hrPublishReason = HR_PUB_HARMONIC;
  else if (hrRawAvailableNow && !hrRawAgreementGood) hrPublishReason = HR_PUB_RAW_DISAGREE;
  else hrPublishReason = HR_PUB_OTHER;

  if (loggedRRValid) rrPublishReason = RR_PUB_OK;
  else if (!humanDetected) rrPublishReason = RR_PUB_NO_HUMAN;
  else if (!trustedRrFreshNow) rrPublishReason = RR_PUB_TRUST_STALE;
  else if (inPublishWarmup) rrPublishReason = RR_PUB_HOLDOFF;
  else if (!haveFreshRREvidence) rrPublishReason = RR_PUB_NO_EVIDENCE;
  else if (!phaseBackedPublishReady) rrPublishReason = RR_PUB_HOLDOFF;
  else if (!rrFreshNow) rrPublishReason = RR_PUB_STALE;
  else if (!(pqiBreath >= RR_PQI_LOG_THRESHOLD)) rrPublishReason = RR_PUB_PQI;
  else if (rrPostMotionHoldActive) rrPublishReason = RR_PUB_HOLDOFF;
  else if (!rrSourceLogOk) rrPublishReason = RR_PUB_SOURCE;
  else rrPublishReason = RR_PUB_OTHER;

  // HARD INVARIANT:
  // reported_hr must be zero whenever logged_hr_valid is false
  // reported_rr must be zero whenever logged_rr_valid is false
  float hrPublishCandidate = (isfinite(smoothHR) && smoothHR >= HR_MIN && smoothHR <= HR_MAX) ? smoothHR : 0.0f;
  float rrPublishCandidate = (isfinite(smoothRR) && smoothRR >= RR_MIN && smoothRR <= RR_MAX) ? smoothRR : 0.0f;
  float loggedHR = loggedHRValid ? hrPublishCandidate : 0.0f;
  float loggedRR = loggedRRValid ? rrPublishCandidate : 0.0f;
  float loggedDist = loggedDistValid ? lastGoodDistance : 0.0f;
  hrFinalPublishCandidate = hrPublishCandidate;
  rrFinalPublishCandidate = rrPublishCandidate;
  if (!trustedHrFreshNow && (lastHRUpdateMs == 0UL || safeElapsedMs(now, lastHRUpdateMs) > FINAL_PUBLISH_CANDIDATE_STALE_MS)) {
    hrFinalPublishCandidate = 0.0f;
  }
  if (!trustedRrFreshNow && (lastRRUpdateMs == 0UL || safeElapsedMs(now, lastRRUpdateMs) > FINAL_PUBLISH_CANDIDATE_STALE_MS)) {
    rrFinalPublishCandidate = 0.0f;
  }
  bool loggedHRViaGraceOnly = loggedHRValid && hrPublishGrace &&
                         !(pqiHeart >= HR_LOG_PQI_MIN) &&
                         !hrHighConfidencePqiBypass;
  if (loggedHRValid && isfinite(smoothHR) && smoothHR >= HR_MIN && smoothHR <= HR_MAX &&
      !loggedHRViaGraceOnly &&
      (hrUpdateSourceThisCycle == HR_PATH_AUTO || hrUpdateSourceThisCycle == HR_PATH_SPECTRAL)) {
    lastValidPublishHRMs = now;
    lastValidPublishHR = smoothHR;
  }

  {
    float governingAnchorLog = NAN;
    uint8_t governingAnchorSourceLog = HR_ANCHOR_NONE;
    currentGoverningHRAnchor(now, hrArbiterAnchorValue, hrRejectPhaseAnchorValue,
                             governingAnchorLog, governingAnchorSourceLog);
    hrTrustedPhaseAnchorLogged = isTrustedPhaseHRFreshAt(now) ? trustedPhaseHR : NAN;
    hrAnchorSourceLogged = governingAnchorSourceLog;
    hrAnchorErrBpmLogged = (candidateHR > 0.0f && isfinite(candidateHR) && isfinite(governingAnchorLog))
      ? fabsf(candidateHR - governingAnchorLog) : NAN;
  }

  // 5. Route Alerts to the Display Pipeline only
  if (allowDisplayVitals) {
      checkHRAlerts(displayHR, displayHRValid && hrState >= 2 && displayHR > 0.0f);
  } else {
      hrAlertActive = false;
  }

  // Optional runtime sanity check for future debugging
  if ((!loggedHRValid && loggedHR > 0.0f) || (loggedHRValid && loggedHR <= 0.0f)) {
      Serial.printf("[LOGIC] HR log invariant violated: loggedHRValid=%d loggedHR=%.2f\n",
                    (int)loggedHRValid, loggedHR);
  }
  if ((!loggedRRValid && loggedRR > 0.0f) || (loggedRRValid && loggedRR <= 0.0f)) {
      Serial.printf("[LOGIC] RR log invariant violated: loggedRRValid=%d loggedRR=%.2f\n",
                    (int)loggedRRValid, loggedRR);
  }

  if (hrGateReason == HR_GATE_OK && loggedHR == 0.0f && safeElapsedMs(now, lastGateBugLogMs) >= 1000UL) {
      lastGateBugLogMs = now;
      Serial.printf("[GATE_BUG] gate=OK but reported=0 | cand=%.1f pqi=%.3f conf=%.2f allow=%d hrState=%d fresh=%d human=%d radar=%d\n",
                    candidateHR, pqiHeart, hrConfidence, (int)allowLoggedVitals, hrState, (int)isHRFresh(),
                    (int)humanDetected, (int)radarIsPresent);
  }

  if (hrGateReason == HR_GATE_RAW_DISAGREE && safeElapsedMs(now, lastGateBugLogMs) >= 1000UL) {
      lastGateBugLogMs = now;
      float dbgRaw = rawHRValid ? rawHR : (latchedRawHRValidAt(now) ? latchedRawHR : 0.0f);
      Serial.printf("[HR_RAW_DISAGREE] cand=%.1f raw=%.1f err=%.1f pqi=%.3f conf=%.2f band=[%.1f,%.1f]\n",
                    candidateHR, dbgRaw, lastHRAgreementErr, pqiHeart, hrConfidence, lastLoggedHrBandMin, lastLoggedHrBandMax);
  }

  if (safeElapsedMs(now, lastHrCandidateLogMs) >= 1000UL) {
      lastHrCandidateLogMs = now;
      Serial.printf("[HR_DBG] reported=%.1f candidate=%.1f raw=%.1f gate=%s(%u) pqi=%.3f conf=%.2f allow=%d valid=%d human=%d radar=%d dist=%.2f band=[%.1f,%.1f] gain=%.3f\n",
                    loggedHR, candidateHR, rawHRValid ? rawHR : (latchedRawHRValidAt(now) ? latchedRawHR : 0.0f),
                    hrGateReasonName(hrGateReason), (unsigned)hrGateReason, pqiHeart, hrConfidence,
                    (int)allowLoggedVitals, (int)loggedHRValid, (int)humanDetected, (int)radarIsPresent, lastGoodDistance,
                    lastLoggedHrBandMin, lastLoggedHrBandMax, radarGain);
  }

  // =========================================================================

  if (!lcdConnected && (lastLcdRescanMs == 0 || safeElapsedMs(now, lastLcdRescanMs) >= 10000UL)) {
    lastLcdRescanMs = now;
    if (scanForLCD()) { prevDispState = DISP_NONE; lastDisplay = 0; }
  }

  if (lcdConnected&&now-lastDisplay>=DISPLAY_INTERVAL && !radarSerialBacklogged()) {
    if (!probeI2C(lcdAddr)) {
      lcdConnected = false;
      lcdPtr = nullptr;
      invalidateLcdRowCache();
      lastLcdRescanMs = now;
    } else {
      lastDisplay=now;
    if (dispState!=DISP_CALIB_RESULT&&dispState!=DISP_WELCOME&&dispState!=DISP_GOODBYE) {
      if (!calibrationDone) dispState=DISP_CALIBRATING;
      else if (humanDetected) dispState=DISP_VITALS;
      else dispState=DISP_IDLE;
    }
    bool stateChanged=(dispState!=prevDispState);
    if (stateChanged) {
      lcdSmoothTransition(); lcdPtr->clear(); invalidateLcdRowCache(); customCharsValid=false; lcdCreateChars();
      if (dispState == DISP_IDLE) lastIdleRedrawMs = 0;
      prevDispState=dispState; lastLedColor=0xFFFFFFFF;
      heartAnimFrame=false; animPending=false;
    }

    hrVisibleOnVitalsScreen = false;
    switch (dispState) {
      case DISP_CALIBRATING: renderCalibrationScreen(stateChanged); break;
      case DISP_CALIB_RESULT: break;
      case DISP_WELCOME: renderWelcomeScreen(); break;

      case DISP_VITALS: 
          {
              bool showHRFresh = allowDisplayVitals && isHRFresh() && hrState > 0 && smoothHR >= HR_MIN;
              bool showHRGrace = !showHRFresh &&
                                 lastHRDisplayedMs > 0 &&
                                 (now - lastHRDisplayedMs < HR_DISPLAY_GRACE_MS) &&
                                 lastDisplayedHR >= HR_MIN;
              bool showHR = showHRFresh || showHRGrace;
              hrVisibleOnVitalsScreen = showHR;
              bool hrGrace = showHRGrace;
              float hrForDisplay = showHRFresh ? smoothHR : (showHRGrace ? lastDisplayedHR : 0.0f);

              int iHR=showHR?(int)safe_float(hrForDisplay):0;
              int iRR=(int)safe_float(displayRR);
              bool hasTemp=!isnan(smoothTemp)&&isfinite(smoothTemp);
              int iTemp=hasTemp?constrain((int)lroundf(smoothTemp),-20,50):0;
              bool showRR=isRRFresh()&&iRR>=(int)RR_MIN;

              if (showHRFresh && iHR >= (int)HR_MIN) updateHRDisplayCache(now);

              char line[21];
              uint8_t row[LCD_COLS];

              lcdBuildBlankRow(row);
              row[0] = (uint8_t)CHAR_HEART;
              if (showHR&&iHR>=(int)HR_MIN) {
                if (hrGrace) snprintf(line,sizeof(line)," HR:~%3d %.4s  ",iHR,getHRZone((float)iHR));
                else if (hrState==2&&hrVarCount>=3) {
                  int iVar=constrain((int)lroundf(computeHRVar()),0,99);
                  snprintf(line,sizeof(line)," HR:%3d+/-%2d %.4s",iHR,iVar,getHRZone((float)iHR));
                } else snprintf(line,sizeof(line)," HR:%3d bpm  %.4s",iHR,getHRZone((float)iHR));
              } else snprintf(line,sizeof(line)," HR: ---  bpm");
              lcdCopyTextToRow(row, 1, 19, line);
              lcdWriteCachedRowRaw(0, row, stateChanged);

              lcdBuildBlankRow(row);
              row[0] = (uint8_t)CHAR_BREATH;
              if (showRR&&hasTemp) snprintf(line,sizeof(line)," RR:%2d brpm  %2d%cC",iRR,iTemp,byte(CHAR_DEGREE));
              else if (showRR) snprintf(line,sizeof(line)," RR:%2d brpm",iRR);
              else if (hasTemp) snprintf(line,sizeof(line)," RR:-- brpm  %2d%cC",iTemp,byte(CHAR_DEGREE));
              else snprintf(line,sizeof(line)," RR:-- brpm");
              lcdCopyTextToRow(row, 1, 19, line);
              lcdWriteCachedRowRaw(1, row, stateChanged);

              lcdBuildBlankRow(row);
              lcdCopyTextToRow(row, 0, 4, "Sig:");
              float signalQuality = (pqiHeart+pqiBreath)/2.0f;
              int dots = 0;
              if (signalQuality >= 0.80f) dots = 5;
              else if (signalQuality >= 0.60f) dots = 4;
              else if (signalQuality >= 0.40f) dots = 3;
              else if (signalQuality >= 0.20f) dots = 2;
              else if (signalQuality > 0.05f) dots = 1;
              for (int i = 0; i < 5; ++i) row[4 + i] = (uint8_t)(i < dots ? CHAR_DOT_FULL : CHAR_DOT_EMPTY);
              lcdCopyTextToRow(row, 9, 4, " St:");
              const char* statusMsg=inMotion?STATUS_MOVING:(presenceState==PRESENCE_SILENT_HOLD)?STATUS_HOLD:(presenceState==PRESENCE_LEAVING)?STATUS_EXIT:(hrState==2&&showHR)?STATUS_LOCKED
                :(hrState==1&&showHR)?STATUS_SENSING:STATUS_WAITING;
              lcdCopyTextToRow(row, 13, 4, statusMsg);
              lcdWriteCachedRowRaw(2, row, stateChanged);

              lcdBuildBlankRow(row);
              if (displayDist >= 30.0f && !inMotion) {
                  int distCm = (int)lroundf(displayDist);
                  snprintf(line, sizeof(line), "%3dcm", constrain(distCm, 0, 999));
              } else if (displayDist > 0.5f && !inMotion) {
                  snprintf(line, sizeof(line), "Near ");
              } else {
                  snprintf(line, sizeof(line), "---cm");
              }
              lcdCopyTextToRow(row, 0, 5, line);
              row[5] = ' ';

              const char* distZone = distanceZoneLabel(lastGoodDistance, isDistanceFresh());
              const char* msg = hrAlertActive ? "! ALERT !     " : inMotion ? "Hold Still    "
                : (presenceState==PRESENCE_SILENT_HOLD) ? "Radar Hold    "
                : (presenceState==PRESENCE_LEAVING) ? "Checking...   "
                : (strcmp(distZone, "TooClose")==0) ? "Move Back     "
                : (strcmp(distZone, "TooFar")==0 || strcmp(distZone, "Far")==0) ? "Move Closer   "
                : ghostSuspect ? "Improve Signal"
                : rrPostMotionHold ? "Breath Reset  "
                : rhcSuspect ? "Stay Still    "
                : (hrState==2&&showHR) ? "Locked OK     " : (hrState==1) ? "Measuring..   " : "Waiting..     ";
              lcdCopyTextToRow(row, 6, 14, msg);
              lcdWriteCachedRowRaw(3, row, stateChanged);
          }
          break;
      case DISP_GOODBYE: renderGoodbyeScreen(); break;
      case DISP_IDLE: renderIdleScreen(now); break;
      default: break;
    }
  }
  }
  if (lcdConnected&&dispState==DISP_VITALS) { updateHeartAnimation(); flushAnimPending(); }

  updateLED(inMotion);

  // Logging 
#if (LOG_MODE > 0)
  static unsigned long lastLogMs=0;
  static bool logHeaderPrinted=false;

  if (calibrationDone) {
    if (!logHeaderPrinted) {
      Serial.println("DATA,timestamp_ms,heart_phase_stabilized,breath_phase_stabilized,raw_hr,raw_rr,raw_rr_effective,raw_rr_likely_harmonic,"
                      "reported_hr,reported_rr,candidate_hr,candidate_rr,candidate_hr_conf,candidate_rr_conf,"
                      "pqi_heart,pqi_breath,hr_confidence,hr_gate_reason,rr_gate_reason,hr_gate_pqi_used,rr_gate_pqi_used,"
                      "hr_zc_bpm,hr_zc_conf,hr_spec_bpm,hr_spec_mag,hr_triple_agree,"
                      "rr_zc_bpm,rr_zc_conf,rr_spec_bpm,rr_spec_conf,rr_triple_agree,"
                      "reflector_distance_cm,reported_distance_cm,in_motion,human_detected,radar_is_present,hr_state,"
                      "ghost_suspect,dist_sd_cm,disp_state,dsp_task,present_votes,absent_votes,"
                      "phase_fresh,trusted_vital_fresh,logged_hr_valid,logged_rr_valid,"
                      "hr_publish_reason,rr_publish_reason,skipdsp_misses,"
                      "hr_band_min,hr_band_max,radar_gain,"
                      "hr_arbiter_corrected,hr_rejectphase_rejected,hr_coherence_rejected,"
                      "hr_raw_source,hr_raw_agree,hr_agree_err_bpm,"
                      "hr_bypass_pqi_ok,hr_bypass_conf_ok,hr_bypass_gate_ok,hr_bypass_active,"
                      "hr_grace_eligible,hr_grace_active,hr_trust_fresh,"
                      "rr_anchor_fresh,rr_outlier_persist,rr_raw_agree_ok,"
                      "rr_pre_acceptphase,rr_post_acceptphase,rr_post_blend,rr_post_bias_correction,rr_post_kalman,rr_final_publish_candidate,"
                      "rr_anchor_value,rr_anchor_age_ms,rr_anchor_source,rr_anchor_confidence,"
                      "rr_fundamental_recovery_count,rr_fundamental_recovery_triggered,rr_raw_seed_consistent_count,"
                      "rr_midsession_raw_reanchor_allowed,rr_midsession_raw_reanchor_blocked,rr_midsession_raw_reanchor_reason,rr_raw_anchor_err_bpm,"
                      "hr_pre_rejectphase,hr_post_rejectphase,hr_post_blend,hr_post_coherence,"
                      "hr_final_publish_candidate,"
                      "hr_arbiter_anchor_used,hr_arbiter_anchor_value,"
                      "hr_rejectphase_anchor_used,hr_rejectphase_anchor_value,"
                      "hr_raw_looks_like_half_rate,hr_trusted_anchor_value,"
                      "hr_age_ms,candidate_hr_age_ms,candidate_rr_age_ms,hr_updated_this_cycle,hr_update_source,latched_raw_hr,trusted_rr_fresh,radar_is_present_raw,"
                      "harmonic_mode,session_phase,"
                      "point_cloud_ok,target_info_ok,num_targets,max_dop_abs,max_dop_speed_cms,doppler_motion,cluster_anomaly,multi_target,"
                      "primary_x,primary_y,primary_dop,primary_dop_speed_cms,primary_cluster,spatial_source,spatial_age_ms,position_radius_cm,"
                      "phase_warmup_complete,clutter_warmup_count,current_clutter_alpha,use_fast_path,phase_valid_this_frame,dsp_ran_this_frame,hr_confidence_source,hr_path_source,module_fw_major,module_fw_sub,module_fw_mod,sketch_major,sketch_sub,sketch_mod,"
                      "hr_trusted_phase_anchor,hr_anchor_source,hr_anchor_err_bpm,hr_raw_high_bias_suspect,rr_seed_from_raw_used,"
                      "trusted_hr_fresh,allow_logged_hr_vitals,allow_logged_rr_vitals,logged_hr_quality_gate,logged_rr_quality_gate,phase_valid_run_len,phase_invalid_run_len,hr_phase_backed_update_count,rr_phase_backed_update_count,"
                      "near_field_reflector_suspect,agc_floor_suspect,phase_backed_publish_ready,hr_anchor_drift_suspect,phase_gap_fill_count,clutter_rewarm_count,experimental_profile_enabled");
      logHeaderPrinted=true;
    }

    if (now-lastLogMs>=LOG_INTERVAL_MS) {
      lastLogMs=now;
      int phaseFresh_i        = phaseFreshNow ? 1 : 0;
      int trustedVitalFresh_i = trustedVitalFreshNow ? 1 : 0;
      int trustedHrFresh_i    = trustedHrFreshNow ? 1 : 0;
      int trustedRrFresh_i    = trustedRrFreshNow ? 1 : 0;
      hrAgeMsLogged = (lastHRUpdateMs > 0) ? safeElapsedMs(now, lastHRUpdateMs) : 999999UL;
      candidateHrAgeMsLogged = (lastCandidateHRMs > 0) ? safeElapsedMs(now, lastCandidateHRMs) : 999999UL;
      candidateRrAgeMsLogged = (lastCandidateRRMs > 0) ? safeElapsedMs(now, lastCandidateRRMs) : 999999UL;
      int loggedHrValid_i     = loggedHRValid ? 1 : 0;
      int loggedRrValid_i     = loggedRRValid ? 1 : 0;
      uint8_t hrConfidenceSourceLogged = (hrConfidenceSourceThisCycle != HR_CONF_NONE) ? hrConfidenceSourceThisCycle : hrConfidenceSourceLatched;
      uint8_t hrPathSourceLogged = (hrUpdateSourceThisCycle != HR_PATH_NONE) ? hrUpdateSourceThisCycle : hrPathSourceLatched;
      bool rrRawAgreeOk = rawRRValid && isfinite(rawRREffective) &&
                          candidateRR > 0.0f && isfinite(candidateRR) &&
                          fabsf(candidateRR - rawRREffective) <= 2.0f;
      uint8_t harmonicModeLogged = 0;
      if (rawRRLikelyHarmonic)                  harmonicModeLogged |= HARM_MODE_RR_RAW_HARMONIC_CORRECTED;
      if (hrArbiterCorrectedThisWindow)         harmonicModeLogged |= HARM_MODE_HR_ARBITER_CORRECTED;
      if (hrRawLooksLikeHalfRateLogged)         harmonicModeLogged |= HARM_MODE_HR_RAW_LOOKED_HALF_RATE;
      if (hrHarmonicAmbiguous)                  harmonicModeLogged |= HARM_MODE_HR_HARMONIC_AMBIGUOUS;
      if (rrGateReason == RR_GATE_SUBHARMONIC)  harmonicModeLogged |= HARM_MODE_RR_SUBHARMONIC_REJECTED;
      if (rrGateReason == RR_GATE_HARMONIC)     harmonicModeLogged |= HARM_MODE_RR_HARMONIC_REJECTED;

      uint8_t sessionPhaseLogged = SESSION_PHASE_ABSENT;
      bool inWarmupPhase = sessionStats.active && sessionStats.startMs > 0 &&
                           (safeElapsedMs(now, sessionStats.startMs) < PUBLISH_WARMUP_MS);
      bool postMotionPhase = (lastMotionDetectedMs > 0) &&
                             (safeElapsedMs(now, lastMotionDetectedMs) < RR_POST_MOTION_HOLDOFF_MS);
      if (presenceState == PRESENCE_LEAVING) sessionPhaseLogged = SESSION_PHASE_LEAVING;
      else if (!humanDetected)               sessionPhaseLogged = SESSION_PHASE_ABSENT;
      else if (inWarmupPhase)                sessionPhaseLogged = SESSION_PHASE_WARMUP;
      else if (inMotion || postMotionPhase)  sessionPhaseLogged = SESSION_PHASE_POST_MOTION;
      else if (hrState >= 2 && isHRFresh())  sessionPhaseLogged = SESSION_PHASE_LOCKED;
      else                                   sessionPhaseLogged = SESSION_PHASE_SETTLING;

      // CSV column count widened for v13.9a observability / branch-stable schema telemetry.
      // Expected DATA payload column count after the DATA, prefix: 157. Update this comment whenever columns are added or removed.
      #define CSV_COLUMN_COUNT 157
#define CSVU(v) do { Serial.print((unsigned long)(v)); Serial.print(','); } while (0)
#define CSVI(v) do { Serial.print((int)(v)); Serial.print(','); } while (0)
#define CSVF(v,p) do { float __csv_v = (float)(v); Serial.print(isfinite(__csv_v) ? __csv_v : -1.0f, (p)); Serial.print(','); } while (0)
#define CSVFN(v,p) do { float __csv_v = (float)(v); Serial.print(isfinite(__csv_v) ? __csv_v : -1.0f, (p)); Serial.print(','); } while (0)
      Serial.print("DATA,");
      CSVU(now);
      CSVF(prevStableHeartPhase, 5);
      CSVF(prevStableBreathPhase, 5);
      CSVF(rawHRValid ? rawHR : 0.0f, 2);
      CSVF(rawRRValid ? rawRR : 0.0f, 2);
      CSVF(rawRRValid ? rawRREffective : 0.0f, 2);
      CSVI((int)rawRRLikelyHarmonic);
      CSVF(loggedHR, 2);
      CSVF(loggedRR, 2);
      CSVF(candidateHR, 2);
      CSVF(candidateRR, 2);
      CSVF(candidateHRConf, 4);
      CSVF(candidateRRConf, 4);
      CSVF(pqiHeart, 4);
      CSVF(pqiBreath, 4);
      CSVF(hrConfidence, 4);
      CSVI((unsigned)hrGateReason);
      CSVI((unsigned)rrGateReason);
      CSVF(hrGatePqiUsed, 4);
      CSVF(rrGatePqiUsed, 4);
      CSVF(hrZcBpmLogged, 2);
      CSVF(hrZcConfLogged, 3);
      CSVF(hrSpecBpmLogged, 2);
      CSVF(hrSpecMagLogged, 5);
      CSVI((int)hrTripleAgreeLogged);
      CSVF(rrZcBpmLogged, 2);
      CSVF(rrZcConfLogged, 3);
      CSVF(rrSpecBpmLogged, 2);
      CSVF(rrSpecConfLogged, 3);
      CSVI((int)rrTripleAgreeLogged);
      CSVF(rawReflectorDist, 3);
      CSVF(loggedDist, 3);
      CSVI((int)inMotion);
      CSVI((int)humanDetected);
      CSVI((int)radarIsPresent);
      CSVI(hrState);
      CSVI((int)ghostSuspect);
      CSVF(currentDistStdDev, 3);
      CSVI((int)dispState);
      CSVI(dspTask);
      CSVI(presentVotes);
      CSVI(absentVotes);
      CSVI(phaseFresh_i);
      CSVI(trustedVitalFresh_i);
      CSVI(loggedHrValid_i);
      CSVI(loggedRrValid_i);
      CSVI((unsigned)hrPublishReason);
      CSVI((unsigned)rrPublishReason);
      CSVI((unsigned)skipDSP_consecMisses);
      CSVF(lastLoggedHrBandMin, 1);
      CSVF(lastLoggedHrBandMax, 1);
      CSVF(radarGain, 3);
      CSVI((int)hrArbiterCorrectedThisWindow);
      CSVI((int)hrRejectPhaseRejectedThisWindow);
      CSVI((int)hrCoherenceRejectedThisWindow);
      CSVI((int)hrRawSourceThisWindow);
      CSVI((int)hrRawAgreementGood);
      CSVF(isfinite(lastHRAgreementErr) ? lastHRAgreementErr : -1.0f, 2);
      CSVI((int)hrBypassPqiOk);
      CSVI((int)hrBypassConfOk);
      CSVI((int)hrBypassGateOk);
      CSVI((int)hrBypassActive);
      CSVI((int)hrGraceEligible);
      CSVI((int)hrGraceActive);
      CSVI(trustedHrFresh_i);
      CSVI((int)rrAnchorFreshLogged);
      CSVI((unsigned)rrOutlierPersistCount);
      CSVI((int)rrRawAgreeOk);
      CSVF(rrPreAcceptPhase, 2);
      CSVF(rrPostAcceptPhase, 2);
      CSVF(rrPostBlend, 2);
      CSVF(rrPostBiasCorrection, 2);
      CSVF(rrPostKalman, 2);
      CSVF(rrFinalPublishCandidate, 2);
      CSVF(rrAnchorValueLogged, 2);
      CSVU(rrAnchorAgeMsLogged);
      CSVI((unsigned)rrAnchorSourceLogged);
      CSVF(rrAnchorConfidenceLogged, 3);
      CSVI((unsigned)rrFundamentalRecoveryCountLogged);
      CSVI((int)rrFundamentalRecoveryTriggeredLogged);
      CSVI((unsigned)rrRawSeedConsistentCountLogged);
      CSVI((int)rrMidSessionRawReanchorAllowedLogged);
      CSVI((int)rrMidSessionRawReanchorBlockedLogged);
      CSVI((unsigned)rrMidSessionRawReanchorReasonLogged);
      CSVF(rrRawAnchorErrBpmLogged, 2);
      CSVF(hrPreRejectPhase, 2);
      CSVF(hrPostRejectPhase, 2);
      CSVF(hrPostBlend, 2);
      CSVF(hrPostCoherence, 2);
      CSVF(hrFinalPublishCandidate, 2);
      CSVI((int)hrArbiterAnchorUsed);
      CSVF(hrArbiterAnchorValue, 2);
      CSVI((int)hrRejectPhaseAnchorUsed);
      CSVF(hrRejectPhaseAnchorValue, 2);
      CSVI((int)hrRawLooksLikeHalfRateLogged);
      CSVF(hrTrustedAnchorValue, 2);
      CSVU(hrAgeMsLogged);
      CSVU(candidateHrAgeMsLogged);
      CSVU(candidateRrAgeMsLogged);
      CSVI((int)hrUpdatedThisCycle);
      CSVI((unsigned)hrUpdateSourceThisCycle);
      CSVF(latchedRawHRValidAt(now) ? latchedRawHR : 0.0f, 2);
      CSVI(trustedRrFresh_i);
      CSVI((int)radarIsPresentRaw);
      CSVI((unsigned)harmonicModeLogged);
      CSVI((unsigned)sessionPhaseLogged);
      CSVI((int)lastPointCloudValid);
      CSVI((int)lastTargetInfoValid);
      CSVI(numDetectedTargets);
      CSVF(maxDopplerAbs, 2);
      CSVF(maxDopplerSpeedCms, 2);
      CSVI((int)dopplerMotionDetected);
      CSVI((int)clusterAnomaly);
      CSVI((int)multiTargetDetected);
      float positionRadiusCm = (isfinite(primaryTargetX) && isfinite(primaryTargetY) && numDetectedTargets >= 1) ? sqrtf(primaryTargetX * primaryTargetX + primaryTargetY * primaryTargetY) * 100.0f : 0.0f;
      CSVF(primaryTargetX, 3);
      CSVF(primaryTargetY, 3);
      CSVI((int)primaryTargetDop);
      CSVF(primaryTargetSpeedCms, 2);
      CSVI((int)primaryTargetCluster);
      CSVI((int)spatialSource);
      CSVU(spatialFreshAgeMs);
      CSVF(positionRadiusCm, 2);
      // v13.9c truthfulness / readiness / drift instrumentation
      CSVI((int)phaseWarmupComplete);
      CSVI(clutterWarmupCount);
      CSVF(currentClutterAlpha, 4);
      CSVI((int)useDirectRawHR);
      CSVI((int)livePhaseThisFrame); // phase_valid_this_frame
      CSVI((int)dspRanThisFrame);
      CSVI((unsigned)hrConfidenceSourceLogged);
      CSVI((unsigned)hrPathSourceLogged);
      CSVI(moduleVersionValid ? (int)moduleVersion.firmware_verson.major_version : -1);
      CSVI(moduleVersionValid ? (int)moduleVersion.firmware_verson.sub_version : -1);
      CSVI(moduleVersionValid ? (int)moduleVersion.firmware_verson.modified_version : -1);
      CSVI(SKETCH_VERSION_MAJOR);
      CSVI(SKETCH_VERSION_SUB);
      CSVI(SKETCH_VERSION_MOD);
      CSVF(isfinite(hrTrustedPhaseAnchorLogged) ? hrTrustedPhaseAnchorLogged : -1.0f, 2);
      CSVI((unsigned)hrAnchorSourceLogged);
      CSVF(isfinite(hrAnchorErrBpmLogged) ? hrAnchorErrBpmLogged : -1.0f, 2);
      CSVI((int)hrRawHighBiasSuspectLogged);
      CSVI((int)(rrSeedConsistentCount >= 3 ? 1 : 0));
      CSVI(trustedHrFresh_i);
      CSVI((int)allowLoggedHRVitals);
      CSVI((int)allowLoggedRRVitals);
      CSVI((int)loggedHRQualityGate);
      CSVI((int)loggedRRQualityGate);
      CSVU(phaseValidRunLen);
      CSVU(phaseInvalidRunLen);
      CSVU(hrPhaseBackedUpdateCount);
      CSVU(rrPhaseBackedUpdateCount);
      CSVI((int)nearFieldReflectorSuspect);
      CSVI((int)agcFloorSuspect);
      CSVI((int)phaseBackedPublishReady);
      CSVI((int)hrAnchorDriftSuspect);
      CSVU(phaseGapFillCount);
      CSVU(clutterRewarmCount);
      Serial.println((int)experimentalProfileEnabled);
#undef CSV_COLUMN_COUNT
#undef CSVU
#undef CSVI
#undef CSVF
#undef CSVFN
    }
  }
#endif

  vTaskDelay(1);
}
