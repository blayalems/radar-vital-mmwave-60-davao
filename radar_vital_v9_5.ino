/* radar_vital_v9_5.ino
 *
 * XIAO ESP32-C6 + MR60BHA2 60 GHz FMCW radar + MLX90614 + HD44780 LCD
 *
 * Change log from v9.4 → v9.5:
 * +---------+-----------------------------------+----------------------------------------------+
 * | ID      | Location                          | Change                                       |
 * +---------+-----------------------------------+----------------------------------------------+
 * | FIX-5   | loop() presence SM                | radarPresenceKeepAlive suppressed when no     |
 * |         |                                   | valid HR/RR for 15 s → fixes "NO HUMAN       |
 * |         |                                   | DETECTED" never reappearing after first lock. |
 * | FIX-6   | Global + setup() + loop() MLX     | I²C bus recovery: 9-SCL-pulse bit-bang per    |
 * |         |                                   | I²C spec §3.1.16. Re-inits LCD + BH1750      |
 * |         |                                   | after bus reset. Replaces bare Wire.end/begin.|
 * | FIX-7   | Global + loop() DSP outputs       | SAFE_FLOAT macro guards Kalman/fusion outputs |
 * |         |                                   | against NaN propagation (permanent corruption)|
 * | FIX-8   | Global + loop() newData path      | Consecutive bad-frame counter: 20 invalid     |
 * |         |                                   | frames → serial buffer flush for resync.      |
 * | FIX-9   | loop() newData radar getters      | mmWave.getHeartRate/getBreathRate/getDistance  |
 * |         |                                   | return values checked; stale data rejected.   |
 * | FIX-10  | Global + loop() calibration       | 30 s calibration timeout: default gain=1.0    |
 * |         |                                   | if sensor never delivers 100 valid phases.    |
 * | FIX-11  | setup() LCD scan                  | LCD address scan skips 0x23 if BH1750 claimed |
 * |         |                                   | it, preventing I²C address collision.         |
 * | FIX-12  | loop() MLX read block             | MLX read + ambientEvidence moved outside      |
 * |         |                                   | if(newData) so temp is polled during radar     |
 * |         |                                   | dropouts.                                     |
 * | FIX-13  | loop() motion detection            | Buffer flush on motion→still transition:      |
 * |         |                                   | bufIndex/bufCount zeroed, memset buffers.      |
 * |         |                                   | Eliminates ~25 s of stale post-motion data.   |
 * | FIX-14  | resetVitals()                     | rawHR, rawRR, rawHRWin[] now zeroed on        |
 * |         |                                   | patient departure. Prevents stale-data carry- |
 * |         |                                   | over to next patient.                         |
 * | FIX-15  | loop() dspTask==0                 | ghostSuspect / rhcSuspect now cleared when    |
 * |         |                                   | the triggering condition no longer holds,      |
 * |         |                                   | instead of requiring a motion event.           |
 * | FIX-16  | loop() motionCooldown             | motionCooldown only decrements when !inMotion.|
 * |         |                                   | Prevents premature cooldown expiry during     |
 * |         |                                   | sustained motion.                             |
 * | FIX-17  | Global + loop() MLX read          | Software emissivity correction (ε=0.98 for    |
 * |         |                                   | human skin). Avoids EEPROM wear from          |
 * |         |                                   | writeEmissivity(). ≈0.24 °C correction at     |
 * |         |                                   | 12 °C differential.                           |
 * | FIX-18  | loop() phaseDelta                 | phaseDelta wrapped to [-π,π] before fabsf to  |
 * |         |                                   | prevent false 6.28 rad spike on 2π→0 wrap.    |
 * | FIX-19  | loop() wasMotion                  | wasMotion updated only on newData frames to   |
 * |         |                                   | prevent edge-detection erasure between frames. |
 * | PROD-1  | setup() + loop()                  | Explicit ESP32-C6 task watchdog: loopTask      |
 * |         |                                   | subscribed to TWDT. Fed at top of every loop().|
 * | PROD-2  | Global + loop() presence SM       | NVS persistence via Preferences: saves radar- |
 * |         |                                   | Gain + KF priors on absence, restores on next  |
 * |         |                                   | presence with bounds-validated values.         |
 * | PROD-3  | Global + setup() + loop() LCD     | Rolling 10-sample HR stddev: "HR 72±3" with   |
 * |         |                                   | custom ± glyph. Temp drops to integer to free  |
 * |         |                                   | LCD columns.                                  |
 * +---------+-----------------------------------+----------------------------------------------+
 *
 * Change log from v9.3 → v9.4:
 * +---------+---------------------------------+------------------------------------------------+
 * | ID      | Location                        | Change                                         |
 * +---------+---------------------------------+------------------------------------------------+
 * | TEL-1   | Global + loop() presenceEvidence| isHumanDetected() polled each newData frame.   |
 * |         |                                  | Stored as radarIsPresent. Feeds presenceEvidence|
 * |         |                                  | ONLY when humanDetected is already true, so it |
 * |         |                                  | cannot create false presence but CAN prevent    |
 * |         |                                  | false-absent when a very still patient stops    |
 * |         |                                  | producing extractable HR/RR rates.             |
 * | TEL-2   | Global + loop() dspTask 1+2     | getDistance() polled each newData frame.        |
 * |         |                                  | Soft 0.7x hrConfidence penalty when distance   |
 * |         |                                  | is non-zero AND outside [0.3, 1.5] m.          |
 * |         |                                  | radarDistance == 0 = no target tracked: no     |
 * |         |                                  | penalty applied (prevents false-penalizing).   |
 * | TEL-3   | Global + loop() roughMotion     | unusedPhase renamed totalPhase; phaseDelta =   |
 * |         |                                  | |totalPhase - prevTotalPhase|. Threshold 0.50  |
 * |         |                                  | rad (above normal breathing ~0.15 rad/frame).  |
 * |         |                                  | ORed into roughMotion for early gross-motion   |
 * |         |                                  | gate. Zero additional UART cost.               |
 * | TEL-D   | loop() (optional)               | #define DEBUG_TELEMETRY enables 1 Hz Serial    |
 * |         |                                  | telemetry line. Off by default.                |
 * +---------+---------------------------------+------------------------------------------------+
 *
 * 9-step proposal — rejected steps with technical reasons:
 *   REJECT Step 4 (breath quality)   : radarBreathRate == rawHR; RR_MIN/RR_MAX already gate it.
 *   REJECT Step 5 (radar HR fusion)  : radarHeartRate == rawHR; already fused in DSP slots 1+2.
 *   REJECT Step 6 (dop_index speed)  : field absent from confirmed struct (x_point/y_point only);
 *                                       std::vector heap alloc at 10 Hz causes fragmentation;
 *                                       gross motion already covered by EMA motionDetected().
 *   REJECT Step 7 (breath-phase gate): already in v9.3 as SS-4 (atan2+lut_sin gate on dhGated).
 *   REJECT HR_valid/RR_valid vars    : dead code with no consumers in DSP pipeline.
 *
 * Change log from v9.2 → v9.3:
 * +---------+---------------------------------+------------------------------------------------+
 * | ID      | Location                        | Change                                         |
 * +---------+---------------------------------+------------------------------------------------+
 * | HW-1    | Global NeoPixel declaration     | Pin changed from hardcoded 8 (no onboard        |
 * |         |                                  | NeoPixel on XIAO ESP32-C6) to D1 (external      |
 * |         |                                  | WS2812 on GPIO D1). Using Arduino-alias D1      |
 * |         |                                  | keeps the board-package pin mapping canonical.  |
 * | HW-2    | setup() BH1750 init             | Added address-fallback: try 0x5C (ADDR=HIGH)    |
 * |         |                                  | first; if begin() returns false, retry at 0x23  |
 * |         |                                  | (ADDR=LOW/float). Suppresses "[BH1750] ERROR:   |
 * |         |                                  | other error" when ADDR pin is not tied HIGH.    |
 * |         |                                  | bh1750Addr global records which address worked  |
 * |         |                                  | so the boot log is informative.                 |
 * +---------+---------------------------------+------------------------------------------------+
 *
 * Multi-panel audit reconciliation (v9.2 already contained):
 *   ✔ Task-slicer double-filter fix (coherenceFilter/kalmanHR only in lastAutoValid path)
 *   ✔ inMotion declared at top of loop() — no scope hazard
 *   ✔ SG warmup guard (sgWarmup < SG_WIN before savitzkyGolay)
 *   ✔ LUT negative-index safety (% + if<0 correction)
 *   ✔ detectRate count<=1 guard before avg division
 *   ✔ motionLP floor (max(..., 1e-6f))
 *   ✔ breathInstPhase epsilon (db + 1e-6f)
 *   ✔ LUT forward declarations before respirationLockedClean
 *   ✔ Breath gate atan2 args corrected (smoothedB, db) + lut_sin (SS-4)
 *   ✔ Stale PQI mitigated by IMP-11 0.90x decay per non-task-0 frame
 *   ✔ motionDetected called exactly once per frame
 *   ✔ pqiHeart not squared in wDSP (wDSP = hrConfidence, not * pqiHeart)
 *   ✔ No orphaned harmonicScore() function present
 *
 * Change log from v9.1 → v9.2:
 * +---------+---------------------------------+------------------------------------------------+
 * | ID      | Location                        | Change                                         |
 * +---------+---------------------------------+------------------------------------------------+
 * | GHO-1   | rawHRGhostSuspect() + dspTask   | Ghost HR detector: stddev(rawHRWin) < 1.0 BPM  |
 * |         | 0/1 + motion rising edge        | over ≥8 samples → suspectGhost flag.           |
 * |         |                                  | Slot 0: blocks fftVote (mechanical ghosts lie  |
 * |         |                                  | exactly on harmonics). Slot 1: hrConfidence    |
 * |         |                                  | ×0.5, hrState capped at SEN (never LCK).       |
 * |         |                                  | rawHRWinCount reset on motion onset to flush   |
 * |         |                                  | stale window. Fixes rawHRStability() rewarding |
 * |         |                                  | ghosts with maximum fusion weight.             |
 * | GHO-2   | dspTask slot 1                  | pqiBreath < 0.20 → hrConfidence × 0.70.        |
 * |         |                                  | hrConfidence = lastConfHR×pqiHeart previously  |
 * |         |                                  | ignored breath quality entirely; ghost with    |
 * |         |                                  | no breath signal could still reach LCK.        |
 * | RHC-1   | dspTask slot 0 + static flag    | Respiration harmonic capture guard: after HR   |
 * |         |                                  | is resolved, check if hrAuto ≈ N×smoothRR      |
 * |         |                                  | (N=2..5, ±3 BPM, only when smoothRR ≤ 20).    |
 * |         |                                  | Hit → confHR × 0.6, rhcSuspect=true → slot 1  |
 * |         |                                  | caps hrState at SEN. respirationLockedClean    |
 * |         |                                  | removes signal harmonics but cannot prevent    |
 * |         |                                  | autocorr/Goertzel locking onto them.           |
 * | BH1-1   | Global + setup() + loop()       | BH1750 ambient light sensor at 0x5C (ADDR pin  |
 * |         |                                  | high, avoids LCD collision at 0x23). Night     |
 * |         |                                  | mode: lux < 10 → display interval × 4 and     |
 * |         |                                  | LED dimmed to 10% brightness. Polled 1/5s.     |
 * | WS2-1   | Global + setup() + updateLED()  | WS2812 RGB LED on GPIO8 (XIAO ESP32-C6         |
 * |         |                                  | onboard). PQI colour gradient red→yellow→green.|
 * |         |                                  | Green pulses at smoothHR rate when LCK.        |
 * |         |                                  | Red flash on motion. Yellow-breathe on search. |
 * |         |                                  | Orange if ghostSuspect or rhcSuspect.          |
 * |         |                                  | Boot sequence: blue→cyan→yellow→green.         |
 * +---------+---------------------------------+------------------------------------------------+
 *
 * Change log from v9.0 → v9.1:
 * +---------+---------------------------------+------------------------------------------------+
 * | ID      | Location                        | Change                                         |
 * +---------+---------------------------------+------------------------------------------------+
 * | FIX-1   | loop() LCD snprintf             | %5.1f → %4.1f; T field was 17 chars, overflowed|
 * |         |                                  | col 0 of line 2, truncating display at 28.xx   |
 * | FIX-2   | loop() newData gate             | Remove early return on !newData; wrap sensor   |
 * |         |                                  | reads + DSP in if(newData){}; presence SM and  |
 * |         |                                  | LCD now always execute. Radar dropout during   |
 * |         |                                  | motion/out-of-range no longer freezes          |
 * |         |                                  | humanDetected state permanently.               |
 * |         |                                  | radarEvidence gated on newData so stale rawHR  |
 * |         |                                  | cannot sustain false presence after dropout.   |
 * | FIX-3   | setup()                         | Wire.setTimeOut(500) — prevents indefinite     |
 * |         |                                  | I2C hang if MLX or LCD holds SDA low           |
 * | FIX-4   | loop() MLX error path           | Wire.end()/Wire.begin()/Wire.setTimeOut(500)   |
 * |         |                                  | after 3 consecutive MLX errors; recovers hung  |
 * |         |                                  | I2C bus rather than just flagging mlxReady=false|
 * +---------+---------------------------------+------------------------------------------------+
 *
 * REJECTED panel suggestions (with reasons):
 *   SG warmup < → <=       : WRONG. Increment is inside the if-block; frame 6 fires clean.
 *   NLMS mu "backwards"     : WRONG. 0.10→0.01 is standard (Haykin). Fast conv → low noise.
 *   Goertzel overflow        : NOT POSSIBLE. Max s0²=16384, float32 max=3.4e38.
 *   BUF_SIZE 256→128        : BREAKS LCK. WIN_FULL=160 requires BUF_SIZE > 160.
 *   UART0 USB conflict       : FACTUALLY WRONG for ESP32-C6. USB uses native USB peripheral.
 *   Watchdog explicit feed   : UNNECESSARY. Arduino idle task feeds TWDT; DSP slices < 6µs.
 *   millis() rollover        : ALREADY SAFE. Unsigned subtraction is modular by C standard.
 *   Emissivity writeEEPROM   : PERMANENT hardware write; not firmware scope for this version.
 *   FreeRTOS refactor        : Out of scope. Round-robin DSP slicer already handles timing.
 *   micros() for phaseTimes  : 1ms res = 1% fs error at 10 Hz. Negligible.
 *   Remove ambientEvidence   : CONFIRM_VOTES=5 hysteresis prevents HVAC false positives.
 *
 * Change log from v8.8 → v9.0:
 * +---------+---------------------------------+------------------------------------------------+
 * | SS-1    | Global + resetVitals + loop      | notchEnabled guard prevents startup notch at   |
 * |         |                                  | 45/60 BPM                                      |
 * | SS-2    | resetVitals()                    | bufIndex=0, bufCount=0, memset×4 circular bufs |
 * | SS-3    | loop() structure                 | phaseDataValid flag; presence SM runs always    |
 * | SS-4    | loop() breath gate               | lut_cos→lut_sin (breath gate regression fix)   |
 * | IMP-1   | autocorr()                       | Divide by (n-lag) for unbiased estimation      |
 * | IMP-2   | New goertzel() + 3 spectral fns  | Replace brute-force DFT inner loops            |
 * | IMP-3   | dspTask switch                   | Add slot 3 for RR; remove standalone RR block  |
 * | IMP-4   | respirationLockedClean synthesis  | fmodf phase wrapping                           |
 * | IMP-5   | Notch update condition            | Hysteresis 0.5→1.5 BPM                        |
 * | IMP-6   | nlmsUpdate + resetVitals          | Adaptive mu annealing (0.10→0.01)              |
 * | IMP-7   | Global + loop + resetVitals       | Remove prevBreathSample dead variable          |
 * | IMP-8   | LCD snprintf                      | line1[20], sizeof (format bug fixed in v9.1)   |
 * | IMP-9   | dspTask motion block              | Unconditional cooldown decrement; rising-edge  |
 * |         |                                   | reset                                          |
 * | IMP-10  | detectSpectral()                  | Adaptive bin count vs DFT resolution           |
 * | IMP-11  | dspTask non-zero path             | pqiHeart/Breath 0.90x decay per frame          |
 * | IMP-12  | Motion rising-edge block          | rejectionCount=0 on motion onset               |
 * | IMP-13  | loop() phase call                 | total_phase → _unused_phase                    |
 * | IMP-14  | MLX read block                    | 1 Hz polling gate                              |
 * | QA-1    | resetVitals()                     | Reset dspTask + inter-slot static variables    |
 * | QA-2    | goertzel()                        | fmaxf(0) floor prevents NaN from sqrtf         |
 * +---------+---------------------------------+------------------------------------------------+
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

#include "Seeed_Arduino_mmWave.h"
#include <Adafruit_MLX90614.h>
#include <BH1750.h>               // BH1750-1: ambient light; tie ADDR high → 0x5C
#include <Adafruit_NeoPixel.h>    // WS2-1: onboard RGB LED
#include <esp_task_wdt.h>         // PROD-1: explicit task watchdog
#include <Preferences.h>          // PROD-2: NVS state persistence

#ifdef ESP32
  #include <HardwareSerial.h>
  HardwareSerial mmWaveSerial(0);
#else
  #define mmWaveSerial Serial1
#endif

#ifndef PI
  #define PI 3.14159265358979323846f
#endif

// FIX-7: guard macro — prevents NaN from entering IIR/Kalman chains permanently
#define SAFE_FLOAT(x) (isfinite(x) ? (x) : 0.0f)

// FIX-17: human skin emissivity for MLX90614 software correction
// Melexis datasheet: default ε=1.0 (perfect blackbody); human skin ε≈0.95–0.98
// (Togawa, 1989; Steketee, 1973).  0.98 is conservative for exposed forehead/wrist.
const float MLX_EMISSIVITY = 0.98f;

SEEED_MR60BHA2    mmWave;
hd44780_I2Cexp    lcd;
Adafruit_MLX90614 mlx;
BH1750            lightMeter;              // BH1-1: ADDR pin tied HIGH → I2C 0x5C
Adafruit_NeoPixel pixel(1, D1, NEO_GRB + NEO_KHZ800); // HW-1: D1 = external WS2812
Preferences       prefs;                  // PROD-2: NVS key-value storage

bool lcdConnected  = false;
bool mlxReady      = false;
bool bh1750Ready   = false;               // BH1-1
uint8_t bh1750Addr = 0x5C;               // HW-2: records address that succeeded
float luxLevel     = 1000.0f;             // BH1-1: default to daytime until first read

// FIX-6: I²C bus recovery per I²C specification §3.1.16 "bus clear procedure".
// If an I²C slave (MLX90614 is the usual offender) holds SDA low after a
// power glitch or interrupted transaction, Wire.end()/begin() alone cannot
// release it because the peripheral driver only tri-states its own pins.
// Nine SCL clock pulses force the slave to complete its current byte and
// release SDA.  Called at startup and on MLX error recovery.
void i2cRecover() {
  Wire.end();
  pinMode(SCL, OUTPUT);
  pinMode(SDA, INPUT_PULLUP);
  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL, HIGH); delayMicroseconds(5);
    digitalWrite(SCL, LOW);  delayMicroseconds(5);
  }
  digitalWrite(SCL, HIGH);
  delayMicroseconds(5);
  Wire.begin();
  Wire.setTimeOut(500);
}

#define BUF_SIZE 256

const float HR_MIN = 40.0f;
const float HR_MAX = 180.0f;
const float RR_MIN = 8.0f;
const float RR_MAX = 35.0f;

const float HR_CONF_THRESHOLD      = 1.05f;
const float HR_CONF_THRESHOLD_FAST = 1.25f;
const float RR_CONF_THRESHOLD      = 1.05f;

const float PQI_LOCK_THRESHOLD = 0.35f;
const float HR_BAND_HALF       = 25.0f;

const float MIN_GAIN      = 0.1f;
const float MAX_GAIN      = 10.0f;
const float TARGET_ENERGY = 0.02f;

const int CALIBRATION_FRAMES  = 100;
const unsigned long CALIB_TIMEOUT_MS = 30000;  // FIX-10: 30 s fallback if phases never arrive
const int CONFIRM_VOTES        = 5;
const int ABSENT_VOTES         = 5;
const int MOTION_WARMUP_FRAMES = 10;

const int WIN_FAST = 64;
const int WIN_FULL = 160;

const float RECAL_RATIO_HI    = 4.0f;
const float RECAL_RATIO_LO    = 0.25f;
const int   RECAL_HOLD_FRAMES = 50;

const unsigned long DISPLAY_INTERVAL   = 600;
const unsigned long MLX_RETRY_INTERVAL = 3000;

float kfHR_x = 70.0f, kfHR_P = 1.0f;
const float kfHR_Q = 0.1f, kfHR_R = 4.0f;

float kfRR_x = 15.0f, kfRR_P = 1.0f;
const float kfRR_Q = 0.05f, kfRR_R = 2.0f;

float kalmanHR(float z) {
  kfHR_P += kfHR_Q;
  float K  = kfHR_P / (kfHR_P + kfHR_R);
  kfHR_x  += K * (z - kfHR_x);
  kfHR_P  *= (1.0f - K);
  return kfHR_x;
}

float kalmanRR(float z) {
  kfRR_P += kfRR_Q;
  float K  = kfRR_P / (kfRR_P + kfRR_R);
  kfRR_x  += K * (z - kfRR_x);
  kfRR_P  *= (1.0f - K);
  return kfRR_x;
}

float bph_b0 =  0.3707f, bph_b1 = 0.0f, bph_b2 = -0.3707f;
float bph_a1 = -0.7414f, bph_a2 = 0.2586f;
float bph_x1 = 0, bph_x2 = 0, bph_y1 = 0, bph_y2 = 0;

float bpHeartProcess(float x) {
  float y = bph_b0*x + bph_b1*bph_x1 + bph_b2*bph_x2
              - bph_a1*bph_y1 - bph_a2*bph_y2;
  bph_x2 = bph_x1; bph_x1 = x;
  bph_y2 = bph_y1; bph_y1 = y;
  return y;
}

float bpb_b0 =  0.1163f, bpb_b1 = 0.0f, bpb_b2 = -0.1163f;
float bpb_a1 = -1.6583f, bpb_a2 = 0.7674f;
float bpb_x1 = 0, bpb_x2 = 0, bpb_y1 = 0, bpb_y2 = 0;

float bpBreathProcess(float x) {
  float y = bpb_b0*x + bpb_b1*bpb_x1 + bpb_b2*bpb_x2
              - bpb_a1*bpb_y1 - bpb_a2*bpb_y2;
  bpb_x2 = bpb_x1; bpb_x1 = x;
  bpb_y2 = bpb_y1; bpb_y1 = y;
  return y;
}

void bpResetHeart()  { bph_x1=0; bph_x2=0; bph_y1=0; bph_y2=0; }
void bpResetBreath() { bpb_x1=0; bpb_x2=0; bpb_y1=0; bpb_y2=0; }

#define SG_WIN 5
float sgBufH[SG_WIN] = {0};
float sgBufB[SG_WIN] = {0};
int   sgIdx    = 0;
int   sgWarmup = 0;

float savitzkyGolay(float *buf) {
  int n = (sgIdx - 1 + SG_WIN) % SG_WIN;
  float v4 = buf[n];
  float v3 = buf[(n - 1 + SG_WIN) % SG_WIN];
  float v2 = buf[(n - 2 + SG_WIN) % SG_WIN];
  float v1 = buf[(n - 3 + SG_WIN) % SG_WIN];
  float v0 = buf[(n - 4 + SG_WIN) % SG_WIN];
  return (-3.0f*v0 + 12.0f*v1 + 17.0f*v2 + 12.0f*v3 - 3.0f*v4) / 35.0f;
}

float         heartBuf[BUF_SIZE];
float         breathBuf[BUF_SIZE];
unsigned long phaseTimes[BUF_SIZE];

static float linearHeart[BUF_SIZE];
static float linearBreath[BUF_SIZE];
static float linearFused[BUF_SIZE];

float fusedBuf[BUF_SIZE];
float pqiBreath = 0;

int bufIndex = 0;
int bufCount = 0;

float prevHeartPhase   = 0, prevBreathPhase = 0;
float heartOffset      = 0, breathOffset    = 0;
float clutterHeart     = 0, clutterBreath   = 0;
float prevHeartDiff    = 0, prevBreathDiff  = 0;
// IMP-7: prevBreathSample removed (dead variable)

float prevStableHeartPhase  = 0;
float prevStableBreathPhase = 0;

float stabilizePhase(float phase, float &prevStable) {
  float diff = phase - prevStable;
  if(fabsf(diff) > 0.5f) phase = prevStable + diff * 0.2f;
  prevStable = phase;
  return phase;
}

#define MED_WIN 5
float medBufH[MED_WIN] = {0};
float medBufB[MED_WIN] = {0};
int   medIdx = 0;

float medianOf5(float *a) {
  float s[MED_WIN];
  for(int i = 0; i < MED_WIN; i++) s[i] = a[i];
  for(int i = 1; i < MED_WIN; i++) {
    float key = s[i]; int j = i - 1;
    while(j >= 0 && s[j] > key) { s[j+1] = s[j]; j--; }
    s[j+1] = key;
  }
  return s[MED_WIN / 2];
}

float notchX1[3] = {0}, notchX2[3] = {0};
float notchY1[3] = {0}, notchY2[3] = {0};
float notchCosW[3] = {0};
const float notchR = 0.95f;
float lastNotchRR  = 0;
bool  notchEnabled = false;  // SS-1

void updateNotchCoeffs(float rrBPM, float fs) {
  float rrHz = rrBPM / 60.0f;
  for(int h = 0; h < 3; h++) {
    float w0     = 2.0f * PI * rrHz * (h + 2) / fs;
    notchCosW[h] = cosf(w0);
  }
}

float applyNotchChain(float x) {
  float v = x;
  for(int h = 0; h < 3; h++) {
    float cw = notchCosW[h];
    float r  = notchR;
    float y  = v
               - 2.0f*cw*notchX1[h]   + notchX2[h]
               + 2.0f*r*cw*notchY1[h] - r*r*notchY2[h];
    notchX2[h] = notchX1[h]; notchX1[h] = v;
    notchY2[h] = notchY1[h]; notchY1[h] = y;
    v = y;
  }
  return v;
}

#define NLMS_TAPS 8
float nlmsW[NLMS_TAPS] = {0};
float nlmsX[NLMS_TAPS] = {0};
const float NLMS_MU  = 0.03f;  // retained for const-name compatibility; see IMP-6 adaptive mu below
const float NLMS_EPS = 1e-4f;

const float NLMS_MU_MAX   = 0.10f;   // IMP-6
const float NLMS_MU_MIN   = 0.01f;   // IMP-6
const float NLMS_MU_DECAY = 0.995f;  // IMP-6
static float nlmsMuCurrent = NLMS_MU_MAX;  // IMP-6

float nlmsUpdate(float ref, float primary, bool freezeWeights) {
  for(int i = NLMS_TAPS-1; i > 0; i--) nlmsX[i] = nlmsX[i-1];
  nlmsX[0] = ref;
  float yhat = 0;
  for(int i = 0; i < NLMS_TAPS; i++) yhat += nlmsW[i] * nlmsX[i];
  float e = primary - yhat;
  if(!freezeWeights) {
    float power = NLMS_EPS;
    for(int i = 0; i < NLMS_TAPS; i++) power += nlmsX[i] * nlmsX[i];
    float mu_n = nlmsMuCurrent / power;  // IMP-6: adaptive mu
    for(int i = 0; i < NLMS_TAPS; i++) nlmsW[i] += mu_n * e * nlmsX[i];
    nlmsMuCurrent = fmaxf(NLMS_MU_MIN, nlmsMuCurrent * NLMS_MU_DECAY);  // IMP-6
  }
  return e;
}

float motionLP     = 0;
int   motionWarmup = 0;

bool motionDetected(float dh, float db) {
  float energy = dh*dh + db*db;
  motionLP = 0.95f * motionLP + 0.05f * energy;
  if(motionWarmup < MOTION_WARMUP_FRAMES) { motionWarmup++; return false; }
  return (energy > max(motionLP * 5.0f, 1e-6f));
}

float computePQI(float *buf, int n) {
  float energy = 0;
  for(int i = 0; i < n; i++) energy += buf[i] * buf[i];
  if(energy <= 1e-6f) return 0;
  float best = 0;
  for(int lag = 2; lag < n/2; lag++) {
    float ac = 0;
    for(int i = 0; i < n - lag; i++) ac += buf[i] * buf[i + lag];
    if(ac > best) best = ac;
  }
  return constrain(best / energy, 0.0f, 1.0f);
}

float lut_sin(float ph);
float lut_cos(float ph);

// IMP-2: Goertzel algorithm replaces brute-force DFT
float goertzel(float *buf, int n, float freq, float fs) {
  float coeff = 2.0f * cosf(2.0f * PI * freq / fs);
  float s0 = 0, s1 = 0, s2 = 0;
  for(int k = 0; k < n; k++) {
    s0 = buf[k] + coeff * s1 - s2;
    s2 = s1; s1 = s0;
  }
  return fmaxf(0.0f, s1*s1 + s2*s2 - coeff*s1*s2);  // IMP-2 + QA-2: floor to 0 prevents NaN from sqrtf
}

void respirationLockedClean(float *buf, int n, float fs, float rrBPM) {
  if(rrBPM < RR_MIN || n < 32) return;
  float fR = rrBPM / 60.0f;
  for(int h = 2; h <= 4; h++) {
    float freq = fR * h;
    if(freq >= fs * 0.5f) break;
    float phStep = 2.0f * PI * freq / fs;
    // IMP-2: Goertzel for analysis
    float power = goertzel(buf, n, freq, fs);
    float amp = 2.0f * sqrtf(power) / n;
    // Need phase: compute via single-pass DFT for phi only
    float re = 0, im = 0;
    float ph = 0;
    for(int k = 0; k < n; k++) {
      re +=  buf[k] * lut_cos(ph);
      im -= buf[k] * lut_sin(ph);
      ph = fmodf(ph + phStep, 2.0f * PI);  // IMP-4: prevent float precision loss
    }
    float phi = atan2f(-im, re);
    // IMP-4: fmodf phase wrapping in synthesis loop
    ph = 0;
    for(int k = 0; k < n; k++) {
      buf[k] -= amp * lut_cos(ph + phi);
      ph = fmodf(ph + phStep, 2.0f * PI);  // IMP-4: prevent float precision loss
    }
  }
}

float radarGain      = 1.0f;
float baselineEnergy = 0;
bool  calibrationDone  = false;
int   calibrationCount = 0;
unsigned long calibStartMs = 0;  // FIX-10: millis() when calibration began

float rollingEnergy = 0;
int   recalFrames   = 0;

float smoothHR   = 0;
float smoothRR   = 15.0f;
float smoothTemp = NAN;

int   hrState      = 0;
float hrConfidence = 0;
float pqiHeart     = 0;

float lastStableHR   = 0;
int   rejectionCount = 0;

static int   dspTask      = 0;
static bool  lastAutoValid = false;
static float lastHrAuto    = 0;
static float lastConfHR    = 0;

static float lastRRDSP  = 0;   // IMP-3
static float lastConfRR  = 0;  // IMP-3
static bool  lastRRValid = false;  // IMP-3

static bool wasMotion      = false;
static int  motionCooldown = 0;

int  presentVotes  = 0;
int  absentVotes   = 0;
bool humanDetected = false;

float rawHR = 0;
float rawRR = 0;

#define RAW_STAB_WIN 12
float rawHRWin[RAW_STAB_WIN] = {0};
int   rawHRWinIdx   = 0;
int   rawHRWinCount = 0;

static bool ghostSuspect = false;  // GHO-1: mechanically-flat rawHR → suspect vibration ghost
static bool rhcSuspect   = false;  // RHC-1: hrAuto ≈ N×smoothRR → suspect resp harmonic lock

float rawHRStability() {
  if(rawHRWinCount < 3) return 0;
  float mean = 0;
  for(int i = 0; i < rawHRWinCount; i++) mean += rawHRWin[i];
  mean /= rawHRWinCount;
  float var = 0;
  for(int i = 0; i < rawHRWinCount; i++) {
    float d = rawHRWin[i] - mean; var += d*d;
  }
  var /= rawHRWinCount;
  return constrain(1.0f - var / 100.0f, 0.0f, 1.0f);
}

// GHO-1: returns true when rawHR has been flat for ≥8 samples (stddev < 1.0 BPM).
// A mechanical oscillator (fan, HVAC motor, vibrating surface) produces a perfectly
// stable frequency → stddev ≈ 0. Real cardiac output produces ≥1 BPM beat-to-beat
// variation even in the radar module's own rate estimate.
// NOTE: rawHRStability() gives this source MAXIMUM fusion weight (stability = 1.0),
// so without this check a ghost actively out-competes a valid DSP reading.
bool rawHRGhostSuspect() {
  if(rawHRWinCount < 8) return false;
  float mean = 0;
  for(int i = 0; i < rawHRWinCount; i++) mean += rawHRWin[i];
  mean /= rawHRWinCount;
  float var = 0;
  for(int i = 0; i < rawHRWinCount; i++) {
    float d = rawHRWin[i] - mean; var += d * d;
  }
  var /= rawHRWinCount;
  return (sqrtf(var) < 1.0f);  // stddev < 1 BPM
}

// ── TEL-1/2/3: Advanced telemetry globals ────────────────────────────────
// Polled once per newData frame; all three are O(1) cached reads from
// the frame parser — no additional UART work beyond mmWave.update(80).
bool  radarIsPresent = false; // TEL-1: mmWave.isHumanDetected()
float radarDistance  = 0;     // TEL-2: mmWave.getDistance(); 0 = no target tracked
float prevTotalPhase = 0;     // TEL-3: previous totalPhase for phaseDelta
float phaseDelta     = 0;     // TEL-3: |totalPhase - prevTotalPhase| rad/frame

// FIX-5: stale-data timeout for presence keepalive.
// If no valid HR or RR is received for this many ms, suppress radarPresenceKeepAlive
// even if the sensor's own isHumanDetected() flag is still high.  The MR60BHA2's
// built-in holdoff can keep that flag active 10–60 s after the patient leaves.
const unsigned long PRESENCE_KEEPALIVE_TIMEOUT = 15000;
unsigned long       lastValidRateMs            = 0;

// FIX-8: consecutive bad-frame counter for UART resynchronisation.
int       consecutiveBadRadar = 0;
const int BAD_RADAR_LIMIT     = 20;

// PROD-3: rolling HR variance for LCD display (HR 72±3)
#define HR_VAR_WIN 10
static float hrVarBuf[HR_VAR_WIN];
static int   hrVarIdx   = 0;
static int   hrVarCount = 0;

// TEL-3: gross body motion produces > ~0.50 rad/frame change in total phase.
// Normal breathing at RR=15 produces ~0.15 rad/frame at 10 Hz.
// Threshold is placed above breathing to avoid false motion gates.
const float MOTION_PHASE_LIMIT = 0.50f;

// Uncomment to enable 1 Hz Serial telemetry debug output:
// #define DEBUG_TELEMETRY

float         prevAmbient  = 0;
unsigned long lastMlxRetry = 0;
unsigned long lastDisplay  = 0;

byte barChars[5][8] = {
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
  {0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10},
  {0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18},
  {0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C},
  {0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F}
};

// PROD-3: ± glyph for LCD custom char slot 6
byte pmChar[8] = {0x04,0x04,0x1F,0x04,0x04,0x00,0x1F,0x00};

void drawBar(int level) {
  for(int i = 0; i < 5; i++) {
    int v = level - (i * 4);
    if     (v <= 0) lcd.write(byte(1));
    else if(v == 1) lcd.write(byte(2));
    else if(v == 2) lcd.write(byte(3));
    else if(v == 3) lcd.write(byte(4));
    else            lcd.write(byte(5));
  }
}

void resetVitals() {
  smoothHR   = 0;
  smoothRR   = 15.0f;
  smoothTemp = NAN;

  hrState        = 0;
  hrConfidence   = 0;
  pqiHeart       = 0;
  lastStableHR   = 0;
  rejectionCount = 0;

  kfHR_x = 70.0f; kfHR_P = 1.0f;
  kfRR_x = 15.0f; kfRR_P = 1.0f;

  bpResetHeart();
  bpResetBreath();

  for(int i = 0; i < SG_WIN;  i++) { sgBufH[i] = 0; sgBufB[i] = 0; }
  sgIdx    = 0;
  sgWarmup = 0;

  for(int i = 0; i < MED_WIN; i++) { medBufH[i] = 0; medBufB[i] = 0; }
  medIdx = 0;

  for(int h = 0; h < 3; h++) {
    notchX1[h]=0; notchX2[h]=0;
    notchY1[h]=0; notchY2[h]=0;
  }
  lastNotchRR = 0;
  notchEnabled = false;  // SS-1

  for(int i = 0; i < NLMS_TAPS; i++) { nlmsW[i]=0; nlmsX[i]=0; }
  nlmsMuCurrent = NLMS_MU_MAX;  // IMP-6

  rawHRWinIdx   = 0;
  rawHRWinCount = 0;
  memset(rawHRWin, 0, sizeof(rawHRWin));   // FIX-14: clear stale rawHR window data
  ghostSuspect  = false;  // GHO-1
  rhcSuspect    = false;  // RHC-1

  // IMP-7: prevBreathSample removed
  pqiBreath        = 0;

  motionLP       = TARGET_ENERGY;
  motionWarmup   = 0;
  wasMotion      = false;
  motionCooldown = 0;

  rollingEnergy = TARGET_ENERGY;
  recalFrames   = 0;

  prevHeartPhase  = 0;
  prevBreathPhase = 0;
  heartOffset     = 0;
  breathOffset    = 0;
  clutterHeart    = 0;
  clutterBreath   = 0;
  prevHeartDiff   = 0;
  prevBreathDiff  = 0;

  prevStableHeartPhase  = 0;
  prevStableBreathPhase = 0;

  // QA-1: reset DSP task slot and inter-slot state to prevent stale patient data
  dspTask       = 0;
  lastAutoValid = false;
  lastHrAuto    = 0;
  lastConfHR    = 0;
  lastRRDSP     = 0;
  lastConfRR    = 0;
  lastRRValid   = false;

  // TEL-1/2/3: reset telemetry state with patient data
  radarIsPresent = false;
  radarDistance  = 0;
  prevTotalPhase = 0;
  phaseDelta     = 0;

  // FIX-5/8/14: reset keepalive timer, bad-frame counter, raw rates
  lastValidRateMs    = millis();
  consecutiveBadRadar = 0;
  rawHR = 0;   // FIX-14: prevent stale carry-over to next patient
  rawRR = 0;   // FIX-14

  // PROD-3: clear HR variance window
  hrVarIdx   = 0;
  hrVarCount = 0;
  memset(hrVarBuf, 0, sizeof(hrVarBuf));

  // SS-2: clear circular buffers to prevent stale patient data
  bufIndex = 0; bufCount = 0;
  memset(heartBuf,  0, sizeof(heartBuf));
  memset(breathBuf, 0, sizeof(breathBuf));
  memset(fusedBuf,  0, sizeof(fusedBuf));
  memset(phaseTimes, 0, sizeof(phaseTimes));
}

float unwrap(float phase, float &prev, float &offset) {
  float diff = phase - prev;
  if(diff >  PI) offset -= 2.0f * PI;
  if(diff < -PI) offset += 2.0f * PI;
  prev = phase;
  return phase + offset;
}

float removeClutter(float s, float &clutter) {
  clutter = 0.995f * clutter + 0.005f * s;
  return s - clutter;
}

float phaseDiff(float phase, float &prev) {
  float diff = phase - prev;
  prev = phase;
  return diff;
}

float autocorr(float *buf, int n, int lag) {
  int cnt = n - lag;              // IMP-1
  if(cnt <= 0) return 0.0f;      // IMP-1
  float sum = 0;
  for(int i = 0; i < cnt; i++) sum += buf[i] * buf[i + lag];
  return sum / (float)cnt;        // IMP-1: unbiased estimator
}

float parabolicPeak(float v1, float v2, float v3) {
  float denom = v1 - 2.0f*v2 + v3;
  if(fabsf(denom) < 1e-10f) return 0;
  return 0.5f * (v1 - v3) / denom;
}

#define LUT_SIZE 512
static float sinLUT[LUT_SIZE];

void buildSinLUT() {
  for(int i = 0; i < LUT_SIZE; i++)
    sinLUT[i] = sinf(2.0f * PI * i / LUT_SIZE);
}

float lut_sin(float ph) {
  int idx = (int)(ph * (LUT_SIZE / (2.0f * PI)));
  idx = idx % LUT_SIZE;
  if(idx < 0) idx += LUT_SIZE;
  return sinLUT[idx];
}

float lut_cos(float ph) {
  int idx = (int)(ph * (LUT_SIZE / (2.0f * PI))) + LUT_SIZE / 4;
  idx = idx % LUT_SIZE;
  if(idx < 0) idx += LUT_SIZE;
  return sinLUT[idx];
}

float harmonicSpectrumScore(float *buf, int n, float fs, float bpm) {
  float f     = bpm / 60.0f;
  float score = 0;
  for(int h = 1; h <= 3; h++) {
    float freq   = f * h;
    float mag = sqrtf(goertzel(buf, n, freq, fs));  // IMP-2: Goertzel replaces DFT
    if(h == 1) score += mag;
    if(h == 2) score += 0.5f  * mag;
    if(h == 3) score += 0.25f * mag;
  }
  return score;
}

bool detectRate(float *buf, int n, float fs,
                float minRate, float maxRate,
                float &rateOut, float &confOut,
                float confThreshold) {

  int minLag = (int)(fs * 60.0f / maxRate);
  int maxLag = (int)(fs * 60.0f / minRate);
  minLag = max(minLag, 1);
  maxLag = min(maxLag, n - 1);
  if(minLag >= maxLag) return false;

  float best    = 0;
  float avg     = 0;
  int   bestLag = minLag;
  int   count   = 0;

  for(int lag = minLag; lag <= maxLag; lag++) {
    float v = autocorr(buf, n, lag);
    avg += v;
    count++;
    if(v > best) { best = v; bestLag = lag; }
  }

  if(count <= 1) return false;
  avg = (avg - best) / (count - 1);
  if(avg <= 1e-6f) return false;

  confOut = constrain(best / avg, 0.0f, 10.0f);
  if(confOut < confThreshold) return false;

  float period;
  if(bestLag > minLag && bestLag < maxLag) {
    float v1    = autocorr(buf, n, bestLag - 1);
    float v3    = autocorr(buf, n, bestLag + 1);
    float delta = constrain(parabolicPeak(v1, best, v3), -0.5f, 0.5f);
    period = (bestLag + delta) / fs;
  } else {
    period = bestLag / fs;
  }

  if(period <= 0) return false;

  rateOut = constrain(60.0f / period, minRate, maxRate);
  return true;
}

float detectSpectral(float *buf, int n, float fs,
                     float fmin, float fmax) {
  // IMP-10: adaptive bin count to prevent scalloping
  int bins = max(20, min(120, (int)((fmax - fmin) * (float)n / fs) + 2));
  float bestMag  = 0;
  float bestFreq = fmin;
  for(int i = 0; i < bins; i++) {
    float freq   = fmin + i * (fmax - fmin) / bins;
    float mag = sqrtf(goertzel(buf, n, freq, fs));  // IMP-2: Goertzel replaces DFT
    if(mag > bestMag) { bestMag = mag; bestFreq = freq; }
  }
  return bestFreq * 60.0f;
}

float coherenceFilter(float newHR) {
  if(lastStableHR == 0) { lastStableHR = newHR; return newHR; }
  float diff = fabsf(newHR - lastStableHR);
  if(diff > 25.0f) {
    rejectionCount++;
    if(rejectionCount > 8) {
      lastStableHR = newHR; rejectionCount = 0; return newHR;
    }
    return lastStableHR;
  }
  rejectionCount = 0;
  if(diff > 10.0f) newHR = (lastStableHR + newHR) * 0.5f;
  lastStableHR = newHR;
  return newHR;
}

// ── WS2-1: LED state machine ──────────────────────────────────────────────
//
// Priority (highest → lowest):
//   1. MOTION      → red flash (patient moved, reading invalid)
//   2. GHOST/RHC   → orange steady (suspect signal, not trusted)
//   3. LCK         → green pulsing at detected HR (confirmed lock)
//   4. SEN         → yellow slow breathe (searching/partial)
//   5. NO HUMAN    → dim blue (standby)
//
// Night mode (lux < 10): brightness scaled to 10% to avoid disturbing sleep.
// All timing uses millis() — no delay(), no blocking.
//
void updateLED(bool inMotion) {
  static unsigned long ledMillis  = 0;
  static bool          flashState = false;

  // Night-mode brightness gate: 10% in dark room, 100% otherwise
  uint8_t brite = (luxLevel < 10.0f) ? 4 : 40;
  pixel.setBrightness(brite);

  uint32_t colour = pixel.Color(0, 0, 0);
  unsigned long now = millis();

  if(!humanDetected) {
    // Dim blue standby — pulses gently every 2 s to show system is alive
    float t   = (now % 2000) / 2000.0f;
    uint8_t b = (uint8_t)(20.0f + 20.0f * sinf(t * 2.0f * PI));
    colour = pixel.Color(0, 0, b);

  } else if(inMotion) {
    // Red flash at 4 Hz — patient is moving
    if(now - ledMillis >= 125) { ledMillis = now; flashState = !flashState; }
    colour = flashState ? pixel.Color(80, 0, 0) : pixel.Color(0, 0, 0);

  } else if(ghostSuspect || rhcSuspect) {
    // Orange steady — signal suspect (mechanical ghost or resp harmonic)
    colour = pixel.Color(80, 30, 0);

  } else if(hrState == 2) {
    // Green pulse at detected HR — visual heartbeat confirmation
    // smoothHR in BPM → period in ms; clamp to physiological range to avoid
    // divide-by-zero or absurdly fast/slow flicker
    float hr     = constrain(smoothHR, HR_MIN, HR_MAX);
    if(!isfinite(hr)) hr = 72.0f;  // FIX-7: prevent UB from NaN→unsigned cast
    unsigned long period = (unsigned long)(60000.0f / hr);  // ms per beat
    float phase  = (float)(now % period) / (float)period;
    // Narrow bright pulse: rises quickly, holds briefly, falls quickly
    float pulse  = powf(sinf(phase * PI), 2.0f);
    uint8_t g    = (uint8_t)(20.0f + 60.0f * pulse);
    uint8_t r    = (uint8_t)(5.0f  + 10.0f * pulse);  // slight warm tint
    colour = pixel.Color(r, g, 0);

  } else if(hrState == 1) {
    // Yellow slow breathe — SEN state, still acquiring
    float t   = (now % 3000) / 3000.0f;
    uint8_t v = (uint8_t)(15.0f + 30.0f * sinf(t * 2.0f * PI));
    colour = pixel.Color(v, v, 0);

  } else {
    // Yellow-dim — human detected but no HR state yet (--- on LCD)
    colour = pixel.Color(15, 15, 0);
  }

  pixel.setPixelColor(0, colour);
  pixel.show();
}

void setup() {
  Serial.begin(115200);
  buildSinLUT();

  // FIX-6: attempt bus recovery before Wire.begin in case SDA stuck from prior crash
  i2cRecover();

  // WS2-1: NeoPixel boot — show blue while hardware initialises
  pixel.begin();
  pixel.setBrightness(40);
  pixel.setPixelColor(0, pixel.Color(0, 0, 80));
  pixel.show();

  mmWaveSerial.begin(115200);
  mmWave.begin(&mmWaveSerial);

  mlxReady = mlx.begin();

  // HW-2: BH1750 address fallback.
  // ADDR=HIGH → 0x5C.  ADDR=LOW or floating → 0x23.
  // Try 0x5C first; if begin() returns false (device not ACKing), retry at 0x23.
  // Without this, a floating ADDR pin produces "[BH1750] ERROR: other error" to Serial
  // and bh1750Ready stays false even though the sensor is present.
  bh1750Addr  = 0x5C;
  bh1750Ready = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x5C, &Wire);
  if(!bh1750Ready) {
    bh1750Addr  = 0x23;
    bh1750Ready = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
  }
  Serial.print("[BH1750] ");
  if(bh1750Ready) { Serial.print("OK at 0x"); Serial.println(bh1750Addr, HEX); }
  else            { Serial.println("not found at 0x5C or 0x23 — light sensor disabled"); }

  // WS2-1: cyan = radar + MLX initialised
  pixel.setPixelColor(0, pixel.Color(0, 60, 60));
  pixel.show();

  // FIX-11: skip any address the BH1750 already claimed to prevent collision
  int lcdCandidates[] = {0x23, 0x27};
  for(int i = 0; i < 2; i++) {
    if(bh1750Ready && lcdCandidates[i] == bh1750Addr) continue;  // FIX-11
    lcd = hd44780_I2Cexp(lcdCandidates[i]);
    if(lcd.begin(16, 2) == 0) {
      lcdConnected = true;
      lcd.backlight();
      for(int c = 0; c < 5; c++) lcd.createChar(c + 1, barChars[c]);
      lcd.createChar(6, pmChar);  // PROD-3: ± glyph
      break;
    }
  }

  // WS2-1: yellow = calibrating about to begin
  pixel.setPixelColor(0, pixel.Color(60, 60, 0));
  pixel.show();

  if(lcdConnected) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Radar Vital v9.5");
    delay(1500);
    lcd.clear();
  }

  // FIX-10: record calibration start time
  calibStartMs = millis();

  // FIX-5: initialise keepalive timer
  lastValidRateMs = millis();

  // PROD-1: explicit task watchdog — 8 s timeout, panic on expiry
  esp_task_wdt_init(8, true);
  esp_task_wdt_add(NULL);   // subscribe loopTask

  // PROD-2: restore persisted state from NVS (if previous session wrote it)
  prefs.begin("rvital", true);  // read-only
  float savedGain = prefs.getFloat("gain", 0);
  if(savedGain >= MIN_GAIN && savedGain <= MAX_GAIN) {
    radarGain = savedGain;
    Serial.printf("[NVS] restored radarGain=%.3f\n", radarGain);
  }
  float savedKfHR = prefs.getFloat("kfHR", 0);
  if(savedKfHR >= HR_MIN && savedKfHR <= HR_MAX) {
    kfHR_x = savedKfHR;
    Serial.printf("[NVS] restored kfHR_x=%.1f\n", kfHR_x);
  }
  float savedKfRR = prefs.getFloat("kfRR", 0);
  if(savedKfRR >= RR_MIN && savedKfRR <= RR_MAX) {
    kfRR_x = savedKfRR;
    Serial.printf("[NVS] restored kfRR_x=%.1f\n", kfRR_x);
  }
  prefs.end();
}

void loop() {

  esp_task_wdt_reset();  // PROD-1: feed watchdog at top of every loop pass

  bool inMotion       = false;
  bool ambientEvidence = false;  // FIX-2: hoisted; must be visible to presence SM below

  if(!mlxReady && millis() - lastMlxRetry > MLX_RETRY_INTERVAL) {
    mlxReady     = mlx.begin();
    lastMlxRetry = millis();
  }

  bool newData = mmWave.update(80);
  // FIX-2: no early return here — presence SM and LCD must always execute.
  // Radar regularly returns false during large motion or out-of-range (> ~1.5 m).
  // Old if(!newData) return caused humanDetected to freeze permanently.

  // FIX-12: MLX + ambientEvidence polled OUTSIDE if(newData) so temperature and
  // ambient drift continue to be sampled during radar dropouts (motion, range edge).
  // IMP-14: MLX90614 polling downsampled to 1 Hz
  static unsigned long lastTempRead = 0;
  // ambientEvidence declared at top of loop() — FIX-2

  if(mlxReady && millis() - lastTempRead >= 1000) {  // IMP-14: 1 Hz
    lastTempRead = millis();

    float amb = mlx.readAmbientTempC();  // FIX-17: read ambient first for emissivity correction
    float obj = mlx.readObjectTempC();

    static int mlxErrors = 0;
    if(!isnan(obj) && !isnan(amb)) {
      mlxErrors = 0;
      // FIX-17: software emissivity correction for human skin (ε=0.98).
      // Linearised Stefan-Boltzmann: Tcorr ≈ Traw + (1−ε)/ε × (Traw−Tamb).
      // At 12 °C differential: correction ≈ +0.24 °C.  Avoids EEPROM wear from
      // writeEmissivity() (100 k cycle limit per Melexis datasheet §8.4.5).
      obj = obj + (1.0f - MLX_EMISSIVITY) / MLX_EMISSIVITY * (obj - amb);

      if(isnan(smoothTemp)) smoothTemp = obj;
      else                  smoothTemp = SAFE_FLOAT(0.9f * smoothTemp + 0.1f * obj);  // FIX-7
    } else {
      if(++mlxErrors > 3) {
        mlxReady  = false;
        mlxErrors = 0;
        // FIX-6: proper bus recovery + peripheral re-init (replaces bare Wire.end/begin)
        i2cRecover();
        if(bh1750Ready)
          bh1750Ready = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, bh1750Addr, &Wire);
        if(lcdConnected) {
          lcd.begin(16, 2);
          lcd.backlight();
        }
      }
    }

    float drift     = fabsf(amb - prevAmbient);
    prevAmbient     = amb;
    ambientEvidence = (!isnan(drift) && drift > 0.2f);  // FIX-7: guard NaN from bad amb read
  }

  if(newData) {  // FIX-2: sensor reads + DSP gated on fresh radar frame

  // FIX-9: check return values from mmWave getters.  The library returns false
  // if the last parsed frame did not contain the requested data type (partial frame,
  // UART noise, firmware mismatch).  Ignoring the return leaves rawHR/rawRR/distance
  // at their previous (stale) values, silently poisoning fusion and presence logic.
  bool hrOk   = mmWave.getHeartRate(rawHR);
  bool rrOk   = mmWave.getBreathRate(rawRR);
  radarIsPresent = mmWave.isHumanDetected();  // TEL-1 (bool return = presence itself)
  bool distOk = mmWave.getDistance(radarDistance);          // TEL-2

  // FIX-8: consecutive bad-frame counter.  If the UART desyncs (partial frame offset),
  // the parser returns garbage indefinitely.  After BAD_RADAR_LIMIT failures, flush
  // the serial buffer to force the parser to resynchronise on the next frame header.
  if(!hrOk && !rrOk && !distOk) {
    consecutiveBadRadar++;
    if(consecutiveBadRadar >= BAD_RADAR_LIMIT) {
      while(mmWaveSerial.available()) mmWaveSerial.read();  // flush RX buffer
      consecutiveBadRadar = 0;
    }
  } else {
    consecutiveBadRadar = 0;
  }

  // FIX-5: update keepalive timer whenever a physiologically valid rate arrives
  if((hrOk && rawHR >= HR_MIN && rawHR <= HR_MAX) ||
     (rrOk && rawRR >= RR_MIN && rawRR <= RR_MAX)) {
    lastValidRateMs = millis();
  }

  // FIX-9: only feed rawHR window with actually-fresh, valid data
  if(hrOk && rawHR >= HR_MIN && rawHR <= HR_MAX && !wasMotion) {
    rawHRWin[rawHRWinIdx] = rawHR;
    rawHRWinIdx = (rawHRWinIdx + 1) % RAW_STAB_WIN;
    if(rawHRWinCount < RAW_STAB_WIN) rawHRWinCount++;
  }

  // SS-3: phaseDataValid flag replaces hard return
  // TEL-3: totalPhase now saved (IMP-13 had discarded it as unusedPhase).
  // Used only for phaseDelta — raw gross-motion indicator before NLMS/EMA settle.
  float totalPhase, breath_phase, heart_phase;
  bool phaseDataValid = mmWave.getHeartBreathPhases(totalPhase, breath_phase, heart_phase);  // SS-3

  // FIX-10: if calibration hasn't completed and phases aren't arriving (phaseDataValid
  // is persistently false), the timeout must still be checked here — the in-block
  // timeout only fires when phaseDataValid is true and energy can be accumulated.
  if(!calibrationDone && !phaseDataValid && millis() - calibStartMs >= CALIB_TIMEOUT_MS) {
    radarGain       = 1.0f;
    rollingEnergy   = TARGET_ENERGY;
    motionLP        = TARGET_ENERGY;
    calibrationDone = true;
    presentVotes    = 0;
    absentVotes     = 0;
    Serial.println("[CALIB] timeout (no phase data) — using default gain=1.0");
    if(lcdConnected) {
      lcd.setCursor(0, 0);
      lcd.print("Calib timeout!  ");
      delay(800);
      lcd.clear();
    }
  }

  if(phaseDataValid) {  // SS-3: phase DSP only when valid

    // TEL-3 + FIX-18: compute phase delta for early gross-motion gate.
    // prevTotalPhase initialises to 0; the first frame produces a large spike —
    // harmless because motionWarmup gate already suppresses motion for 10 frames.
    // FIX-18: wrap delta to [-π,π] before fabsf. Without this, a natural 2π→0
    // totalPhase wrap produces phaseDelta ≈ 6.28 rad → false roughMotion.
    float rawPD    = totalPhase - prevTotalPhase;
    if(rawPD >  PI) rawPD -= 2.0f * PI;
    if(rawPD < -PI) rawPD += 2.0f * PI;
    phaseDelta     = fabsf(rawPD);
    prevTotalPhase = totalPhase;

    float uh = unwrap(heart_phase,  prevHeartPhase,  heartOffset);
    float ub = unwrap(breath_phase, prevBreathPhase, breathOffset);

    uh = stabilizePhase(uh, prevStableHeartPhase);
    ub = stabilizePhase(ub, prevStableBreathPhase);

    float ch = removeClutter(uh, clutterHeart);
    float cb = removeClutter(ub, clutterBreath);

    sgBufH[sgIdx] = ch;
    sgBufB[sgIdx] = cb;
    sgIdx = (sgIdx + 1) % SG_WIN;

    float smoothedH, smoothedB;
    if(sgWarmup < SG_WIN) {
      sgWarmup++;
      smoothedH = ch;
      smoothedB = cb;
    } else {
      smoothedH = savitzkyGolay(sgBufH);
      smoothedB = savitzkyGolay(sgBufB);
    }

    float dh = phaseDiff(smoothedH, prevHeartDiff);
    float db = phaseDiff(smoothedB, prevBreathDiff);

    dh = bpHeartProcess(dh);
    db = bpBreathProcess(db);

    float energy = dh*dh + db*db;

    if(!calibrationDone) {
      baselineEnergy += energy;
      calibrationCount++;

      if(lcdConnected) {
        lcd.setCursor(0, 0);
        lcd.print("Calibrating...  ");
        lcd.setCursor(0, 1);
        int progress = map(calibrationCount, 0, CALIBRATION_FRAMES, 0, 20);
        drawBar(progress);
        lcd.print("           ");
      }

      // FIX-10: if sensor never delivers CALIBRATION_FRAMES valid phases within
      // 30 s (no person in FOV, radar FW init delay, UART sync issue), fall back
      // to unity gain + warning rather than hanging forever.
      bool calibComplete = (calibrationCount >= CALIBRATION_FRAMES);
      bool calibTimeout  = (millis() - calibStartMs >= CALIB_TIMEOUT_MS);

      if(calibComplete || calibTimeout) {
        if(calibComplete && calibrationCount > 0) {
          baselineEnergy /= calibrationCount;
          baselineEnergy  = max(baselineEnergy, 1e-6f);
          radarGain       = constrain(TARGET_ENERGY / baselineEnergy, MIN_GAIN, MAX_GAIN);
        } else {
          radarGain = 1.0f;  // FIX-10: safe default
          Serial.println("[CALIB] timeout — using default gain=1.0");
          if(lcdConnected) {
            lcd.setCursor(0, 0);
            lcd.print("Calib timeout!  ");
            delay(800);
          }
        }
        rollingEnergy   = TARGET_ENERGY;
        motionLP        = TARGET_ENERGY;
        calibrationDone = true;
        presentVotes    = 0;
        absentVotes     = 0;
        if(lcdConnected) lcd.clear();
      }

      return;
    }

    dh *= radarGain;
    db *= radarGain;

    float rawEnergy   = dh*dh + db*db;
    // TEL-3: include phaseDelta-based gross-motion detection.
    // Threshold 0.50 rad/frame is above normal breathing variation (~0.15 rad/frame
    // at 10 Hz for RR=15) but below large positional shifts (typically 2–10 rad).
    bool  roughMotion = (rawEnergy > max(motionLP * 5.0f, 1e-6f)) ||
                        (phaseDelta > MOTION_PHASE_LIMIT && motionWarmup >= MOTION_WARMUP_FRAMES);

    if(!roughMotion) {
      rollingEnergy = 0.99f * rollingEnergy + 0.01f * rawEnergy;
    }

    if(rollingEnergy > TARGET_ENERGY * RECAL_RATIO_HI ||
       rollingEnergy < TARGET_ENERGY * RECAL_RATIO_LO) {
      recalFrames++;
      if(recalFrames > RECAL_HOLD_FRAMES) {
        rollingEnergy = max(rollingEnergy, 1e-6f);
        radarGain     = constrain(TARGET_ENERGY / rollingEnergy, MIN_GAIN, MAX_GAIN);
        rollingEnergy = 0.5f * rollingEnergy + 0.5f * TARGET_ENERGY;
        recalFrames   = 0;
      }
    } else {
      recalFrames = 0;
    }

    medBufH[medIdx] = dh;
    medBufB[medIdx] = db;
    medIdx = (medIdx + 1) % MED_WIN;
    dh = medianOf5(medBufH);
    db = medianOf5(medBufB);

    // SS-1: notch only fires after first valid smoothRR
    if(notchEnabled && smoothRR >= RR_MIN) {  // SS-1
      if(fabsf(smoothRR - lastNotchRR) > 1.5f) {  // IMP-5: was 0.5f
        updateNotchCoeffs(smoothRR, 10.0f);
        lastNotchRR = smoothRR;
      }
      dh = applyNotchChain(dh);
    }

    bool nlmsFreeze = (fabsf(db) >= 0.5f);
    dh = nlmsUpdate(db, dh, nlmsFreeze);

    inMotion = motionDetected(dh, db) || roughMotion;

    float breathInstPhase = atan2f(smoothedB, db + 1e-6f);
    float breathGate      = fabsf(lut_sin(breathInstPhase));  // SS-4: sin=1 at breath peak (chest still=best HR window); cos was inverted
    float dhGated         = dh * (0.5f + 0.5f * breathGate);
    // IMP-7: prevBreathSample assignment removed

    float w1 = pqiHeart;
    float w2 = pqiBreath * 0.7f;
    float w3 = 0.3f;
    float weightSum = w1 + w2 + w3;
    float fusedSig;
    if(weightSum > 0.01f)
      fusedSig = (w1 * dhGated + w2 * db + w3 * (dhGated + db) * 0.5f) / weightSum;
    else
      fusedSig = dhGated;

    heartBuf[bufIndex]   = dhGated;
    fusedBuf[bufIndex]   = fusedSig;
    breathBuf[bufIndex]  = db;
    phaseTimes[bufIndex] = millis();
    bufIndex = (bufIndex + 1) % BUF_SIZE;
    if(bufCount < BUF_SIZE) bufCount++;

    for(int i = 0; i < bufCount; i++) {
      int idx = (bufIndex - bufCount + i + BUF_SIZE) % BUF_SIZE;
      linearHeart[i]  = heartBuf [idx];
      linearBreath[i] = breathBuf[idx];
      linearFused[i]  = fusedBuf [idx];
    }

    float fs = 10.0f;
    if(bufCount > 1) {
      unsigned long span =
        phaseTimes[(bufIndex - 1 + BUF_SIZE) % BUF_SIZE] -
        phaseTimes[(bufIndex - bufCount + BUF_SIZE) % BUF_SIZE];
      if(span > 0)
        fs = (bufCount - 1) * 1000.0f / span;
    }

    if(smoothRR >= RR_MIN)
      respirationLockedClean(linearFused, bufCount, fs, smoothRR);

    int detectWindow = 0;
    if     (bufCount >= WIN_FULL) detectWindow = 3;
    else if(bufCount >= 96)       detectWindow = 2;
    else if(bufCount >= WIN_FAST) detectWindow = 1;

    if(detectWindow > 0) {

      // FIX-16: only decrement motionCooldown when patient is still — prevents
      // premature expiry during sustained motion (original code counted down
      // even while inMotion was true).
      if(!inMotion && motionCooldown > 0) motionCooldown--;  // FIX-16 + IMP-9

      if(inMotion && !wasMotion) {               // IMP-9: rising edge only
        motionCooldown = 15;
        rejectionCount = 0;                      // IMP-12: allow faster HR re-lock after motion stabilises
        rawHRWinCount  = 0;                      // GHO-1: flush stale ghost window; refill from scratch post-motion
        ghostSuspect   = false;
        rhcSuspect     = false;
      }

      // FIX-13: on motion→still transition (falling edge), flush circular buffers.
      // Post-motion data contains noisy dh/db from motion frames that would otherwise
      // take ~25.6 s (256 samples at 10 Hz) to rotate out, causing degraded PQI and
      // potential false HR/RR detection.
      if(!inMotion && wasMotion) {
        bufIndex = 0; bufCount = 0;
        memset(heartBuf,  0, sizeof(heartBuf));
        memset(breathBuf, 0, sizeof(breathBuf));
        memset(fusedBuf,  0, sizeof(fusedBuf));
        memset(phaseTimes, 0, sizeof(phaseTimes));
        // Reset NLMS weights: old clutter coefficients are invalid post-motion
        for(int i = 0; i < NLMS_TAPS; i++) { nlmsW[i] = 0; nlmsX[i] = 0; }
        nlmsMuCurrent = NLMS_MU_MAX;
      }

      if(inMotion) {
        dspTask = 0;

      } else {

        if(dspTask != 0) { pqiHeart *= 0.90f; pqiBreath *= 0.90f; }  // IMP-11

        float hrMinAdaptive = (lastStableHR > 0)
                                ? max(HR_MIN, lastStableHR - HR_BAND_HALF)
                                : HR_MIN;
        float hrMaxAdaptive = (lastStableHR > 0)
                                ? min(HR_MAX, lastStableHR + HR_BAND_HALF)
                                : HR_MAX;
        float confThreshold = (detectWindow == 1) ? HR_CONF_THRESHOLD_FAST
                                                   : HR_CONF_THRESHOLD;

        if(dspTask == 0) {
          pqiHeart  = computePQI(linearFused,  bufCount);
          pqiBreath = computePQI(linearBreath, bufCount);

          float hrAuto = 0, confHR = 0;
          bool  autoValid = detectRate(linearFused, bufCount, fs,
                                       hrMinAdaptive, hrMaxAdaptive,
                                       hrAuto, confHR, confThreshold);

          if(autoValid && pqiHeart >= PQI_LOCK_THRESHOLD) {
            float doubled = hrAuto * 2.0f;
            if(doubled <= HR_MAX && motionCooldown == 0) {
              float scoreOrig    = harmonicSpectrumScore(linearFused, bufCount, fs, hrAuto);
              float scoreDoubled = harmonicSpectrumScore(linearFused, bufCount, fs, doubled);
              bool  fftVote  = (scoreDoubled > scoreOrig * 1.5f);
              bool  rawValid = (rawHR >= HR_MIN && rawHR <= HR_MAX);
              bool  rawVeto  = rawValid && (fabsf(rawHR - hrAuto) <= fabsf(rawHR - doubled));
              // GHO-1: mechanical ghosts lie exactly on harmonic multiples; block doubling
              ghostSuspect = rawHRGhostSuspect();
              if(fftVote && !rawVeto && !ghostSuspect) hrAuto = doubled;
            }
            // RHC-1: respiration harmonic capture guard.
            // respirationLockedClean removes resp signal harmonics but cannot prevent
            // autocorr/Goertzel from locking onto residual harmonic energy.
            // Only test when smoothRR is valid and low enough (≤20 BPM) for its harmonics
            // to land meaningfully inside the HR band. Soft penalty only — at RR=20,
            // 3×RR=60 BPM is a plausible real HR, so we reduce confidence, not hard-reject.
            rhcSuspect = false;
            if(smoothRR >= RR_MIN && smoothRR <= 20.0f) {
              for(int n = 2; n <= 5; n++) {
                if(fabsf(hrAuto - n * smoothRR) <= 3.0f) {
                  confHR    *= 0.6f;   // reduce confidence stored in lastConfHR
                  rhcSuspect = true;
                  break;
                }
              }
            }
            lastAutoValid = true;
            lastHrAuto    = hrAuto;
            lastConfHR    = confHR;
          } else {
            lastAutoValid = false;
            // FIX-15: clear suspect flags when the triggering condition no longer
            // holds.  Without this, a transient vibration or harmonic coincidence
            // sets the flag permanently (orange LED / confidence penalty) until a
            // motion event occurs — the system cannot self-recover.
            ghostSuspect = false;
            rhcSuspect   = false;
          }
          dspTask = 1;

        } else if(dspTask == 1) {
          if(lastAutoValid) {
            float finalHR = lastHrAuto;
            hrConfidence  = lastConfHR * pqiHeart;

            // GHO-2: hrConfidence previously ignored pqiBreath entirely.
            // A ghost with no breathing but clean phase PQI could reach LCK.
            // Threshold 0.20: ghost/absent ≈ 0.05-0.15, shallow breath ≈ 0.20-0.40.
            if(pqiBreath < 0.20f) hrConfidence *= 0.70f;

            // GHO-1: flat rawHR (stddev < 1 BPM) → confidence halved, cannot reach LCK
            if(ghostSuspect) hrConfidence *= 0.50f;

            // TEL-2: soft distance penalty — subject outside sensor's reliable range.
            // radarDistance == 0 means no target tracked: skip penalty to avoid
            // false-penalising during the first few frames or after brief dropouts.
            if(radarDistance > 0.01f &&
               (radarDistance < 0.3f || radarDistance > 1.5f))
              hrConfidence *= 0.70f;

            hrState = (detectWindow >= 3) ? 2 : 1;

            // RHC-1: HR matched to a respiration harmonic → stay in SEN, not LCK
            if(rhcSuspect) hrState = min(hrState, 1);
            // GHO-1: ghost cannot reach LCK either
            if(ghostSuspect) hrState = min(hrState, 1);

            if(rawHR >= HR_MIN && rawHR <= HR_MAX) {
              float wDSP   = hrConfidence;
              float wRadar = rawHRStability();
              float wSum   = wDSP + wRadar;
              if(wSum > 0.01f)
                finalHR = (wDSP * finalHR + wRadar * rawHR) / wSum;
            }

            finalHR  = constrain(finalHR, HR_MIN, HR_MAX);
            finalHR  = coherenceFilter(finalHR);
            smoothHR = SAFE_FLOAT(kalmanHR(finalHR));  // FIX-7
          }
          dspTask = 2;

        } else if(dspTask == 2) {
          if(!lastAutoValid) {
            float hrSpec = detectSpectral(linearFused, bufCount, fs,
                                          hrMinAdaptive / 60.0f, hrMaxAdaptive / 60.0f);
            bool spectralPlausible =
              (rawHR > HR_MIN && fabsf(hrSpec - rawHR) < 30.0f) || (lastStableHR == 0);

            float finalHR = spectralPlausible
                              ? hrSpec
                              : (lastStableHR > 0 ? lastStableHR : smoothHR);

            if(rawHR >= HR_MIN && rawHR <= HR_MAX) {
              hrConfidence = (1.0f + rawHRStability() * 4.0f) * max(pqiHeart, 0.2f);
              hrState      = 1;

              // TEL-2: same distance penalty in spectral fallback path
              if(radarDistance > 0.01f &&
                 (radarDistance < 0.3f || radarDistance > 1.5f))
                hrConfidence *= 0.70f;

              float wDSP   = hrConfidence;
              float wRadar = rawHRStability();
              float wSum   = wDSP + wRadar;
              if(wSum > 0.01f)
                finalHR = (wDSP * finalHR + wRadar * rawHR) / wSum;
            } else {
              hrConfidence = 0;
              hrState      = 0;
            }

            if(hrState > 0) {
              finalHR  = constrain(finalHR, HR_MIN, HR_MAX);
              finalHR  = coherenceFilter(finalHR);
              smoothHR = SAFE_FLOAT(kalmanHR(finalHR));  // FIX-7            }
          }
          dspTask = 3;  // IMP-3: advance to slot 3

        } else {
          // IMP-3: slot 3 — RR detectRate + RR fusion + kalmanRR
          float rrDSP   = 0;
          float confRR  = 0;
          float finalRR = 0;

          bool rrValid = detectRate(linearBreath, bufCount, fs,
                                    RR_MIN, RR_MAX, rrDSP, confRR,
                                    RR_CONF_THRESHOLD);

          if(rrValid) {
            finalRR = rrDSP;
          } else if(rawRR >= RR_MIN && rawRR <= RR_MAX) {
            finalRR = rawRR;
          }

          if(rawRR >= RR_MIN && rawRR <= RR_MAX)
            finalRR = 0.7f * finalRR + 0.3f * rawRR;

          if(finalRR >= RR_MIN && finalRR <= RR_MAX) {
            finalRR  = constrain(finalRR, RR_MIN, RR_MAX);
            smoothRR = SAFE_FLOAT(kalmanRR(finalRR));  // FIX-7

            // SS-1: enable notch after first valid RR
            if(!notchEnabled && smoothRR >= RR_MIN) notchEnabled = true;  // SS-1
          }

          lastRRDSP   = rrDSP;    // IMP-3
          lastConfRR  = confRR;   // IMP-3
          lastRRValid = rrValid;  // IMP-3

          dspTask = 0;  // IMP-3: cycle back to slot 0
        }

      }

    }

  }  // end if(phaseDataValid)  // SS-3

  }  // end if(newData)  // FIX-2: presence SM + LCD now always execute below

  // FIX-19: only update wasMotion on frames that actually evaluated inMotion.
  // Without this, non-newData iterations wipe wasMotion=false (inMotion is local,
  // defaults to false at top of loop), causing the motion rising-edge detector to
  // re-fire on every valid motion frame instead of once at onset.
  if(newData) wasMotion = inMotion;

  // FIX-2: gate radarEvidence on newData — stale rawHR/rawRR from previous frames
  // must not sustain false presence after a radar dropout
  bool radarEvidence    = newData &&
                          ((rawHR >= HR_MIN && rawHR <= HR_MAX) ||
                           (rawRR >= RR_MIN && rawRR <= RR_MAX));

  // TEL-1 + FIX-5: radarIsPresent is the sensor's own human-detection flag (good up
  // to ~6 m).  We allow it to PREVENT false-absent (a still patient stops generating
  // HR/RR) but NOT to generate false-present (hence the humanDetected guard).
  // FIX-5: suppress keepalive when no valid HR/RR has arrived for 15 s.
  // The MR60BHA2 sensor firmware has a holdoff (10–60 s+) on its isHumanDetected()
  // flag — once set, it stays true long after the patient leaves.  Without the
  // timeout, presenceEvidence stays true indefinitely and "NO HUMAN DETECTED"
  // never reappears.  lastValidRateMs is updated whenever a physiologically valid
  // rawHR or rawRR arrives (FIX-9 gate above).
  bool keepAliveTimedOut       = (millis() - lastValidRateMs >= PRESENCE_KEEPALIVE_TIMEOUT);
  bool radarPresenceKeepAlive  = newData && humanDetected && radarIsPresent && !keepAliveTimedOut;

  bool presenceEvidence = radarEvidence || ambientEvidence || radarPresenceKeepAlive;

  if(presenceEvidence) { presentVotes++; absentVotes  = 0; }
  else                 { absentVotes++;  presentVotes = 0; }

  if(!humanDetected && presentVotes >= CONFIRM_VOTES) {
    humanDetected = true;
    presentVotes  = 0;

    if(rawHR >= HR_MIN && rawHR <= HR_MAX) {
      kfHR_x = rawHR; smoothHR = rawHR; lastStableHR = rawHR;
    }
    if(rawRR >= RR_MIN && rawRR <= RR_MAX) {
      kfRR_x = rawRR; smoothRR = rawRR;
    }

    motionLP       = TARGET_ENERGY;
    motionWarmup   = 0;
    wasMotion      = false;
    motionCooldown = 0;

    // SS-2: defensive, already cleared by resetVitals
    bufIndex = 0;
    bufCount = 0;
  }

  if(humanDetected && absentVotes >= ABSENT_VOTES) {
    humanDetected = false;

    // PROD-2: persist state to NVS before clearing — instant re-lock on next patient
    prefs.begin("rvital", false);  // read-write
    prefs.putFloat("gain", radarGain);
    prefs.putFloat("kfHR", kfHR_x);
    prefs.putFloat("kfRR", kfRR_x);
    prefs.end();

    resetVitals();
  }

  // BH1-1: poll BH1750 every 200 ms — cheap, non-blocking
  static unsigned long lastLuxRead = 0;
  if(bh1750Ready && millis() - lastLuxRead >= 200) {
    lastLuxRead = millis();
    float lux = lightMeter.readLightLevel();
    if(lux >= 0) luxLevel = lux;  // < 0 means sensor not ready yet; keep last value
  }

  // BH1-1: night mode — room is dark, slow the LCD refresh to avoid flicker
  // and reduce LED brightness so it doesn't disturb sleep monitoring.
  // effectiveInterval is what the LCD block tests against.
  unsigned long effectiveInterval = (luxLevel < 10.0f)
                                      ? DISPLAY_INTERVAL * 4
                                      : DISPLAY_INTERVAL;

  if(lcdConnected && millis() - lastDisplay > effectiveInterval) {
    lastDisplay = millis();

    if(humanDetected) {
      // PROD-3: feed rolling HR variance buffer (only when locked or sensing)
      if(hrState > 0 && smoothHR >= HR_MIN) {
        hrVarBuf[hrVarIdx] = smoothHR;
        hrVarIdx = (hrVarIdx + 1) % HR_VAR_WIN;
        if(hrVarCount < HR_VAR_WIN) hrVarCount++;
      }

      // PROD-3: compute 10-sample rolling stddev for ± display
      int hrSD = 0;
      if(hrVarCount >= 3) {
        float mean = 0;
        for(int i = 0; i < hrVarCount; i++) mean += hrVarBuf[i];
        mean /= hrVarCount;
        float var = 0;
        for(int i = 0; i < hrVarCount; i++) {
          float d = hrVarBuf[i] - mean; var += d * d;
        }
        hrSD = (int)(sqrtf(var / hrVarCount) + 0.5f);
      }

      // PROD-3: line 1 format — "HR 72±3 RR12 T36" (16 chars)
      // Temp dropped to integer to free columns for ± variance display.
      char line1[20];
      int iHR = (int)SAFE_FLOAT(smoothHR);  // FIX-7
      int iRR = (int)SAFE_FLOAT(smoothRR);  // FIX-7
      if(hrVarCount >= 3) {
        if(isnan(smoothTemp))
          snprintf(line1, sizeof(line1), "HR%3d", iHR);
        else
          snprintf(line1, sizeof(line1), "HR%3d", iHR);
        // We'll compose the line with custom ± char below
      } else {
        if(isnan(smoothTemp))
          snprintf(line1, sizeof(line1), "HR%3d   RR%2d T--", iHR, iRR);
        else
          snprintf(line1, sizeof(line1), "HR%3d   RR%2d T%2d", iHR, iRR, (int)smoothTemp);
      }

      lcd.setCursor(0, 0);
      if(hrVarCount >= 3) {
        lcd.print(line1);         // "HR 72" (5 chars)
        lcd.write(byte(6));       // ± glyph (1 char)
        char rest[11];
        if(isnan(smoothTemp))
          snprintf(rest, sizeof(rest), "%-2dRR%2d T--", hrSD, iRR);
        else
          snprintf(rest, sizeof(rest), "%-2dRR%2d T%2d", hrSD, iRR, (int)smoothTemp);
        lcd.print(rest);
      } else {
        lcd.print(line1);
      }

      lcd.setCursor(0, 1);
      if(inMotion)           lcd.print("MOV ");
      else if(hrState == 2)  lcd.print("LCK ");
      else if(hrState == 1)  lcd.print("SEN ");
      else                   lcd.print("--- ");

      int level = constrain((int)(hrConfidence * 2), 0, 20);
      drawBar(level);
      lcd.print("       ");

    } else {
      lcd.setCursor(0, 0);
      lcd.print("NO HUMAN        ");
      lcd.setCursor(0, 1);
      lcd.print("DETECTED        ");
    }
  }

  // TEL-D: optional Serial telemetry — uncomment #define DEBUG_TELEMETRY at top of file.
  // Output is throttled to ~1 Hz to avoid flooding the UART during normal operation.
#ifdef DEBUG_TELEMETRY
  static unsigned long lastDebugPrint = 0;
  if(millis() - lastDebugPrint >= 1000) {
    lastDebugPrint = millis();
    Serial.printf("[TEL] dist=%.2fm phaseDelta=%.3frad present=%d HR=%.0f RR=%.0f conf=%.2f\n",
                  radarDistance, phaseDelta, (int)radarIsPresent, smoothHR, smoothRR, hrConfidence);
  }
#endif

  // WS2-1: update RGB LED every loop pass — fast, non-blocking
  updateLED(inMotion);
}
