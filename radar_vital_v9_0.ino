/* radar_vital_v9_0.ino
 *
 * XIAO ESP32-C6 + MR60BHA2 60 GHz FMCW radar + MLX90614 + HD44780 LCD
 *
 * Change log from v8.8 → v9.0:
 * +---------+---------------------------------+------------------------------------------------+
 * | ID      | Location                        | Change                                         |
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
 * | IMP-8   | LCD snprintf                      | line1[20], sizeof, %5.1f                       |
 * | IMP-9   | dspTask motion block              | Unconditional cooldown decrement; rising-edge   |
 * |         |                                   | reset                                          |
 * | IMP-10  | detectSpectral()                  | Adaptive bin count vs DFT resolution           |
 * | IMP-11  | dspTask non-zero path             | pqiHeart/Breath 0.90x decay per frame          |
 * | IMP-12  | Motion rising-edge block          | rejectionCount=0 on motion onset               |
 * | IMP-13  | loop() phase call                 | total_phase → _unused_phase                    |
 * | IMP-14  | MLX read block                    | 1 Hz polling gate                              |
 * +---------+---------------------------------+------------------------------------------------+
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

#include "Seeed_Arduino_mmWave.h"
#include <Adafruit_MLX90614.h>

#ifdef ESP32
  #include <HardwareSerial.h>
  HardwareSerial mmWaveSerial(0);
#else
  #define mmWaveSerial Serial1
#endif

#ifndef PI
  #define PI 3.14159265358979323846f
#endif

SEEED_MR60BHA2    mmWave;
hd44780_I2Cexp    lcd;
Adafruit_MLX90614 mlx;

bool lcdConnected = false;
bool mlxReady     = false;

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
const float NLMS_MU  = 0.03f;
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
  return s1*s1 + s2*s2 - coeff*s1*s2;  // IMP-2: magnitude squared; callers apply sqrtf for amplitude
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

void setup() {
  Serial.begin(115200);
  buildSinLUT();
  Wire.begin();

  mmWaveSerial.begin(115200);
  mmWave.begin(&mmWaveSerial);

  mlxReady = mlx.begin();

  int addresses[] = {0x23, 0x27};
  for(int i = 0; i < 2; i++) {
    lcd = hd44780_I2Cexp(addresses[i]);
    if(lcd.begin(16, 2) == 0) {
      lcdConnected = true;
      lcd.backlight();
      for(int c = 0; c < 5; c++) lcd.createChar(c + 1, barChars[c]);
      break;
    }
  }

  if(lcdConnected) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Radar Vital v9.0");
    delay(1500);
    lcd.clear();
  }
}

void loop() {

  bool inMotion = false;

  if(!mlxReady && millis() - lastMlxRetry > MLX_RETRY_INTERVAL) {
    mlxReady     = mlx.begin();
    lastMlxRetry = millis();
  }

  bool newData = mmWave.update(80);
  if(!newData) return;

  mmWave.getHeartRate(rawHR);
  mmWave.getBreathRate(rawRR);

  if(rawHR >= HR_MIN && rawHR <= HR_MAX && !wasMotion) {
    rawHRWin[rawHRWinIdx] = rawHR;
    rawHRWinIdx = (rawHRWinIdx + 1) % RAW_STAB_WIN;
    if(rawHRWinCount < RAW_STAB_WIN) rawHRWinCount++;
  }

  // IMP-14: MLX90614 polling downsampled to 1 Hz
  static unsigned long lastTempRead = 0;
  bool ambientEvidence = false;  // IMP-14: declared outside guard for scope

  if(mlxReady && millis() - lastTempRead >= 1000) {  // IMP-14: 1 Hz
    lastTempRead = millis();

    float obj = mlx.readObjectTempC();
    float amb = mlx.readAmbientTempC();

    static int mlxErrors = 0;
    if(!isnan(obj)) {
      mlxErrors = 0;
      if(isnan(smoothTemp)) smoothTemp = obj;
      else                  smoothTemp = 0.9f * smoothTemp + 0.1f * obj;
    } else {
      if(++mlxErrors > 3) { mlxReady = false; mlxErrors = 0; }
    }

    float drift     = fabsf(amb - prevAmbient);
    prevAmbient     = amb;
    ambientEvidence = (drift > 0.2f);
  }

  // SS-3: phaseDataValid flag replaces hard return
  float _unused_phase, breath_phase, heart_phase;  // IMP-13: total_phase not used
  bool phaseDataValid = mmWave.getHeartBreathPhases(_unused_phase, breath_phase, heart_phase);  // SS-3

  if(phaseDataValid) {  // SS-3: phase DSP only when valid

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

      if(calibrationCount >= CALIBRATION_FRAMES) {
        baselineEnergy /= calibrationCount;
        baselineEnergy  = max(baselineEnergy, 1e-6f);
        radarGain       = constrain(TARGET_ENERGY / baselineEnergy, MIN_GAIN, MAX_GAIN);
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
    bool  roughMotion = (rawEnergy > max(motionLP * 5.0f, 1e-6f));

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
    float breathGate      = fabsf(lut_sin(breathInstPhase));  // SS-4: was lut_cos
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

      if(motionCooldown > 0) motionCooldown--;  // IMP-9: always decrement
      if(inMotion && !wasMotion) {               // IMP-9: rising edge only
        motionCooldown = 15;
        rejectionCount = 0;                      // IMP-12
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
              if(fftVote && !rawVeto) hrAuto = doubled;
            }
            lastAutoValid = true;
            lastHrAuto    = hrAuto;
            lastConfHR    = confHR;
          } else {
            lastAutoValid = false;
          }
          dspTask = 1;

        } else if(dspTask == 1) {
          if(lastAutoValid) {
            float finalHR = lastHrAuto;
            hrConfidence  = lastConfHR * pqiHeart;
            hrState       = (detectWindow >= 3) ? 2 : 1;

            if(rawHR >= HR_MIN && rawHR <= HR_MAX) {
              float wDSP   = hrConfidence;
              float wRadar = rawHRStability();
              float wSum   = wDSP + wRadar;
              if(wSum > 0.01f)
                finalHR = (wDSP * finalHR + wRadar * rawHR) / wSum;
            }

            finalHR  = constrain(finalHR, HR_MIN, HR_MAX);
            finalHR  = coherenceFilter(finalHR);
            smoothHR = kalmanHR(finalHR);
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
              smoothHR = kalmanHR(finalHR);
            }
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
            smoothRR = kalmanRR(finalRR);

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

  // SS-3: These ALWAYS run (presence SM + LCD outside phaseDataValid gate)
  wasMotion = inMotion;

  bool radarEvidence    = (rawHR >= HR_MIN && rawHR <= HR_MAX) ||
                          (rawRR >= RR_MIN && rawRR <= RR_MAX);
  bool presenceEvidence = radarEvidence || ambientEvidence;

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
    resetVitals();
  }

  if(lcdConnected && millis() - lastDisplay > DISPLAY_INTERVAL) {
    lastDisplay = millis();

    if(humanDetected) {
      char line1[20];  // IMP-8: expanded from 18
      if(isnan(smoothTemp))
        snprintf(line1, sizeof(line1), "HR%3d RR%2d T--.-",  // IMP-8: sizeof
                 (int)smoothHR, (int)smoothRR);
      else
        snprintf(line1, sizeof(line1), "HR%3d RR%2d T%5.1f",  // IMP-8: %5.1f + sizeof
                 (int)smoothHR, (int)smoothRR, smoothTemp);

      lcd.setCursor(0, 0);
      lcd.print(line1);

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
}
