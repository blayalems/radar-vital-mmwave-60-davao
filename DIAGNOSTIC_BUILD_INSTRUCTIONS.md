# Diagnostic Build Instructions — UART/USB RCA Experiment

> **Branch:** `debug/pr79-investigation`
> **Base:** PR #79 (`codex/firmware-state-recovery`)
> **Purpose:** Isolate the cold-boot calibration failure and post-calibration restart

---

## Quick Start

1. **Download** this branch (clone or download ZIP from GitHub)
2. **Open** `radar_vital_v16_4_0.ino` in Arduino IDE
3. **Change one line** near the top of the file (line ~231):

```cpp
#define DIAG_BUILD 'A'   // ← change to 'A', 'B', 'C', or 'D'
```

4. **Compile and flash** to the XIAO ESP32-C6
5. **Cold boot** with USB data cable and capture serial output

---

## Build Variants

| Build | Radar UART | CSV Logging | Purpose |
|:---:|:---:|:---:|:---|
| **`'A'`** | UART0 (current) | OFF | Isolates whether CSV output causes the restart |
| **`'B'`** | UART1 on GPIO16/17 | OFF | Isolates whether UART0 assignment causes calibration failure |
| **`'C'`** | UART1 on GPIO16/17 | 1 row/s | Tests moderate USB output after UART fix |
| **`'D'`** | UART1 on GPIO16/17 | 5 rows/s | Reproduces full output pressure after UART fix |

---

## ⚠️ BEFORE Builds B, C, D — Verify Your Wiring

Builds B/C/D move the radar from UART0 to UART1 with **explicit pin assignment**.
The default pins are:

| Pin | GPIO | Direction | Connected to |
|:---:|:---:|:---:|:---|
| D7 | GPIO17 | ESP32 RX ← | MR60BHA2 TX (radar data out) |
| D6 | GPIO16 | ESP32 TX → | MR60BHA2 RX (radar data in) |

**If your physical wiring uses different pins**, edit these two lines (near line ~242):

```cpp
static constexpr int RADAR_RX_PIN = 17;  // D7/GPIO17 — ESP32 RX ← MR60BHA2 TX
static constexpr int RADAR_TX_PIN = 16;  // D6/GPIO16 — ESP32 TX → MR60BHA2 RX
```

---

## Arduino IDE Settings

Ensure these board settings for the XIAO ESP32-C6:

| Setting | Value |
|:---|:---|
| Board | XIAO_ESP32C6 |
| USB CDC On Boot | **Enabled** (verify — this is critical to the diagnosis) |
| Upload Speed | 921600 |
| Flash Size | 4MB |

---

## Test Procedure (Same for All Builds)

### Setup
- Use the **same PC**, **same USB data cable**, **same physical wiring** for every build
- Ensure a **cold power cycle** (unplug USB, wait 3 seconds, replug)

### Steps

1. Close **all** serial applications (Arduino Serial Monitor, Trainer, PuTTY, VS Code serial, etc.)
2. Unplug the USB cable from the XIAO ESP32-C6
3. Wait 3 seconds
4. Plug in the USB data cable
5. **Immediately** open Arduino Serial Monitor at 115200 baud
6. Watch the LCD and serial output for 90 seconds (calibration timeout is 60s + buffer)
7. Record:
   - Did the LCD show "Calibration Done!" or "Using Defaults"?
   - Did the device restart? If so, after how long?
   - Did calibration succeed on the second boot?

### Required Serial Output to Capture

Copy these lines from the Serial Monitor for each build:

```
[CONFIG] Serial is USB CDC              ← or WARNING: Serial is UART0
[DIAG] Build X | radar_uart=N | ...     ← confirms which build is flashed
[BOOT] reset_reason=N (reason_text)     ← CRITICAL — identifies restart cause
[SETTINGS] Gain: X.XXX                  ← NVS restore (may not appear on fresh NVS)
[CAL] TIMEOUT count=N nvs_valid=N ...   ← appears ONLY if calibration times out
```

If the device restarts, **the second boot's `[BOOT] reset_reason` is the most important line**.

---

## Interpreting Results

| Result | Conclusion |
|:---|:---|
| **A times out, B succeeds** | ✅ UART0 assignment is the cause |
| **A succeeds (no timeout)** | CSV output or debug traffic was influencing before the restart |
| **B succeeds, C or D restarts** | ✅ USB serial backpressure is the cause |
| **All fail despite UART1 + no logging** | Investigate host reset behavior, USB build config, or power |
| **Reset reason = `brownout`** | Stop firmware diagnosis — check power supply, grounding, cable |
| **Reset reason = `task_wdt` or `panic`** | ✅ Confirms watchdog timeout — serial output is blocking the loop |
| **Reset reason = `software` or `external`** | USB host DTR/RTS is resetting the ESP32 |

---

## What This Patch Changes (Technical)

All changes are conditional on `DIAG_BUILD` and do not alter runtime behavior when set to `'D'` (which matches the original firmware minus the added diagnostics):

1. **Build selector** — `#define DIAG_BUILD` at line ~231 controls all other conditionals
2. **UART declaration** — Builds B/C/D use `HardwareSerial mmWaveSerial(1)` instead of `(0)`
3. **Pin assignment** — Builds B/C/D pass explicit RX/TX pins to `mmWaveSerial.begin()`
4. **LOG_MODE** — Builds A/B set `LOG_MODE 0` (no CSV output)
5. **LOG_INTERVAL_MS** — Build C sets 1000ms (1 row/s), Build D keeps 200ms (5 rows/s)
6. **Boot diagnostics** — All builds print `[CONFIG]`, `[DIAG]` at startup
7. **Calibration timeout telemetry** — All builds print `[CAL] TIMEOUT` with packet count, NVS state, phase age, and UART backlog when calibration falls back

No functional firmware logic is changed. The presence FSM, vital signs DSP, LCD rendering, and all recovery state machines are untouched.

---

## After Testing

Report the captured serial output for all four builds. The results will determine:
- Whether to permanently move the radar to UART1 (the architectural fix)
- Whether serial output throttling or a ring buffer is needed
- The actual reset reason for the post-calibration restart
- Whether PR #79 can proceed to merge

---

*Diagnostic patch produced by Claude Opus 4.6 · 2026-07-30*
