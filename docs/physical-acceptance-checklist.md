# Physical Hardware Acceptance Checklist (v12 stable)

This document serves as the official QA manual and step-by-step physical test checklist to approve the packaged Capacitor APK (Android) and Tauri EXE (Windows) builds. Because native BLE and secure LAN authentication require direct hardware interface access, complete verification must be signed off using this checklist.

---

## 1. Required Device Lab Setup

Ensure the following hardware components are available:
1. **Target Mobile Device**: Android 10+ (tested against Pixel baseline).
2. **Target Windows Machine**: Windows 10 or 11 with a working Bluetooth 4.2+ BLE USB adapter or integrated chip.
3. **Reference BLE Device**: AiLink Pulse Oximeter or compatible GATT simulator.
4. **Target Firmware Controller**: XIAO ESP32-C6 loaded with `radar_vital_v16_3_0.ino`.
5. **LAN Test Environment**: Both the trainer server (PC) and the mobile device must be connected to the exact same subnet (e.g., local Wi-Fi router).

---

## 2. Protocol Validation Checklist

### Phase A: BLE Scan & Service Allowance
- [ ] Turn on the AiLink reference oximeter (verify it is in pairing/broadcasting mode).
- [ ] Launch the packaged application (Capacitor APK or Tauri EXE).
- [ ] Navigate to the **Home** tab and activate the **Native BLE acceptance probe**.
- [ ] Verify that the scan results populate and list the reference oximeter.
- [ ] Confirm that non-approved UUID services are rejected by the native backend filters.

### Phase B: Secure LAN PIN Exchange
- [ ] Run the trainer python script in LAN mode:
  ```bash
  python radar_vital_trainer_v12_for_v16_0.py serve --bind lan
  ```
- [ ] Scan the displayed operator QR code or enter the paired PIN displayed on the `/pair` route.
- [ ] Confirm that once authenticated, the PIN is deleted from the server and the app receives an `X-RVT-Auth` token.
- [ ] Verify that requests to LAN physiological endpoints without this token are blocked with a `401 Unauthorized` response.

### Phase C: 1 Hz Stream and EventSource Stability
- [ ] Start a live telemetry recording session from the dashboard.
- [ ] Verify the Overview graphs draw vital parameters (Heart Rate and Respiration) at exactly 1 Hz.
- [ ] Confirm that minimizing the application for 60 seconds does not cause the service worker EventSource to drop or fail reconnecting.
- [ ] Verify the **DEMO** banner displays if sandbox fallback occurs, and that sandbox telemetry is isolated inside the IndexedDB store.

### Phase D: Firmware Robustness Fault Injection
- [ ] Disconnect LCD SDA mid-session and verify the serial DATA cadence remains at 1 Hz with loop timing still within the pre-fault budget ±5 ms.
- [ ] Confirm serial logs show LCD/I2C retry backoff increasing from 3 s toward the 5 min cap, and that `i2c_recover_count` / `lcd_reinit_count` increase in Live Audit.
- [ ] Reconnect LCD SDA and verify the display recovers within one active backoff interval without rebooting the controller.
- [ ] Pull radar TX mid-session, then restore it; verify recovery logs are visible and lock is reacquired without a firmware reboot.
- [ ] Power-cycle the controller eight times and verify `[BOOT] reset_reason=<n> (<name>)` logs appear and the NVS reset-reason ring advances across boots.
- [ ] Force or simulate repeated NVS write failures and verify three consecutive failures trigger one namespace reopen attempt, then NVS writes are disabled for the rest of that boot with a loud log line.

### Phase E: PR59 Power and Thermal Management
- [ ] Build and flash default firmware (`RV_POWER_SAVE 0`) and verify DATA cadence remains 1 Hz ±2 % for at least 30 min with no LCD/LED/MLX behavior change versus the v16.2.0 firmware-behavior baseline.
- [ ] Build with `RV_POWER_SAVE 1`, leave the radar scene empty for at least 60 s, and verify `[POWER] idle power save entered` logs with CPU frequency at 80 MHz and MLX poll interval at 10 s.
- [ ] While power save is active, introduce a subject and verify `[POWER] idle power save exited` appears within one display frame and presence reacquisition latency matches the v16.2.0 bench-timing baseline.
- [ ] Record idle current before and after the `RV_POWER_SAVE 1` transition, including LCD backlight state and LED duty, and attach the measured mA delta to the release notes.
- [ ] Verify the radar UART/DSP DATA cadence remains 1 Hz ±2 % during power-save entry, steady idle, and exit.
- [ ] Heat the enclosure or bench environment enough to cross 75 C internal chip temperature and verify `[THERMAL] chip_temp_c=<value>` warns no more than once per minute.
- [ ] With `RV_LCD_LUX_BACKLIGHT 1`, sweep BH1750 input below 8 lux and above 15 lux and verify the LCD backlight changes only across those hysteresis thresholds.

### Phase F: v16.3 RTM Shell, Dialogs, and Store-Readiness QA
- [ ] Android APK: verify edge-to-edge layout does not clip the topbar, bottom navigation, consent dialog, onboarding tutorial, recovery-code dialog, or issue-report preview on a Pixel-class phone.
- [ ] Android APK: open each new dialog (consent, tutorial, recovery code, issue preview), use the system back gesture, and verify it either dismisses only when allowed or returns to the previous step without losing operator/session state.
- [ ] PWA on Android Chrome: verify the Home install banner appears after `beforeinstallprompt`, can be dismissed, and does not reappear in the same accepted/dismissed session.
- [ ] PWA on desktop Chrome/Edge: verify install prompt handling, installed-app launch, offline shell load, and network-only `/api/*` behavior.
- [ ] Windows EXE: run the NSIS installer and confirm the EULA page appears before installation; on first launch, confirm the app still shows the first-run RA 10173/UM consent gate before operator onboarding.
- [ ] Forgot-PIN flow: on Android APK, installed PWA, and Windows EXE, create an operator, save the recovery code, lock the station, reset with the recovery code, and verify the rotated recovery code appears once and the old code is rejected.
- [ ] Windows EXE host reset: from the locked screen, use **Reset from this computer** and verify it only works on the loopback-hosted EXE and returns a new recovery code.
- [ ] Issue reporting: from Settings on EXE and APK, preview the diagnostic bundle, toggle diagnostics off and on, then open the external GitHub issue URL; verify no token, PIN, or private key material appears in the URL/body.
- [ ] Accessibility: keyboard-tab through consent, tutorial, recovery-code, and issue-preview dialogs; verify focus starts inside the dialog, remains trapped, Escape behavior matches the dialog contract, and focus returns to the invoking control when the dialog closes.
- [ ] PWA privacy URL: from the hosted Pages build, open `/privacy.html` and `/terms.html`; verify both render without the trainer API and match the in-app legal copy version.

---

## 3. Reference GATT UUID Profile Specifications

Verify that the target BLE device communicates on the following allowlisted profile characteristics:
* **Primary Service UUID**: `0000ffe0-0000-1000-8000-00805f9b34fb`
* **Notify Characteristic UUID**: `0000ffe2-0000-1000-8000-00805f9b34fb`
