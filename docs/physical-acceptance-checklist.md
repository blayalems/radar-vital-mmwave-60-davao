# Physical Hardware Acceptance Checklist (v12 stable)

This document serves as the official QA manual and step-by-step physical test checklist to approve the packaged Capacitor APK (Android) and Tauri EXE (Windows) builds. Because native BLE and secure LAN authentication require direct hardware interface access, complete verification must be signed off using this checklist.

---

## 1. Required Device Lab Setup

Ensure the following hardware components are available:
1. **Target Mobile Device**: Android 10+ (tested against Pixel baseline).
2. **Target Windows Machine**: Windows 10 or 11 with a working Bluetooth 4.2+ BLE USB adapter or integrated chip.
3. **Reference BLE Device**: AiLink Pulse Oximeter or compatible GATT simulator.
4. **Target Firmware Controller**: XIAO ESP32-C6 loaded with `radar_vital_v16_1_0.ino`.
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

---

## 3. Reference GATT UUID Profile Specifications

Verify that the target BLE device communicates on the following allowlisted profile characteristics:
* **Primary Service UUID**: `0000ffe0-0000-1000-8000-00805f9b34fb`
* **Notify Characteristic UUID**: `0000ffe2-0000-1000-8000-00805f9b34fb`
