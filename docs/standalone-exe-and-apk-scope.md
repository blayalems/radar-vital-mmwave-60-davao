# Standalone EXE and APK Scope

## Goal

This branch makes the Windows Tauri installer the standalone operator-console path by bundling the Python trainer as a local sidecar. The Android APK remains a LAN companion client.

## Windows EXE

The Windows EXE target is:

```text
Tauri shell -> bundled PyInstaller trainer sidecar -> http://127.0.0.1:8765 -> Angular dashboard
```

The installer should not require the operator to manually run `radar_vital_trainer_v12_for_v16_0.py`. On app startup, Tauri launches the bundled `rvt-trainer` sidecar in local-bind mode, pins native API/download requests to `http://127.0.0.1:8765`, and allows the Angular dashboard to talk to the same local trainer through the native bridge.

### Acceptance criteria

- The NSIS installer contains the Tauri shell and the `rvt-trainer` sidecar.
- Launching the installed EXE starts the trainer sidecar automatically.
- `/api/health` and `/api/status` respond on `http://127.0.0.1:8765` without a separate Python terminal.
- Manual demo mode can be turned off while the local trainer sidecar is healthy.
- Serial port discovery and live session start remain trainer-owned.
- Closing the Tauri window terminates the sidecar process.
- The app can still connect to an external LAN trainer through the existing Settings/PIN path when needed.

## Android APK

The APK target remains:

```text
Capacitor APK -> PIN-paired LAN API -> trainer running on Windows/laptop/PC
```

The APK does not embed the full Python trainer in this milestone. Android-only trainer execution would require replacing the current desktop serial path with Android USB-host/OTG code or promoting the firmware BLE GATT path, then validating storage, background-service, battery, and native dependency behavior on real phones.

### APK acceptance criteria

- APK pairs with a trainer using the existing six-digit PIN flow.
- APK receives live 1 Hz telemetry from the trainer over LAN.
- APK can start/stop sessions through the protected API.
- APK clearly remains a companion client and does not claim to read the radar USB serial path directly.
- Native BLE acceptance probe remains labeled as a hardware probe, not a trainer replacement.

## Release gates

Before this branch is considered merge-ready:

1. `python scripts/build-trainer-sidecar.py --self-test` passes on Windows.
2. `cargo test --manifest-path src-tauri/Cargo.toml --verbose` passes.
3. `cargo tauri build --verbose --bundles nsis` produces an installer with the sidecar.
4. The installed EXE is manually smoke-tested on Windows with the XIAO/MR60BHA2 attached.
5. APK LAN pairing is regression-tested against the trainer, with no embedded-Python claim added.
