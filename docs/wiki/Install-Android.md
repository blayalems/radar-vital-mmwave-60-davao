# Install Android

The Android app is a companion client. It connects to a Windows trainer over the
local network; it does not include the Python trainer and cannot read the USB
radar directly.

## Closed testing

The v16.3 release plan targets Google Play closed testing before production.
Play personal-account promotion has practical gates: a closed-testing cohort
needs at least 12 opted-in testers and 14 days of active testing before
production eligibility. AAB signing, Play listing review, data-safety answers,
and the live privacy-policy URL must all be ready before upload.

If the app is distributed outside Play during RC testing, use only the APK/AAB
from the release workflow. Do not sideload random local debug builds onto study
devices unless the tester understands that the build is unsigned or debug-signed.

## Pair with a trainer

1. Start the Windows trainer in LAN mode on a trusted Wi-Fi network.
2. Confirm the PC and phone are on the same subnet.
3. Open `http://<pc-ip>:8765/pair` on the trainer PC.
4. In the Android app, enter the trainer URL and the six-digit PIN, or scan the
   QR code if the build exposes the camera pairing flow.
5. Confirm the app shows connected live telemetry before starting a session.

The pairing PIN is single-use and expires. Restart the trainer or pairing flow
to generate a fresh PIN.

## Limits

- iOS Safari Web Bluetooth is not supported by the web platform and is outside
  this Android install path.
- The Android app depends on the host trainer for serial radar access, session
  storage, authentication, and export.
- Demo mode is safe for training and UI review but does not capture real data.
