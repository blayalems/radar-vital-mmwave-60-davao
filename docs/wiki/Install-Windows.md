# Install Windows

The Windows build is the primary station workflow. It packages the dashboard and
a Python trainer sidecar so the operator can run sessions from one PC.

## Before installing

- Use a Windows 10/11 PC with WebView2 available.
- Connect the XIAO ESP32-C6 / MR60BHA2 radar over USB.
- Flash the current firmware, `radar_vital_v16_5_1.ino`, before validating the
  session-data metrics.
- Keep the PC physically secured. Session CSV files and operator profiles live
  on this host.
- If the installer is not yet production signed, expect Windows SmartScreen or
  Defender warnings. Code signing improves identity, but it does not guarantee
  instant SmartScreen reputation.

## Install and launch

1. Download the NSIS installer from the GitHub Release or CI artifact.
2. Run the installer. Accept the EULA only if the draft terms match your use.
3. Launch Radar Vital from the Start menu or desktop shortcut.
4. Confirm the Python Server card shows the trainer sidecar as running.
5. Create or unlock an operator profile with the PIN flow.
6. Use the Home tab preflight before starting a real session. Advisory
   Python/firmware-source/BLE warnings do not block Start if radar collection,
   storage, schema, and clock checks pass.

By default the trainer binds to `127.0.0.1:8765`, which is local-only. This is
the safest mode for a single PC because phones and other devices cannot reach
the trainer. Use LAN mode only when a phone or tablet must pair with the host.

The EXE does not depend on a Chromium local-device-discovery prompt for BLE.
AiLink oximeter reference capture runs through the bundled Python/WinRT sidecar;
if the oximeter is unavailable, the radar session is preserved and marked
`radar_only` for analysis.

## LAN use

LAN mode must be explicit. Start or restart the trainer with `--bind lan`, allow
Windows Firewall access on private networks, then use the pairing page and
single-use PIN described in [Pairing and LAN](Pairing-and-LAN). Do not place the
trainer on public Wi-Fi or an untrusted network.

## Updates

Release builds publish a manifest so older clients can see update availability.
Treat RC builds as lab-only. For RTM installs, verify the release tag, installer
hash, and signing status before distributing to operators.
