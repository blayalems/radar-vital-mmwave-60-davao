# Operator Quickstart — Radar Vital mmWave 60 GHz

This guide is for clinical or research operators who will run monitoring sessions.
No command-prompt work is required for the EXE path; basic network familiarity is
needed for the APK/PWA path.

---

## 1. Setup — Windows EXE

The Windows installer bundles the Python trainer as a background sidecar.

1. **Install**: run the NSIS installer (`RVT-Installer-*.exe`). Accept the
   Windows SmartScreen prompt if it appears.
2. **Launch**: double-click the desktop shortcut. The trainer sidecar starts
   automatically in local mode (`http://127.0.0.1:8765`). The dashboard opens in
   the embedded WebView. No separate terminal window is needed.
   - The Settings card "Python Server" shows a status chip: **Running** (green)
     or **Offline** (red). If **Offline**, click **Start** in that card.
3. **Local use (single PC)**: the default local binding is sufficient. Navigate
   to the Home tab, run preflight, and start a session.

### Enabling phone access (LAN mode)

The trainer binds `127.0.0.1` by default, so no network traffic leaves the PC.
To pair a phone:

> **Note**: LAN mode is currently configured by launching the trainer with
> `--bind lan` from the command line (or the Tauri sidecar restart path). The
> Settings card "Python Server" shows **Restart** — this restarts the sidecar and
> re-generates a PIN. The exact UI flow for LAN-mode restart is verified in the
> Python Server card in `web/src/app/components/settings/settings.component.html`.

Steps:
1. Open **Settings** (gear icon) on the dashboard.
2. In the **Python Server** card, click **Restart** while the trainer is running
   with `--bind lan`. The sidecar restarts and generates a fresh six-digit PIN
   with a five-minute TTL.
3. Navigate to `http://<pc-ip>:8765/pair` on the PC browser — a QR code and the
   six-digit PIN are displayed. Note that the PIN is **single-use**: once a
   phone consumes it, it is deleted from the trainer's memory. To pair a second
   device, click **Restart** again to generate a new PIN.
4. On the phone, open the dashboard (APK or PWA) and go to **Settings →
   Source Mode**. Enter the trainer origin (`http://<pc-ip>:8765`) and the
   six-digit PIN, then tap **Pair with PIN**. The dashboard receives an
   `X-RVT-Auth` token stored in session storage only — re-pair after every
   trainer restart.
5. After pairing the QR/PIN page shows the paired confirmation. The phone can
   now access all physiological endpoints.

**Firewall note**: Windows Defender may prompt to allow `rvt-trainer.exe` on
private networks. Click **Allow access**. If pairing fails, confirm both devices
are on the same subnet and the firewall allows TCP on port 8765.

**Five-minute PIN TTL**: if the QR page has been open for more than five minutes
without scanning, restart the sidecar to get a fresh PIN. Five consecutive wrong
PIN attempts from one IP address trigger a one-minute pairing cooldown; wait for
the cooldown or restart the sidecar.

---

## 2. Setup — Android APK / PWA (companion client)

The APK and the GitHub Pages PWA are **companion clients only** — they connect
to a trainer running on a Windows PC over the local Wi-Fi network. They do not
embed the radar serial path.

### Demo mode (no server needed)

Open the app. In **Settings → Source Mode**, turn on **Demo Mode**. The
dashboard immediately shows synthesised telemetry. This is for UI orientation
only; no real radar data is captured in demo mode.

### Pairing with a LAN trainer

Prerequisites: the PC trainer must be running with `--bind lan` (see Windows EXE
section above, step 3 onward). Both devices must be on the same Wi-Fi subnet.

**Option A — QR scan**:
1. On the PC at `http://<pc-ip>:8765/pair`, a QR code encodes
   `http://<pc-ip>:8765/?pair=<PIN>`.
2. In the app, go to **Settings → Source Mode**, tap **Pair with server**, then
   scan the QR code with the phone camera. The app decodes the origin and PIN
   automatically and exchanges it for an `X-RVT-Auth` token.

**Option B — manual address + PIN**:
1. In **Settings → Source Mode**, enter `http://<pc-ip>:8765` in the **Trainer
   server origin** field.
2. Enter the six-digit PIN shown on the PC in the **Six-digit LAN pairing PIN**
   field, then tap **Pair with PIN**.

After pairing, the **Source Mode** card shows **Connected** and the session
controls become available. Re-pair after every trainer restart.

---

## 3. Subject placement

Mount the sensor (XIAO ESP32-C6 carrier with MR60BHA2 module) so it faces the
subject's chest from the side at roughly chest height (sternum level). Keep the
front face of the module perpendicular to the subject's torso.

### Distance guidance

| Zone | Distance | Typical signal quality |
|---|---|---|
| Optimal | 40–100 cm | Reliable heart and breath phase lock |
| Good | 100–140 cm | Usable; minor SNR reduction |
| Acceptable | 140–180 cm | Possible; lower confidence expected |
| Beyond range | > 180 cm | Not recommended for capture |

The MR60BHA2 module's specified detection range is 0.4–2 m; Seeed's getting-started
guidance ([MR60BHA2 wiki](https://wiki.seeedstudio.com/getting_started_with_mr60bha2_mmwave_kit/))
recommends positioning the subject typically within 1.5 m. The Home radar
scope shows the live range with a colored placement-zone chip (Optimal / Good /
Acceptable / Too close / Out of range) and an actionable hint line under the
Range metric — these zones come from `placementZone()` in
`web/src/app/components/home/home.component.ts` and mirror the firmware's
distance-confidence bands.

### Practical tips

- Avoid metal surfaces or furniture directly behind the sensor face.
- The subject should be in a relaxed seated or supine position; large voluntary
  motion (talking, repositioning) degrades signal quality temporarily.
- The live radar scope on the Home tab shows "Subject detected" or "Searching
  range" based on the firmware presence flag — use this to confirm detection
  before starting a session.

---

## 4. Reading signal quality

### Waves tab — SQI ribbons

Open the **Waves** tab in Live view. Below each waveform canvas (Breathing and
Heartbeat) there is a `mat-progress-bar` labelled with the SQI value, rendered
by `qualityPercent('pqi_breath')` and `qualityPercent('pqi_heart')` in
`live.component.ts`. The label reads:

- **"SQI waiting for signal"** — not enough data yet (value is null).
- **"SQI N%"** — signal quality index as a percentage (0–100). Values
  towards 100% indicate a stable phase signal. The ribbon colour changes from
  green (good) to amber (warn) to red/hatched (rejected), driven by CSS classes
  `.sqi-ribbon span.good / .warn / .bad`.

### Overview tab — status indicators

The Overview tab's Radar Packet mini-card shows live `reported_hr` and
`reported_rr` values. The Publishing Policy card shows:

- **Payload freshness**: LIVE or STALE / WAITING. STALE means no new payload
  has arrived from the trainer recently.
- **Analysis state**: the current state of trainer-side analysis (running,
  idle, etc.).

The Mismatches / Faults card shows any active fault flags from the trainer
payload (e.g., funnel drops, schema warnings).

### HR and RR tabs — confidence and gate flags

The **HR** tab Diagnostics shows chip-level status flags:

- **Arbiter**: whether the hardware arbiter correction applied.
- **Reject phase**: whether the HR reject-phase fired (motional artefact gate).
- **Trust fresh**: whether the trusted anchor value is fresh.

The **RR** tab Diagnostics shows:

- **Anchor fresh**: whether the RR anchor is current.
- **Raw agree**: whether the zero-crossing and spectral estimates agree.
- **Recovery**: whether the fundamental-frequency recovery triggered.

When the anchor is stale or recovery triggered, the RR / Stale-State Warnings
card in the RR tab explains the cause (e.g., "RR anchor is stale; publish values
may rely on aged state.").

> **"holding" display behaviour**: the dashboard does not explicitly label a
> value as "holding" during motion. When telemetry is stale or a gate fires, the
> KPI card continues to show the last accepted value (`reported_hr` /
> `reported_rr`). The **Payload freshness** indicator in the Publishing Policy
> card changes to **STALE / WAITING** to signal that the displayed value is not
> freshly computed.

### HR confidence badge

`candidate_conf` (also read as `candidate_hr_conf`) is the per-frame HR
confidence score. It feeds the Bland-Altman PQI colouring on the scatter plot:
points with `pqi > 0.3` plot green, `0.15–0.30` plot amber, below `0.15` plot
red. This is computed in `live.component.ts` lines 198–205.

### Report — Analysis Readiness

After a session completes, go to the **Report** tab, select the session, and
check:

- **Analysis Readiness Outcome** banner: READY, CONDITIONAL, or NOT READY.
- **Historical Telemetry Trend Plot**: recorded HR and RR series.
- **Session Comparison** card: HR RMSE vs the previous and best sessions
  (drawn from trainer-computed comparison data via `/api/sessions/<id>/compare`).
- **Analysis Job** card: re-run the trainer analysis with **Run analysis** and
  watch the progress bar.

The Audit tab in Live view shows the full gate-reason histograms (HR and RR
gate blocks, AGC anomaly flags, funnel survival %) and BLE reference quality
metrics, which are also available post-session in the summary JSON.

The Report view's **Session Quality** card consolidates the recorded analysis:
PQI lock %, session quality and internal-consistency scores, locked/settling
coverage, an HR/RR accuracy table (RMSE / MAE / bias / coverage against the
reference), readiness gate chips, and the verdict categories with remediation
text for anything that did not pass. The card hides itself for older sessions
whose summaries predate these fields.

---

## 5. Troubleshooting

| Symptom | Likely cause | Resolution |
|---|---|---|
| Trainer offline / Settings shows Offline chip | Sidecar crashed or not started | Click **Start** in the Python Server card (EXE). For APK/PWA, confirm the PC trainer is running and reachable. |
| Demo mode shows instead of live data | Auto-demo-on-disconnect fired, or Demo Mode toggled on | Open Settings, turn off **Demo Mode**, click **Connect to trainer**. |
| Stale telemetry — "STALE / WAITING" in Publishing Policy | PC went to sleep, network disruption, or trainer backlog | Click **Retry connection** in the Source Mode card or close and reopen the dashboard tab. |
| Phone won't pair — "LAN pair token required" 401 | PIN expired (>5 min), wrong subnet, or firewall blocking port 8765 | Restart trainer to get a new PIN; confirm both devices are on the same Wi-Fi; allow access through Windows Defender Firewall. |
| PIN cooldown — pairing locked for 1 minute | Five consecutive wrong PIN attempts from one IP | Wait 60 seconds, then retry with the correct PIN or restart the trainer for a fresh PIN. |
| QR code not scanning | Camera permission denied or QR page expired | Grant camera permission in browser/app settings; restart trainer to refresh the QR. |
| HR/RR reads "--" | No subject detected or signal not yet locked | Confirm subject is within 40–140 cm, facing the sensor. Wait for "Subject detected" on the Home scope. |
| RR anchor stale warning | Subject moved abruptly; anchor not re-established | Ask subject to remain still for 15–30 seconds while the RR anchor re-establishes. |
