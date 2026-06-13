# Google Play — Data Safety Form Answers

**App:** Radar Vital (`app.radarvital.trainer`)  
**Prepared:** 2026-06-12  
**Mirrors:** [`PRIVACY.md`](../../PRIVACY.md) — all answers below are cross-referenced
to the section of PRIVACY.md they reflect.

> These answers are intended to be entered into the Play Console "Data safety"
> section verbatim. Keep this file in sync with PRIVACY.md whenever the privacy
> notice changes.

---

## Section 1 — Data Collection and Security

### Does your app collect or share any of the required user data types?

**Yes** — the app processes the data types listed in Section 2 below.

### Is all of the user data collected by your app encrypted in transit?

**Yes — not applicable / N/A.**  
All data stays on the local device and host PC. There is no network transmission to
developers or third parties, so "in transit" encryption of personal data to external
servers does not apply. LAN traffic between the app and the companion trainer runs
over the local network only; no data crosses the internet.

*Mirrors:* PRIVACY.md § 3 ("No cloud upload. All measurement data stays on the
trainer computer and the paired device.")

### Do you provide a way for users to request that their data is deleted?

**Yes.**

- Operator profiles can be deleted by removing the profile entry from the host
  machine (`operator_profiles.json`). An in-app profile-deletion flow is provided
  on the host-side trainer UI.
- Session recordings can be deleted from the dashboard; deleted sessions are moved
  to a trash folder under the sessions root.
- Uninstalling the app removes all locally cached data from the Android device.

*Mirrors:* PRIVACY.md § 5 ("Session recordings remain on the trainer computer until
deleted by an operator … Profiles can be removed by deleting the profile entry on the
host machine.")

---

## Section 2 — Data Types Collected

### 2.1 Personal Information

#### Name / Operator display name and initials

| Attribute | Value |
|---|---|
| **Collected?** | Yes |
| **Shared with third parties?** | No |
| **Used for app functionality?** | Yes — identifies who ran a session (station access control) |
| **Optional?** | No — required to create an operator profile |
| **Processed ephemerally (not stored)?** | No — stored locally in `operator_profiles.json` on the trainer computer |
| **End-to-end encrypted?** | No — stored on the local host; the host must be physically secured |

*Mirrors:* PRIVACY.md § 2, row "Operator display name and initials".

#### PIN / Authentication credential

| Attribute | Value |
|---|---|
| **Collected?** | Yes |
| **Shared with third parties?** | No |
| **Used for app functionality?** | Yes — station access control |
| **Optional?** | No — required for operator authentication |
| **Processed ephemerally (not stored)?** | No — stored as a salted PBKDF2-HMAC-SHA256 hash only; plaintext PIN is never stored |
| **End-to-end encrypted?** | Hashed with PBKDF2; never transmitted externally |

*Mirrors:* PRIVACY.md § 2, row "Operator PIN and recovery code"; § 7 ("PINs and
recovery codes are stored only as salted PBKDF2-HMAC-SHA256 hashes; never plaintext").

### 2.2 Health and Fitness

#### Heart rate and respiratory rate measurements (vital signs)

| Attribute | Value |
|---|---|
| **Collected?** | Yes |
| **Shared with third parties?** | No |
| **Used for app functionality?** | Yes — the core research measurement output |
| **Optional?** | No — this is the primary purpose of the app |
| **Processed ephemerally (not stored)?** | No — stored locally in session folders on the trainer computer |
| **End-to-end encrypted?** | No — stored on the local host; the host must be physically secured |

*Mirrors:* PRIVACY.md § 2, row "Session vital-sign recordings".

> **Play Data Safety note:** Select "Health and fitness → Other health and fitness
> data" for the radar telemetry CSV (HR, RR, presence, waveform data).

### 2.3 Files and Docs / App activity

#### Session metadata — subject label

| Attribute | Value |
|---|---|
| **Collected?** | Yes |
| **Shared with third parties?** | No |
| **Used for app functionality?** | Yes — distinguishes test subjects within a session; operators are advised to use codes, not full names |
| **Optional?** | Yes — the subject label field is optional |
| **Processed ephemerally (not stored)?** | No — stored in session metadata on the trainer computer |

*Mirrors:* PRIVACY.md § 2, row "Subject label entered by the operator".

#### Device diagnostics (versions, platform, connection state, log lines)

| Attribute | Value |
|---|---|
| **Collected?** | Conditionally — only when the user explicitly triggers "Report an issue" |
| **Shared with third parties?** | No — composed for user review and submitted only if the user chooses to open a GitHub issue manually |
| **Used for app functionality?** | Troubleshooting only |
| **Optional?** | Yes — can be disabled entirely in Settings |
| **Processed ephemerally (not stored)?** | Yes — composed in memory for display; not persisted by the app |

*Mirrors:* PRIVACY.md § 2, row "Device diagnostics"; § 3 ("No automatic transmission
of diagnostics. Issue reports open a GitHub page … with text you can read and edit
before anything is submitted").

---

## Section 3 — Data Sharing

### Is any user data shared with third parties?

**No.**

- No analytics SDKs (no Firebase Analytics, Crashlytics, etc.)
- No advertising SDKs (no AdMob, Meta Audience Network, etc.)
- No crash-reporting SDKs that send data automatically
- No cloud backend; no developer server receives any measurement data

*Mirrors:* PRIVACY.md § 3 ("The system has no analytics, advertising, or tracking SDKs.")

---

## Section 4 — Data Storage and Retention

### Where is user data stored?

All data is stored **locally on device** (Android app cache) and on the **trainer
computer** (the host PC running the Python companion trainer). No data is stored on
developer-controlled servers or any cloud service.

*Mirrors:* PRIVACY.md § 2 ("Where it lives" column for all rows).

### How long is data retained?

Session recordings are retained until deleted by an operator. Operator profiles are
retained until manually removed. The app does not impose automatic deletion deadlines;
research data retention follows the University of Mindanao research-ethics protocol
for this thesis.

*Mirrors:* PRIVACY.md § 5.

---

## Section 5 — Security Practices

| Practice | Status |
|---|---|
| Data encrypted in transit to external servers | N/A — no external transmission |
| Data encrypted at rest on device | Relies on Android device encryption |
| Credentials stored as hashed values | Yes — PBKDF2-HMAC-SHA256 for PINs |
| Pairing tokens in memory only | Yes — LAN tokens never persisted to disk |
| No third-party SDKs with data access | Confirmed — no analytics/ads/crash SDKs |

*Mirrors:* PRIVACY.md § 7.

---

## Section 6 — Play Console Data Safety Form — Field-by-Field Mapping

Use this table when filling in the Play Console UI:

| Play Console field | Answer | PRIVACY.md reference |
|---|---|---|
| "Does your app collect user data?" | Yes | § 2 |
| "Personal info → Name" | Collected, not shared, local only | § 2 row 1 |
| "Personal info → Other personal info" (PIN hash) | Collected, not shared, local only | § 2 row 2 |
| "Health and fitness → Other health and fitness data" (HR, RR, radar CSV) | Collected, not shared, local only | § 2 row 3 |
| "App activity → Other actions" (subject label, session metadata) | Collected, not shared, local only | § 2 rows 4–5 |
| "App info and performance → Diagnostics" | Collected conditionally on user action, not shared | § 2 row 5 |
| "Does your app share any of this data with third parties?" | No | § 3 |
| "Is all data encrypted in transit?" | N/A — no external transmission | § 3, § 7 |
| "Can users request data deletion?" | Yes — in-app deletion + uninstall | § 5 |
