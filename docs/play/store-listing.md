# Google Play — Closed-Testing Store Listing Checklist

**App:** Radar Vital  
**Status:** Closed-testing track (internal / closed-alpha before production)  
**Prepared:** 2026-06-12  
**Contact:** blayalems@gmail.com

---

## 1. App Identity

| Field | Value |
|---|---|
| **App name** | Radar Vital |
| **Package name** | `app.radarvital.trainer` |
| **Version (initial upload)** | 16.2.0 (versionCode TBD by CI) |
| **Default language** | English (en-US) |
| **Category** | Education |
| **Tags / keywords** | mmWave, vital signs, heart rate, respiratory rate, radar, research, academic |

---

## 2. Short Description (≤ 80 characters)

```
60 GHz mmWave vital-signs monitor for academic research sessions.
```

*(79 characters — fits the Play 80-character limit.)*

---

## 3. Full Description (≤ 4 000 characters)

```
Radar Vital is a 60 GHz millimetre-wave (mmWave) vital-signs training and monitoring
application developed as a thesis project at the University of Mindanao, BS Electronics
Engineering program.

HOW IT WORKS
The Android app pairs with a Python companion trainer running on a host PC via a local
Wi-Fi connection. The trainer reads data from a Seeed Studio MR60BHA2 60 GHz radar
module (connected over USB serial), processes heart-rate and respiratory-rate waveforms
in real time, and streams them to the app over a local HTTP/SSE channel. No internet
connection is required during a session.

FEATURES
• Real-time HR and RR waveform display with signal-quality indicators
• Session recording with per-session CSV export for offline analysis
• Operator profile system with PIN-protected station access
• Adaptive placement guidance (optimal 0.4–1.5 m range for the MR60BHA2)
• Session quality scorecard and session-comparison overlay
• Offline PWA caching — dashboard loads without the trainer for review
• Demo mode: explore the full UI with simulated data, no hardware needed

INTENDED USERS
University researchers, Electronics Engineering students, and thesis advisers
participating in the University of Mindanao mmWave vital-signs study. This app is
distributed as a closed-testing APK to study participants and reviewers only.

ACADEMIC CONTEXT
This app is part of a BS Electronics Engineering thesis at the University of Mindanao,
Davao City, Philippines. It is NOT a medical device and is NOT intended for clinical
diagnosis or treatment. All measurements are for research purposes only.

NOT A MEDICAL DEVICE
Radar Vital is a research and educational tool. It has not been evaluated by any
regulatory body (FDA, DOH, or equivalent) as a medical device. Do not use it to
make clinical or diagnostic decisions.

PRIVACY
All session data (heart rate, respiratory rate, radar telemetry) is stored locally on
the host PC only. Nothing is transmitted to developers or third parties. See the
full privacy notice at:
https://blayalems.github.io/radar-vital-mmwave-60-davao/privacy.html

SOURCE CODE
https://github.com/blayalems/radar-vital-mmwave-60-davao

AUTHORS
Lemuel Blaya, Angelo Diaz, Blessie Mugat — BS Electronics Engineering, University of
Mindanao, Davao City, Philippines (2026).
```

---

## 4. Developer / Contact Details

| Field | Value |
|---|---|
| **Developer name** | Lemuel Blaya (University of Mindanao) |
| **Contact email** | blayalems@gmail.com |
| **Website** | https://github.com/blayalems/radar-vital-mmwave-60-davao |
| **Privacy Policy URL** | https://blayalems.github.io/radar-vital-mmwave-60-davao/privacy.html |

> **Note:** The privacy policy URL will be live after the GitHub Pages deployment wave.
> Upload the closed-testing APK only after the Pages build has deployed PRIVACY.md as
> `privacy.html` at the above URL. Play requires the URL to resolve at submission time.

---

## 5. Graphic Assets

### 5.1 Feature Graphic (required for any public or alpha listing)

- **Size:** 1024 × 500 px (PNG or JPG, ≤ 1 MB)
- **Design spec:**
  - Background: deep medical-teal `#0E5E63`
  - Left 40 %: app icon (adaptive icon, white radar arcs + pulse line on teal)
  - Centre/right: "Radar Vital" wordmark in white, caption "60 GHz mmWave Vital Signs"
    in a lighter teal-tinted white (`#B2DFDB`)
  - Bottom-right corner: University of Mindanao logotype or name in small type
  - No device frames or screenshots embedded (Play feature-graphic rules)
  - Safe zone: keep all text/logo within 924 × 400 px (50 px inset each side)

### 5.2 Screenshots — Phone (16:9 or 9:16, ≥ 320 px longest side)

Minimum 2, maximum 8. Capture in portrait on a Pixel-class device or emulator.

| # | Screen | What to show |
|---|---|---|
| 1 | **Home / Setup** | Trainer IP field, radar scope, placement-zone chip showing "Optimal" |
| 2 | **Live — Signal Locked** | HR/RR KPI cards with values, lock-state chip green "Signal locked", SQI ribbons |
| 3 | **Live — Demo mode** | DEMO banner visible, simulated HR/RR, phase lifecycle chips |
| 4 | **Report — Session Quality** | Quality scorecard, ML readiness verdict chips, trend charts |
| 5 | **Session Comparison** | Comparison picker open, overlay dashed line on HR chart, delta table |
| 6 | **Operator PIN Lock** | Lock overlay with PIN keypad, operator name shown |
| 7 | **Settings** | Update card, LAN share toggle, version info |
| 8 | **Help** | Help tab with topic list open |

### 5.3 Screenshots — 7" Tablet (minimum 1)

Capture at 600 dp width (Nexus 7 emulator or similar).

| # | Screen | What to show |
|---|---|---|
| T1 | **Live — landscape rail nav** | Bottom-nav replaced by side rail, HR/RR cards in two-column grid |
| T2 | **Report** | Full-width session quality scorecard |

---

## 6. Content Rating Questionnaire

Answer these in the Play Console "Content rating" section (IARC questionnaire):

| Question | Answer | Notes |
|---|---|---|
| Does your app contain user-generated content visible to other users? | **No** | Session data is local; no sharing or social features |
| Does your app allow users to communicate with each other? | **No** | No chat, forum, or messaging |
| Does your app contain or reference gambling? | **No** | |
| Does your app contain violence or graphic imagery? | **No** | |
| Does your app contain sexual content? | **No** | |
| Does your app display advertisements? | **No** | No ad SDKs |
| Is this a news app? | **No** | |
| Is this a navigation app? | **No** | |
| Does your app collect personal information from children? | **No** | Not directed at children; research use by adults |

**Expected rating:** Everyone (E) — or "Unrated" for internal testing tracks where
the IARC questionnaire has not yet been submitted. Complete the questionnaire before
promoting to production.

---

## 7. Closed-Testing Track Constraints

These are Play Console rules that apply to personal developer accounts and closed
testing — not policy choices by the team:

| Constraint | Detail |
|---|---|
| **Google Play developer registration fee** | One-time USD $25 (personal account). Required before any APK can be uploaded to Play Console. |
| **Closed-testing tester limit** | Up to **100 testers** on a closed alpha/beta track via email list. A minimum of **12 opted-in testers** is required by Google before the app can be promoted to open testing or production. |
| **Minimum closed-testing period** | **14 continuous days** of active closed testing (testers installed and used the app) before Google will approve a production promotion request for a personal account. |
| **Tester opt-in** | Each tester must accept the testing invitation via a Play opt-in URL; they cannot find the app via search. Share the closed-testing opt-in link directly. |
| **APK / AAB format** | Play now requires Android App Bundle (`.aab`) for new apps. The current CI produces a `.apk`; update `build-apk.yml` to produce an `.aab` with `bundleRelease` before submitting to Play (a follow-up wave). |
| **Signing** | Production APK/AAB must be signed with a release keystore. The current CI debug build is NOT acceptable for Play upload. A signing keystore and GitHub Actions secret must be set up (follow-up wave). |
| **Privacy policy live before submission** | The privacy policy URL must resolve at submission time. Do not upload until the GitHub Pages wave has deployed. |
| **Content rating** | IARC questionnaire must be completed even for closed testing. |
| **Data safety form** | Must be completed before any version can be published (even closed testing). See `docs/play/data-safety.md`. |

---

## 8. Pre-Submission Checklist

- [ ] Privacy policy URL is live: `https://blayalems.github.io/radar-vital-mmwave-60-davao/privacy.html`
- [ ] `$25` Play developer registration fee paid
- [ ] Release keystore generated and stored as GitHub Actions secret (`ANDROID_KEYSTORE_BASE64`, `ANDROID_KEY_ALIAS`, `ANDROID_KEY_PASSWORD`, `ANDROID_STORE_PASSWORD`)
- [ ] CI updated to produce a signed `.aab` (`bundleRelease`) rather than a debug `.apk`
- [ ] Feature graphic (1024 × 500 px) uploaded
- [ ] At least 2 phone screenshots uploaded
- [ ] At least 1 tablet screenshot uploaded
- [ ] Content rating questionnaire completed (IARC)
- [ ] Data safety form completed (see `docs/play/data-safety.md`)
- [ ] Short description ≤ 80 characters confirmed
- [ ] Full description ≤ 4 000 characters confirmed
- [ ] Closed-testing tester list populated (≥ 12 emails for eventual production promotion)
- [ ] Tester opt-in link shared with testers
- [ ] 14-day active closed-testing period started
