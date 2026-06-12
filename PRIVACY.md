# Radar Vital — Privacy Notice

> **DRAFT 2026-06-12.1 — pending University of Mindanao legal / Research Ethics
> Committee review. Republic Act No. 10173 (Data Privacy Act of 2012) compliance
> language has not yet been counsel-verified.**

This notice explains how Radar Vital handles personal data, in line with
**Republic Act No. 10173 — the Data Privacy Act of 2012 (DPA)**, its
Implementing Rules and Regulations, and National Privacy Commission (NPC)
guidance, and with University of Mindanao research-ethics requirements.

## 1. Who is responsible

For this thesis research, the personal information controllers are the student
researchers **Lemuel Blaya, Angelo Diaz, and Blessie Mugat** (BS Electronics
Engineering, University of Mindanao), under the supervision of the University
of Mindanao. Contact: open an issue at
https://github.com/blayalems/radar-vital-mmwave-60-davao/issues or reach the
researchers through the University.

## 2. What data the system processes

| Data | Where it lives | Why |
|---|---|---|
| Operator display name and initials | `operator_profiles.json` on the trainer computer | Identifies who ran a session |
| Operator PIN and recovery code | Stored only as salted PBKDF2 hashes (never plaintext) | Station access control |
| Session vital-sign recordings (heart rate, respiratory rate, presence, radar telemetry CSV) | Session folders on the trainer computer | The research itself |
| Subject label entered by the operator | Session metadata on the trainer computer | Distinguishing test subjects; use codes, not full names, where possible |
| Device diagnostics (versions, platform, connection state, recent log lines) | Composed **only** when you use "Report an issue", shown to you for review first | Troubleshooting; can be disabled in Settings |

## 3. What the system does NOT do

- **No cloud upload.** All measurement data stays on the trainer computer and
  the paired device. The system has no analytics, advertising, or tracking SDKs.
- **No automatic transmission of diagnostics.** Issue reports open a GitHub page
  in your browser with text you can read and edit before anything is submitted,
  and diagnostics inclusion can be switched off entirely in Settings.
- **No biometric identification.** Radar data measures vital signs; it does not
  identify faces or bodies.

## 4. Legal basis and consent

Processing is based on your **informed consent** (DPA §12(a)), collected on
first run and re-collected whenever this notice changes. Measurement subjects
must consent before a session is recorded; operators are responsible for
obtaining that consent.

## 5. Retention and deletion

Session recordings remain on the trainer computer until deleted by an operator
(sessions can be deleted from the dashboard; deleted sessions are moved to a
trash folder under the sessions root). Profiles can be removed by deleting the
profile entry on the host machine. Research data retention follows the
University of Mindanao research-ethics protocol for this thesis.

## 6. Your rights under the Data Privacy Act

Subject to the DPA, data subjects have the right to be informed; to access;
to object; to erasure or blocking; to damages; to data portability; and to file
a complaint with the **National Privacy Commission** (https://privacy.gov.ph).
To exercise these rights, contact the researchers (Section 1).

## 7. Security measures

PINs and recovery codes are stored only as salted PBKDF2-HMAC-SHA256 hashes;
LAN access requires explicit opt-in with single-use pairing PINs; network
pairing tokens live in memory only; sensitive API routes require an operator
session. Despite these measures, the trainer computer itself must be physically
secured — anyone with access to it can read session files.

## 8. Changes to this notice

The application records the version of the notice you accepted
(`2026-06-12.1`) and will ask for consent again when it changes.
