# Radar Vital

Radar Vital is a contactless heart-rate, respiratory-rate, and presence
monitoring system built around the Seeed MR60BHA2 60 GHz mmWave radar. It is a
BS Electronics Engineering thesis project by **Lemuel Blaya, Angelo Diaz, and
Blessie Mugat** at the University of Mindanao.

Radar Vital is not a medical device. It is an academic research and training
instrument. Do not use its readings for diagnosis, treatment, triage, or any
clinical decision. Use clinically validated equipment and qualified health
professionals for health decisions.

The current release line targets three operator surfaces:

- Windows EXE: bundles the Python trainer sidecar and the dashboard shell for a
  single-station PC workflow.
- Android APK/AAB: companion client for a LAN-bound trainer. It does not read
  the radar over USB by itself.
- Hosted PWA: GitHub Pages shell for demo mode, review, and LAN pairing when a
  trainer is reachable.

Legal and privacy text is still a draft pending University of Mindanao legal /
Research Ethics Committee review. RA 10173 language must not be treated as final
counsel-reviewed wording until that sign-off is recorded.

Wiki pages are sourced from `docs/wiki/` in the repository. Edit the repository
files first, then publish them to the GitHub wiki from the release process.

- [Install on Windows](Install-Windows)
- [Install on Android](Install-Android)
- [Install as PWA](Install-PWA)
- [Operator guide](Operator-Guide)
- [Pairing and LAN](Pairing-and-LAN)
- [Privacy and data](Privacy-and-Data)
- [Troubleshooting](Troubleshooting)
- [Release process](Release-Process)
