# Privacy and Data

Radar Vital processes research data locally. The privacy notice is published at
`/privacy.html` for GitHub Pages and Play Console review, but the current text
is still a draft pending University of Mindanao legal / Research Ethics
Committee review. RA 10173 wording is not final counsel-verified language yet.

## Data handled

- Operator display name and initials.
- Operator PIN and recovery code hashes.
- Session vital-sign recordings: HR, RR, presence, signal quality, and radar CSV.
- Optional subject label entered by the operator.
- User-initiated diagnostics for GitHub issue reports.

## Storage

Session data and operator profiles live on the trainer computer. The app does
not include analytics, advertising, cloud sync, or automatic crash upload. The
hosted PWA is a static shell; it does not receive session CSV files unless the
operator chooses to submit text through GitHub.

## Consent and subject handling

Operators are responsible for obtaining informed consent before recording a
subject. Use subject codes rather than full names when possible. Do not record
people who have not been informed and have not consented.

## Diagnostics

The issue-report flow may assemble version, platform, connection status, alert
counts, and recent log lines. It should show a preview before opening GitHub.
Operators can disable diagnostics. Even when enabled, the report is not sent
until the operator submits it on GitHub.

## Deletion

Delete sessions from the dashboard when they should not be retained. Profiles
can be removed on the trainer host. Retention rules for thesis data must follow
the approved University of Mindanao research-ethics protocol.
