# Troubleshooting

Use this page for operator-level triage before filing an issue.

| Symptom | Likely cause | Action |
|---|---|---|
| Trainer offline | Sidecar not running, port conflict, or PC slept | Restart the trainer. If port 8765 is busy, stop the other process or choose an explicit port. |
| Phone cannot pair | Trainer is local-only, PIN expired, wrong subnet, or firewall block | Start LAN mode, generate a fresh PIN, keep both devices on the same private Wi-Fi, and allow firewall access. |
| App shows demo data | Demo mode is enabled or the trainer is unreachable | Turn off demo mode and reconnect to the trainer origin. |
| HR/RR stays blank | Subject not detected or signal not locked | Reposition the subject within the tested range and wait for stable presence. |
| Values look stale | Telemetry stream is disconnected or holding last accepted values | Check freshness indicators, SSE reconnect state, and trainer logs. Do not treat stale values as fresh readings. |
| Recovery code fails | Wrong code, already used code, or profile predates recovery codes | Use the new rotated code after reset, or perform a host reset from the trusted trainer computer. |
| Windows warning on install | Unsigned build or low SmartScreen reputation | Verify release hash and source. Signing identity work reduces warnings but cannot promise immediate reputation. |
| Play install unavailable | Closed-testing gate not met or tester not opted in | Add tester email, share opt-in URL, and wait for Play's closed-testing requirements. |

## When filing an issue

Include the product version, platform, connection mode, what you expected, what
happened, and whether the session was demo or live. If diagnostics are enabled,
review the preview and remove any subject identifiers before submitting.

## When to stop using the session

Stop and restart the session if the trainer reports schema warnings, repeated
serial faults, severe stale telemetry, or a placement state that cannot be
corrected. Do not paper over bad input data with manual notes.
