# Pairing and LAN

Radar Vital defaults to local-only operation. LAN access is an opt-in mode for
phone, tablet, or PWA companion clients on a trusted private network.

## Modes

- Local mode: trainer binds to `127.0.0.1`; only the host PC can reach it.
- LAN mode: trainer advertises a LAN origin and accepts paired clients that
  provide a valid operator session token.
- Native shells: Capacitor and Tauri should use their verified native or local
  transport path. Do not widen WebView CSP just to make ad hoc LAN fetches work.

## Pairing flow

1. Start or restart the trainer with LAN binding.
2. Open `/pair` on the host PC.
3. The trainer displays a QR code and a six-digit PIN.
4. On the companion client, scan the QR code or enter the trainer origin and PIN.
5. The PIN is exchanged for an `X-RVT-Auth` token.
6. Start live use only after the client shows connected status.

PINs are single-use and expire after their time-to-live. Several wrong PIN
attempts trigger a cooldown. Restart the trainer or pairing flow when a new PIN
is needed.

## Security expectations

- Use trusted private Wi-Fi only.
- Do not expose the trainer to the public internet.
- Do not add subnet scanning. Pairing is QR plus PIN by design.
- Do not serve `.rvt_tls/` or private key material from `assets/`.
- HSTS belongs only behind a trusted certificate. Self-signed TLS must not send
  `Strict-Transport-Security`.

If pairing fails, check firewall rules, subnet, trainer bind mode, the advertised
origin, and whether another process already owns port 8765.
