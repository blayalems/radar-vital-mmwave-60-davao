# 13-Agent Audit — Remediation Ledger

Tracking the wave-by-wave remediation of the consolidated 13-agent audit. Each
finding was **re-validated against the current source** before any change —
several late-agent claims were stale or contradicted by the code and are marked
`invalid/stale` rather than patched.

**Severity:** `P0 safety/security` · `P1 release blocker` · `P2 reliability` ·
`P3 maintainability` · `P4 hygiene/docs`

**Status vocabulary:** `confirmed` · `partially-confirmed` · `candidate` ·
`invalid/stale` · `blocked-by-hardware` · `deferred-design-decision` ·
`fixed` (shipped in this PR).

Baseline before this PR: `236 passed, 1 skipped`. After: `265 passed, 1 skipped`
(+29 regression tests, 0 regressions).

---

## Shipped in this PR

| ID | Wave | Finding | Agents | Sev | Status | Evidence | Fix | Tests |
| -- | ---: | ------- | ------ | --- | ------ | -------- | --- | ----- |
| R1 | 1 | `build-exe.yml` firmware path filter referenced non-existent `radar_vital_v16_2_0.ino`, so firmware edits never triggered the EXE gate | CGPT, Claude, Genspark, Arena, Jules, Grok Fast | P1 | **fixed** | `.github/workflows/build-exe.yml:15`; real file is `radar_vital_v16_3_0.ino` | Changed filter to glob `radar_vital_v16_*.ino` | `test_workflow_firmware_paths_resolve` (scans every workflow `*.ino` filter, asserts each matches a real file) |
| R2 | 1 | `_read_body()` trusted client `Content-Length` into an unbounded `rfile.read(n)` and silently degraded malformed/oversized/wrong-type bodies to `{}` for state-changing endpoints | CGPT, Gemini, Arena, Sonnet Search | P1 | **fixed** | `rvt_trainer/monolith.py` `_read_body` | Bound by `MAX_JSON_BODY_BYTES` (1 MiB → **413**); **415** on non-JSON Content-Type; **400** on malformed/non-object JSON; bodyless POST still `{}`; callers short-circuit on `None` | `test_read_body_*` (413/415/400/non-object), `test_bodyless_post_still_works`, `test_valid_json_body_accepted`, `test_public_routes_need_no_auth` |
| R3 | 4 | `_json_safe_response()` only guarded `NaN`, not `±Inf`, and used default `allow_nan=True` → could emit invalid `Infinity` tokens in API responses | Sonnet Search (Claim 3 sibling) | P2 | **fixed** | `rvt_trainer/monolith.py` `_json_safe_response` | `np.isfinite` coercion to `null` + `allow_nan=False` | `test_json_safe_response_sanitizes_nan_and_inf` |
| R4 | 2 | Operator-profile DB failed **open**: a corrupt/unreadable `operator_profiles.json` silently returned empty profiles → `is_bootstrap=True` → unauthenticated admin-create window. **Both** the main path **and** the legacy parent-dir migration branch were affected | (backend re-validation; legacy branch via PR #57 review) | P0 | **fixed** | `rvt_trainer/api/auth.py` `load_operator_profiles` (main + legacy-migration branches); bootstrap at `monolith.py` `_require_control_auth` | Distinguish absent (legit bootstrap) from present-but-broken; surface `_load_error` (`unreadable`/`bad_schema`/`legacy_unreadable`/`legacy_bad_schema`), preserve the file, log a no-PHI warning; `is_bootstrap = empty and not _load_error`. Legacy migration validates schema before migrating and fails closed on error | `test_corrupt_profiles_db_fails_closed`, `test_bad_schema_*`, `test_corrupt_legacy_profiles_db_fails_closed`, `test_bad_schema_legacy_profiles_db_fails_closed`, `test_valid_legacy_profiles_db_still_migrates`, `test_absent_profiles_db_is_clean_bootstrap`, `test_corrupt_db_blocks_lan_bootstrap_admin_create`, `test_empty_db_allows_lan_bootstrap_admin_create` |
| R5 | 2 | Operator `display_name` only `.strip()`+length-checked — accepted control chars, bidi overrides, zero-width/invisible code points (log/RTL/confusable spoofing); echoed into responses, session metadata, reports | (backend re-validation) | P2 | **fixed** | `rvt_trainer/api/auth.py` `create_operator_profile` | New `_sanitize_display_name`: NFC-normalize, reject Unicode categories `Cc/Cf/Cs/Co` | `test_display_name_rejects_unsafe_codepoints`, `test_display_name_accepts_normal_unicode`, `test_create_profile_rejects_bidi_display_name` |
| R6 | 3 | Legacy snapshot-compare modal built table rows via `innerHTML` with **unescaped** operator/data-derived labels, timestamps, KPI keys & values → stored XSS (the only unescaped sink among ~12 in the file) | Jules, G3.1 Pro Ext, Grok Fast, Sonnet Search | P1 | **fixed** | `web-legacy/modules/patches/legacy-patches.js` `buildCompareModal` (3 sinks) | Added scope-local `escHtml()` and routed `snap.label`, `snap.ts`, KPI `key`, non-numeric `val`, and `delta` through it | `test_compare_modal_sinks_are_escaped` (static); `test_legacy_escHtml_neutralizes_xss_payload` (runs the **actual** `escHtml` from source through Node and proves `<img src=x onerror=…>` → escaped text); `node --check` gate |
| R7 | 4 | Preflight firmware guidance told operators to install `radar_vital_v15_0_0.ino` / "v15.0.0 firmware" — a file that does not exist and whose 207-col schema the current 219-col contract check rejects | Jules, GF3.5 Ext, G3.1 Pro Ext, data re-validation | P2 | **fixed** | `rvt_trainer/audit/runner.py:94,102` | Point to `radar_vital_v16_3_0.ino` / the 219-column contract | `test_preflight_firmware_guidance_points_to_current_firmware` |

> **R6 note:** `web-legacy/` is served unbuilt with no JS unit harness in-repo, so
> the regression test extracts the real `escHtml()` from source and executes it via
> Node in the pytest `test` job (which already provisions Node) — proving behavior,
> not just presence. A full Playwright DOM assertion remains a Wave 9 follow-up.
>
> **R2 note (Content-Type leniency — raised in PR #57 review):** the hardened
> `_read_body()` is intentionally *lenient on a missing* `Content-Type` (it still
> rejects an explicitly non-JSON type with 415, oversized bodies with 413, and
> malformed/non-object JSON with 400). This preserves bodyless-POST clients and
> any caller that omits the header; the in-repo legacy/Angular callers and the
> pairing flow all send `application/json`. Tightening to require the header on
> every non-empty body is a low-risk follow-up once external callers are audited.

---

## Validity triage — confirmed but deferred (with rationale)

These are **confirmed** against source but intentionally **not** changed in this
PR because they need a frontend build/visual-baseline run, a product decision, or
firmware/hardware that is unavailable here. Each is a clean follow-up.

| ID | Wave | Finding | Agents | Sev | Status | Evidence | Why deferred |
| -- | ---: | ------- | ------ | --- | ------ | -------- | ------------ |
| D1 | 5 | Angular `maxChartPoints` setting is **dead** — declared & user-settable but never read to trim any array; waveform/trend `series` flow unbounded into canvases; sparkline hardcodes `slice(-20)` | GF3.5 Ext, G3.1 Pro Ext, Grok Fast, Oppie | P2 | confirmed | `web/src/app/services/stores/telemetry.store.ts:8-9`; `telemetry.service.ts:523-525`; `wave-canvas.component.ts:17`, `trend-canvas.component.ts:37` | Needs Angular build + visual-baseline refresh to verify no regression; out of scope for a Python/legacy-JS PR |
| D2 | 3 | No CSP on `web-legacy/index.html` or `web/src/index.html`; Tauri CSP uses `script-src 'self' 'unsafe-inline'` + `dangerousDisableAssetCspModification`; `withGlobalTauri: true`; capability `shell:allow-execute … "args": true` | CGPT, Claude, Genspark, G3.1 Pro Ext, Grok Fast, Oppie, Sonnet Search | P1 | confirmed | `tauri.conf.json:12,25-28`; `capabilities/default.json:14` | Adding CSP risks breaking inline scripts and the just-refreshed visual baselines; `main.rs` already constrains sidecar args to a fixed allowlist (the capability is broader than the code uses). Needs a CSP nonce/hash migration + Tauri smoke run. R6 already removes the XSS that this would have amplified. |
| D3 | 7 | Local firmware-flash path exists (`cmd_flash` → `arduino-cli upload`); the radar→trainer serial frame is plain ASCII CSV with **no per-line CRC / sequence / length** | Oppie, GF3.5 Ext, G3.1 Pro Ext, Grok Fast | P2 | partially-confirmed / blocked-by-hardware | `monolith.py` `cmd_flash`; `radar_vital_v16_3_0.ino:6969-7152`; `transport/serial.py:73-86` | Host-side CRC/seq detection requires a firmware-side change + a real device to validate; the `"DATA,"` marker + 219-col contract + `to_numeric(coerce)` give partial robustness today |
| D4 | 7/13 | `cmd_predict` does `pickle.load` of `model_hr/rr.pkl` + `preprocessor.pkl` from `--model-dir` with no path/realpath check, no signature/hash → arbitrary-code-on-load from an untrusted model dir | Claude, Jules, Grok Fast, Oppie, Sonnet Search | P1 (local-trust) | confirmed | `monolith.py` `cmd_predict` (`open(... .pkl) → pickle.load`); CLI arg `--model-dir` | Local-CLI trust boundary (not a network param). Right fix is a signed model manifest (HMAC/sha256) verified before load — a focused follow-up that also closes the "no model signing" gap |
| D5 | 8 | PWA manifest declares `categories: ["health","medical","utilities"]` while LICENSE/repo-hygiene assert "NOT A MEDICAL DEVICE" | Genspark, CGPT, Oppie, PPL Sonar | P3 | deferred-design-decision | `assets/manifest.webmanifest:13`; `tests/test_repo_hygiene_contract.py:24` | Product/regulatory call — see "Open question" below |
| D6 | 6 | Angular update comparator `isNewerVersion` discards prerelease/build metadata (`split(/[+-]/)[0]`) so `16.3.0` vs `16.3.0-rc1` compare equal | Gemini, CGPT, Claude, Genspark, Grok Fast | P2 | confirmed | `web/src/app/services/update.service.ts:281-292` | Build-number freshness is carried separately (`isNewBuild`/`release_tag`), so impact is bounded; proper SemVer precedence is a frontend follow-up with unit tests |
| D7 | 5 | Shared `operator_sessions` / `sse_tokens` maps mutated from HTTP handler threads without a lock (ThreadingHTTPServer); SSE tokens carry no `operator_id` so they survive PIN/host reset for their 30s TTL | (backend re-validation) | P2 | partially-confirmed | `monolith.py` `_require_control_auth` (`operator_sessions.pop`, `sse_tokens.pop`), `do_POST` sse-token mint; `auth.py` `_invalidate_operator_sessions` | Needs a careful shared-lock pass + binding SSE tokens to `operator_id`; moderate blast radius across several call sites — staged as its own PR to keep this one reviewable |
| D8 | 6 | `make_latest:false` + `pages.yml` manifest re-fetch tolerates transient `curl` failure → live `rvt-latest.json` can drop until next release run | Genspark, Grok Fast | P3 | partially-confirmed | `release-artifacts.yml:488-489`; `pages.yml:78-86` | Distinguish 404 (not-yet-published) from transient HTTP errors with retry; low frequency |
| D9 | 6 | No `pip-audit` / `npm audit` / SBOM / published checksums in CI; PR build artifacts retained 30 days; `web/package-lock.json` not in the npm cache key | Genspark, Grok Fast, Oppie, Sonnet Search | P3 | partially-confirmed | all `.github/workflows/*.yml` (no audit step); `npm ci` used everywhere (good) | Additive CI hardening; safe as a dedicated follow-up |
| D10 | 8 | Naive/local timestamps in human-facing report stamps (`time.strftime('%Y-%m-%d %H:%M:%S')`) and one `datetime.utcnow()` (deprecated, tz-naive) for chart annotations | PPL Sonar, data re-validation | P3 | confirmed | `monolith.py:687,1305,5296,8446,13248` (local); `:5658` (`utcnow`) | ISO `_iso_now()` is already UTC-`Z`; remaining items are display stamps — low-risk batch cleanup |

---

## Validity triage — invalid / stale (NOT patched, with proof)

Re-validation **contradicted** these claims; patching them would be churn or
regressions. Recorded so they are not re-raised.

| Claim | Agent(s) | Verdict | Proof |
| ----- | -------- | ------- | ----- |
| Missing `/api/health` endpoint | DeepSeek | **invalid/stale** | Exists at `monolith.py` and in the public bypass list (`{"/api/health","/api/version","/api/update/manifest", …}`) |
| PIN comparison may be non-constant-time | Sonnet Search | **invalid/stale** | `auth.py:verify_pin` uses `hmac.compare_digest` over PBKDF2-HMAC-SHA256 (200k iters) |
| Hardcoded `ADMIN_PIN="0000"` / `OPERATOR_PIN="1234"` | Grok Fast | **invalid/stale** | No such constants anywhere; PINs are operator-supplied and hashed at creation |
| Tokens generated with `random` not `secrets` | Sonnet Search | **invalid/stale** | Session/SSE/pair tokens use `secrets.token_urlsafe(24)`; pair PIN uses `secrets.randbelow`; recovery uses `secrets.choice`/`token_hex` |
| Session start/stop & control endpoints lack auth | DeepSeek | **invalid/stale** | Central `_require_control_auth` gates all `/api/` mutations; loopback + LAN-pairing + operator-session model verified |
| No auth rate limiting | DeepSeek | **partially-invalid** | Profile-PIN (5/30s), pair-PIN (5/60s window→60s), recovery (5/30s) lockouts + global 30/s/IP token bucket all present |
| Missing env-var fallback crashes `generate-rvt-latest.mjs` | DeepSeek | **invalid/stale** | Every `GITHUB_*` env var has an explicit default in `buildOptions()`; only missing-artifact files throw (correct) |
| Hardcoded Python interpreter path | DeepSeek | **weak/invalid** | Build scripts use list-form `subprocess.run([...])`; no hardcoded interpreter; `sys.executable` is venv-safe |
| `shell=True` command injection in build scripts | Oppie, Sonnet Search | **mostly-invalid** | No `shell=True`/`os.system`/`os.popen` in `scripts/`. One quoted-interpolation `execSync('npx esbuild "…"')` in `build-angular.mjs:98` consumes only the project's own build output (theoretical, low) → noted for `execFileSync` hardening |
| OTA firmware signing/flashing vulnerability | Oppie, Grok Fast, GF3.5 Ext | **not-applicable** | No OTA/network flash, no `esptool`/`espota`/`ArduinoOTA`/`.bin` path exists; only local USB `arduino-cli` flash (`cmd_flash`). "OTA" in `web/`/tests = PWA self-update |
| Live dashboard JSON writer is non-atomic / emits NaN | (various) | **invalid/stale** | `save_json` does tempfile→fsync→`os.replace` (atomic) with `allow_nan=False`; sections wrapped in `nan_safe` (NaN+Inf→null). (The separate API encoder gap was real → fixed as R3) |
| No CI cache keying / lockfile pinning | (various) | **mostly-invalid** | `npm ci` everywhere; `setup-node cache:npm` + `setup-python cache:pip` (keyed on requirements) + `rust-cache` (Cargo.lock). Only gap: web lockfile not in npm cache key (→ D9) |

---

## Open question for the maintainer (D5)

`assets/manifest.webmanifest` lists the `"medical"` PWA category, while the
LICENSE and `test_repo_hygiene_contract.py` assert **"NOT A MEDICAL DEVICE"**.
These are in tension. Options: (a) drop `"medical"`, keep `"health"`/`"utilities"`
to match the disclaimer; (b) keep `"medical"` deliberately and reconcile the
disclaimer wording. Left unchanged pending a product/regulatory decision.

---

## Suggested next PRs (priority order)

1. **D7** — shared-lock + operator-bound SSE tokens (auth lifecycle correctness).
2. **D4** — signed model manifest, verify before `pickle.load` (closes pickle-RCE + model-signing together).
3. **D2** — CSP + Tauri capability tightening (with nonce/hash migration & Tauri smoke).
4. **D1** — wire `maxChartPoints` into waveform/trend ingestion (with visual-baseline refresh).
5. **D6** — SemVer-correct update comparator with unit tests.
6. **D9 / D8** — CI supply-chain (`pip-audit`/`npm audit`/SBOM/checksums) + resilient manifest preservation.
7. **D3** — host-side serial CRC/sequence detection (paired with a firmware change).
8. **D10** — timezone-aware UTC cleanup of report/annotation stamps.
