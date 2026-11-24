"""Embedded fallback dashboard template retained for template-less environments."""

from __future__ import annotations

_RAW_LEGACY_DASHBOARD_TEMPLATE = r'''<!DOCTYPE html>
<html lang="en" data-theme="light" data-density="comfortable">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Radar Vital Trainer — Operator Console</title>
<meta name="description" content="Dashboard-first console for trustworthy radar vital-sign monitoring sessions. Configure hardware, run preflight checks, monitor live telemetry, and review ML-readiness verdicts.">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Inter+Tight:wght@500;600;700;800;900&family=Inter:wght@400;500;600;700;800;900&family=JetBrains+Mono:wght@400;500;700;800&display=swap" rel="stylesheet">
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Material+Symbols+Rounded:opsz,wght,FILL,GRAD@24,500,1,0" />
<script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
<style>
/* ============================================================
   INSTRUMENT DESIGN SYSTEM — Merged from sandbox prototype
   Palette: desaturated navy / quartz surface / signal cyan accent
   Typography: Inter Tight (headings) · Inter (body) · JetBrains Mono (data)
   ============================================================ */
:root{
  color-scheme:light;
  /* Surface */
  --surface-1:#ffffff; --surface-2:#f6f8fc; --surface-3:#eef2f8;
  --ink-900:#0b1220; --ink-800:#1a2540; --ink-700:#2d3a53;
  --ink-500:#5a6681; --ink-400:#7b889e; --ink-300:#9aa5b9;
  --line:#dce4ef; --line-2:#c4d0e2;
  /* Accent */
  --brand-500:#2563eb; --brand-400:#3b82f6; --brand-100:#dbeafe;
  --brand-050:#eff6ff;
  --signal-500:#06b6d4; --signal-100:#cffafe;
  /* Semantic */
  --ok-500:#0f766e; --ok-100:#ccfbf1;
  --ember-500:#c2410c; --ember-100:#ffedd5;
  --bad-500:#b91c1c; --bad-100:#fee2e2;
  /* Radii */
  --r-sm:6px; --r-md:10px; --r-lg:14px; --r-xl:20px; --r-pill:999px;
  /* Motion */
  --dur-s:120ms; --dur-m:240ms; --dur-l:400ms;
  --ease:cubic-bezier(.2,0,0,1); --ease-emph:cubic-bezier(.3,0,0,1);
  /* Shadows */
  --shadow-1:0 1px 2px rgba(16,24,40,.04),0 1px 3px rgba(16,24,40,.03);
  --shadow-2:0 1px 2px rgba(16,24,40,.06),0 10px 26px -12px rgba(16,24,40,.15);
  --shadow-3:0 2px 6px rgba(16,24,40,.08),0 26px 56px -22px rgba(16,24,40,.22);
  /* Layout */
  --gap:16px; --pad:22px;
  /* Legacy compat tokens (used by existing JS-generated HTML) */
  --md-sys-color-primary:#2563eb; --md-sys-color-on-primary:#fff;
  --md-sys-color-primary-container:#dbeafe; --md-sys-color-on-primary-container:#1e3a5f;
  --md-sys-color-secondary-container:#e8eef8; --md-sys-color-on-secondary-container:#1a2540;
  --md-sys-color-tertiary:#0f766e; --md-sys-color-tertiary-container:#ccfbf1; --md-sys-color-on-tertiary-container:#064e3b;
  --md-sys-color-error:#b91c1c; --md-sys-color-error-container:#fee2e2; --md-sys-color-on-error-container:#7f1d1d;
  --md-sys-color-warning-container:#ffedd5; --md-sys-color-on-warning-container:#7c2d12;
  --md-sys-color-success:#0f766e; --md-sys-color-success-container:#ccfbf1; --md-sys-color-on-success-container:#064e3b;
  --md-sys-color-background:#f6f8fc; --md-sys-color-on-surface:#0b1220;
  --md-sys-color-on-surface-variant:#5a6681;
  --md-sys-color-outline:#7b889e; --md-sys-color-outline-variant:#dce4ef;
  --md-sys-color-surface-container-lowest:#ffffff;
  --md-sys-color-surface-container-low:#f6f8fc;
  --md-sys-color-surface-container:#eef2f8;
  --md-sys-color-surface-container-high:#e4eaf4;
  --md-sys-color-surface-container-highest:#dae2ee;
  --shape-xs:6px; --shape-sm:10px; --shape-md:12px; --shape-lg:16px; --shape-xl:20px; --shape-full:999px;
  --ease-standard:cubic-bezier(.2,0,0,1); --ease-emph:cubic-bezier(.3,0,0,1);
  --elev-1:var(--shadow-1); --elev-2:var(--shadow-2); --elev-3:var(--shadow-3);
  --tpad:10px 4px;
  /* Merged operator-console tokens */
  --rv-bg:#f6f8fc; --rv-panel:#ffffff; --rv-panel-2:#f8faff; --rv-surface:#eef2f8; --rv-surface-hover:#e4eaf3;
  --rv-ink:#0b1220; --rv-ink-2:#1a2540; --rv-muted:#5a6681; --rv-dim:#7b889e; --rv-line:#dce4ef; --rv-line-2:#c4d0e2;
  --rv-brand:#2563eb; --rv-brand-2:#1e3a8a; --rv-signal:#06b6d4; --rv-ok:#0f766e; --rv-warn:#c2410c; --rv-bad:#b91c1c;
}
[data-theme="dark"]{
  color-scheme:dark;
  --surface-1:#0f1728; --surface-2:#141e33; --surface-3:#1a2744;
  --ink-900:#eef2ff; --ink-800:#dbe2f4; --ink-700:#c0cadf;
  --ink-500:#8895ad; --ink-400:#6b7a94; --ink-300:#556276;
  --line:#1f2d4a; --line-2:#2a3a5e;
  --brand-500:#60a5fa; --brand-400:#93c5fd; --brand-100:#1e3a5f;
  --brand-050:#172554;
  --signal-500:#22d3ee; --signal-100:#164e63;
  --ok-500:#34d399; --ok-100:#064e3b;
  --ember-500:#fb923c; --ember-100:#7c2d12;
  --bad-500:#fca5a5; --bad-100:#7f1d1d;
  /* Legacy compat */
  --md-sys-color-primary:#60a5fa; --md-sys-color-on-primary:#172554;
  --md-sys-color-primary-container:#1e3a5f; --md-sys-color-on-primary-container:#dbeafe;
  --md-sys-color-secondary-container:#1a2744; --md-sys-color-on-secondary-container:#dbe2f4;
  --md-sys-color-tertiary:#34d399; --md-sys-color-tertiary-container:#064e3b; --md-sys-color-on-tertiary-container:#ccfbf1;
  --md-sys-color-error:#fca5a5; --md-sys-color-error-container:#7f1d1d; --md-sys-color-on-error-container:#fee2e2;
  --md-sys-color-warning-container:#7c2d12; --md-sys-color-on-warning-container:#ffedd5;
  --md-sys-color-success:#34d399; --md-sys-color-success-container:#064e3b; --md-sys-color-on-success-container:#ccfbf1;
  --md-sys-color-background:#0f1728; --md-sys-color-on-surface:#eef2ff;
  --md-sys-color-on-surface-variant:#8895ad;
  --md-sys-color-outline:#6b7a94; --md-sys-color-outline-variant:#1f2d4a;
  --md-sys-color-surface-container-lowest:#0a0f1c;
  --md-sys-color-surface-container-low:#0f1728;
  --md-sys-color-surface-container:#141e33;
  --md-sys-color-surface-container-high:#1a2744;
  --md-sys-color-surface-container-highest:#223050;
  --rv-bg:#0a0f1c; --rv-panel:#0f1728; --rv-panel-2:#131c31; --rv-surface:#1a2744; --rv-surface-hover:#1d2947;
  --rv-ink:#eef2ff; --rv-ink-2:#dbe2f4; --rv-muted:#8895ad; --rv-dim:#6b7a94; --rv-line:#1f2d4a; --rv-line-2:#2a3a5e;
  --rv-brand:#60a5fa; --rv-brand-2:#93c5fd; --rv-signal:#22d3ee; --rv-ok:#34d399; --rv-warn:#fb923c; --rv-bad:#fca5a5;
}
[data-density="compact"]{ --gap:12px; --pad:16px; --tpad:7px 4px; }
*{box-sizing:border-box; margin:0; padding:0}
html{scroll-behavior:smooth}
body{
  min-height:100vh;
  font-family:'Inter','Inter Tight',system-ui,-apple-system,sans-serif;
  font-optical-sizing:auto;
  background:
    linear-gradient(90deg,color-mix(in oklab,var(--line) 36%,transparent) 1px,transparent 1px) 0 0/32px 32px,
    linear-gradient(0deg,color-mix(in oklab,var(--line) 36%,transparent) 1px,transparent 1px) 0 0/32px 32px,
    radial-gradient(1100px 620px at 12% -12%,color-mix(in oklab,var(--brand-500) 12%,transparent),transparent 58%),
    radial-gradient(900px 500px at 100% 0%,color-mix(in oklab,var(--signal-500) 12%,transparent),transparent 56%),
    var(--surface-2);
  color:var(--ink-800);
  transition: background var(--dur-m) var(--ease), color var(--dur-m) var(--ease);
}
code,.mono{font-family:'JetBrains Mono',monospace}

/* ---- APP SHELL ---- */
.app{display:grid;grid-template-columns:240px minmax(0,1fr);min-height:100vh}
.rail{
  background:linear-gradient(180deg,var(--surface-2),var(--surface-1));
  border-right:1px solid var(--line);
  padding:18px 14px 14px;display:flex;flex-direction:column;gap:18px;
  position:sticky;top:0;height:100vh;overflow-y:auto;
  box-shadow:6px 0 24px rgba(16,24,40,.04);
}
.rail::-webkit-scrollbar{width:0}
.brand{
  display:flex;align-items:center;gap:12px;
  padding:8px 10px;
  border-radius:var(--r-md);
  background:transparent;
  color:var(--ink-900);margin:0;position:relative;overflow:hidden;
  box-shadow:none;
}
.brand .material-symbols-rounded{font-size:28px;position:relative;z-index:1}
.material-symbols-rounded{font-family:'Material Symbols Rounded';font-weight:normal;font-style:normal;display:inline-block;line-height:1;text-transform:none;letter-spacing:normal;word-wrap:normal;white-space:nowrap;direction:ltr;-webkit-font-feature-settings:'liga';-webkit-font-smoothing:antialiased;font-feature-settings:'liga'}
.brand:hover{background:var(--surface-3)}
.brand::after{display:none}
@keyframes spin{to{transform:rotate(360deg)}}
.brand-mark{
  position:relative;width:40px;height:40px;flex:none;z-index:1;
  border-radius:12px;
  background:
    radial-gradient(circle at 30% 30%,#fff,transparent 40%),
    linear-gradient(135deg,var(--brand-500),var(--brand-400) 70%,#0a1630);
  box-shadow:0 0 0 1px rgba(255,255,255,.25) inset,0 6px 14px -4px rgba(29,78,216,.45);
  overflow:hidden;display:grid;place-items:center;
}
.brand-ring{
  position:absolute;inset:6px;
  border:1.5px solid rgba(255,255,255,.55);
  border-radius:50%;
  animation:sweep 4s linear infinite;
}
.brand-ring.r2{inset:10px;opacity:.45;animation-duration:6s;animation-direction:reverse}
.brand-dot{
  width:6px;height:6px;border-radius:50%;
  background:#fff;
  box-shadow:0 0 10px #fff,0 0 0 3px rgba(255,255,255,.25);
  position:relative;z-index:2;
}
@keyframes sweep{
  0%{clip-path:polygon(50% 50%,100% 0,100% 50%);transform:rotate(0)}
  100%{clip-path:polygon(50% 50%,100% 0,100% 50%);transform:rotate(360deg)}
}
.brand-label{display:flex;flex-direction:column;line-height:1.1}
.bl-top{font-family:'Inter Tight','Inter',sans-serif;font-weight:900;font-size:14px;letter-spacing:.14em;color:var(--ink-900)}
.bl-bot{font-size:10.5px;letter-spacing:.14em;color:var(--ink-400);font-weight:700;margin-top:2px}
.nav-groups{display:flex;flex-direction:column;gap:6px}
.nav-group{display:flex;flex-direction:column;gap:2px;padding-top:10px}
.nav-group + .nav-group{border-top:1px dashed var(--line);margin-top:6px}
.nav-heading{
  font-size:10.5px;font-weight:800;letter-spacing:.14em;
  color:var(--ink-400);text-transform:uppercase;
  padding:8px 10px 6px;
}
.r-item{
  appearance:none;display:grid;grid-template-columns:22px minmax(0,1fr) auto;
  align-items:center;gap:12px;padding:9px 10px;
  border-radius:var(--r-md);background:transparent;border:1px solid transparent;
  color:var(--ink-700);font:600 14px/1.15 'Inter',sans-serif;
  letter-spacing:0;cursor:pointer;position:relative;text-align:left;
  transition:background var(--dur-s) var(--ease),color var(--dur-s) var(--ease),border-color var(--dur-s) var(--ease);
}
.r-item .r-ic{width:22px;height:22px;display:grid;place-items:center;border-radius:var(--r-sm);transition:background var(--dur-m) var(--ease-emph)}
.r-item .material-symbols-rounded{font-size:20px;color:var(--ink-500);transition:font-variation-settings var(--dur-s) var(--ease),color var(--dur-s) var(--ease)}
.r-item:hover{background:var(--surface-3);color:var(--ink-900)}
.r-item:hover .material-symbols-rounded{color:var(--ink-800)}
.r-item.active{
  background:color-mix(in oklab,var(--brand-100) 60%,var(--surface-1));
  color:var(--ink-900);border-color:color-mix(in oklab,var(--brand-500) 25%,transparent);
  font-weight:700;
}
.r-item.active::before{content:'';position:absolute;left:-14px;top:8px;bottom:8px;width:3px;border-radius:0 3px 3px 0;background:var(--brand-500)}
.r-item.active .r-ic{background:transparent}
.r-item.active .material-symbols-rounded{color:var(--brand-500);font-variation-settings:'FILL' 1,'wght' 600}
.r-item.sub{padding:7px 10px 7px 14px;font-size:13px;color:var(--ink-500)}
.r-item.sub .material-symbols-rounded{font-size:18px}
.r-item.sub.active{background:var(--surface-3);color:var(--ink-900)}
.r-item.sub.active::before{background:var(--signal-500)}
.nav-ind{
  width:8px;height:8px;border-radius:50%;
  background:var(--signal-500);
  box-shadow:0 0 0 0 color-mix(in oklab,var(--signal-500) 60%,transparent);
  animation:pulseDot 1.6s var(--ease) infinite;
  justify-self:end;margin-right:6px;
}
@keyframes pulseDot{
  0%{box-shadow:0 0 0 0 color-mix(in oklab,var(--signal-500) 60%,transparent)}
  70%{box-shadow:0 0 0 8px color-mix(in oklab,var(--signal-500) 0%,transparent)}
  100%{box-shadow:0 0 0 0 transparent}
}
body:not([data-view="live"]) #liveTabsGroup{display:none}
.nav-kbd{
  justify-self:end;
  font-family:'JetBrains Mono',monospace;font-size:10.5px;font-weight:700;
  color:var(--ink-400);
  background:var(--surface-3);border:1px solid var(--line);
  padding:2px 6px;border-radius:6px;
}
.r-item.active .nav-kbd{background:#fff}
.r-spacer{flex:1}
.rail-foot{margin-top:auto;display:flex;flex-direction:column;gap:10px;padding-top:12px;border-top:1px dashed var(--line)}
.operator{
  display:flex;align-items:center;gap:10px;
  padding:8px;
  background:var(--surface-3);border-radius:var(--r-md);
}
.op-avatar{
  width:32px;height:32px;border-radius:10px;
  display:grid;place-items:center;
  background:linear-gradient(135deg,var(--brand-500),var(--signal-500));
  color:#fff;font-weight:800;font-size:12px;
  letter-spacing:.05em;
}
.op-meta{display:flex;flex-direction:column;line-height:1.2;min-width:0}
.op-name{font-size:13px;font-weight:700;color:var(--ink-900);white-space:nowrap;overflow:hidden;text-overflow:ellipsis;max-width:150px}
.op-role{font-size:11px;color:var(--ink-400);white-space:nowrap;overflow:hidden;text-overflow:ellipsis;max-width:150px}

/* ---- MAIN ---- */
.main{padding:24px 30px 88px;min-width:0;max-width:1720px;width:100%}
.topbar{
  position:sticky;top:0;z-index:60;
  display:flex;align-items:center;gap:14px;padding:12px 4px 16px;flex-wrap:wrap;
  backdrop-filter:blur(16px);
  background:color-mix(in oklab,var(--surface-2) 88%,transparent);
  border-bottom:1px solid color-mix(in oklab,var(--line) 60%,transparent);
}
.eyebrow{font-size:11px;font-weight:800;letter-spacing:.14em;text-transform:uppercase;color:var(--brand-500);margin-bottom:4px}
.topbar h1{margin:0;font-family:'Inter Tight','Inter',sans-serif;font-size:36px;font-weight:900;letter-spacing:-.02em;line-height:1;color:var(--ink-900)}
.sp{flex:1}
.conn{display:inline-flex;align-items:center;gap:8px;padding:8px 14px 8px 12px;border-radius:var(--r-md);background:var(--surface-1);border:1px solid var(--line);font-size:13px;font-weight:700;color:var(--ink-500)}
.dot{width:10px;height:10px;border-radius:50%;background:var(--ok-500);box-shadow:0 0 0 0 color-mix(in oklab,var(--ok-500) 50%,transparent);animation:pulse 1.8s var(--ease) infinite}
.conn.warn .dot{background:var(--ember-500);animation:none}
.conn.bad .dot{background:var(--bad-500);animation:none}
@keyframes pulse{0%{box-shadow:0 0 0 0 color-mix(in oklab,var(--ok-500) 70%,transparent)}70%{box-shadow:0 0 0 10px transparent}100%{box-shadow:0 0 0 0 transparent}}
.ic-btn{width:42px;height:42px;border-radius:var(--r-md);background:var(--surface-1);border:1px solid var(--line);display:grid;place-items:center;color:var(--ink-700);cursor:pointer;position:relative;transition:background var(--dur-s) var(--ease),transform var(--dur-s) var(--ease-emph)}
.ic-btn:hover{background:var(--surface-3);border-color:var(--line-2)}
.ic-btn:active{transform:scale(.93)}
.ic-btn .material-symbols-rounded{font-size:20px}
.ic-badge{position:absolute;top:-4px;right:-4px;min-width:18px;height:18px;padding:0 5px;border-radius:var(--r-pill);background:var(--bad-500);color:#fff;font-size:11px;font-weight:900;display:grid;place-items:center;border:2px solid var(--surface-2)}
.fab{display:inline-flex;align-items:center;gap:8px;padding:12px 20px 12px 16px;border-radius:var(--r-md);background:linear-gradient(135deg,var(--brand-500),color-mix(in oklab,var(--brand-500) 72%,var(--signal-500)));color:#fff;font:700 14px/1 inherit;border:none;cursor:pointer;box-shadow:0 10px 24px -12px var(--brand-500);transition:box-shadow var(--dur-s) var(--ease),transform var(--dur-s) var(--ease-emph)}
.fab:hover{box-shadow:0 14px 28px -12px var(--brand-500);transform:translateY(-1px)}
.fab:active{transform:scale(.97)}
.fab .material-symbols-rounded{font-size:20px}
.seg{display:inline-flex;border:1px solid var(--line);border-radius:var(--r-md);overflow:hidden}
.seg button{appearance:none;background:transparent;border:none;padding:8px 14px;font:600 13px/1 inherit;color:var(--ink-500);cursor:pointer;display:inline-flex;align-items:center;gap:6px;transition:background var(--dur-s) var(--ease),color var(--dur-s) var(--ease)}
.seg button+button{border-left:1px solid var(--line)}
.seg button:hover{background:var(--surface-3)}
.seg button.active{background:var(--brand-100);color:var(--brand-500)}
.seg button .material-symbols-rounded{font-size:18px}

/* ---- CHIPS / PILLS ---- */
.chips{display:flex;flex-wrap:wrap;gap:8px;margin-top:4px}
.chip{display:inline-flex;align-items:center;gap:6px;padding:6px 12px 6px 8px;border-radius:var(--r-md);font-size:12px;font-weight:700;background:var(--surface-3);color:var(--ink-500);border:1px solid transparent;transition:all var(--dur-s) var(--ease)}
.chip .material-symbols-rounded{font-size:16px}
.chip.good{background:var(--ok-100);color:var(--ok-500)}
.chip.warn{background:var(--ember-100);color:var(--ember-500)}
.chip.bad{background:var(--bad-100);color:var(--bad-500)}
.pills{display:flex;flex-wrap:wrap;gap:6px}
.pill{display:inline-flex;align-items:center;gap:5px;padding:4px 10px;border-radius:var(--r-pill);background:var(--surface-3);color:var(--ink-500);font-size:11.5px;font-weight:700;border:1px solid var(--line)}
.pill.good{background:var(--ok-100);color:var(--ok-500);border-color:transparent}
.pill.warn{background:var(--ember-100);color:var(--ember-500);border-color:transparent}
.pill.bad{background:var(--bad-100);color:var(--bad-500);border-color:transparent}
.pill .material-symbols-rounded{font-size:14px}

/* ---- FIRMWARE BADGE ---- */
.fw{display:grid;grid-template-columns:auto minmax(0,1fr) auto;align-items:center;gap:14px;padding:14px 18px;border-radius:var(--r-lg);background:var(--surface-3);margin-bottom:var(--gap);border:1px solid var(--line);box-shadow:var(--shadow-1)}
.fw.good{background:color-mix(in oklab,var(--ok-100) 50%,var(--surface-1));color:var(--ok-500);border-color:color-mix(in oklab,var(--ok-500) 18%,transparent)}
.fw.warn{background:color-mix(in oklab,var(--ember-100) 50%,var(--surface-1));color:var(--ember-500);border-color:color-mix(in oklab,var(--ember-500) 20%,transparent)}
.fw.bad{background:color-mix(in oklab,var(--bad-100) 50%,var(--surface-1));color:var(--bad-500);border-color:color-mix(in oklab,var(--bad-500) 18%,transparent)}
.fw .material-symbols-rounded{font-size:24px}
.fw-title{font-size:11px;font-weight:900;letter-spacing:.1em;text-transform:uppercase;opacity:.85}
.fw-line{display:flex;flex-wrap:wrap;gap:10px;margin-top:4px;font-family:'JetBrains Mono',monospace;font-size:12.5px;font-weight:600}
.fw-line span{padding:2px 8px;background:rgba(0,0,0,.06);border-radius:var(--r-sm)}
[data-theme="dark"] .fw-line span{background:rgba(255,255,255,.08)}

/* ---- TABS / LAYOUT ---- */
.tab{display:none;animation:fadeUp var(--dur-m) var(--ease-emph) forwards}
.tab.active{display:block}
@keyframes fadeUp{from{opacity:0;transform:translateY(8px)}to{opacity:1;transform:none}}
.grid{display:grid;gap:var(--gap)}
.c2{grid-template-columns:repeat(2,minmax(0,1fr))}
.c3{grid-template-columns:repeat(3,minmax(0,1fr))}
.c4{grid-template-columns:repeat(4,minmax(0,1fr))}

/* ---- CARD ---- */
.card{background:color-mix(in oklab,var(--surface-1) 96%,transparent);border-radius:var(--r-xl);padding:var(--pad);min-width:0;border:1px solid var(--line);position:relative;box-shadow:var(--shadow-2);transition:border-color var(--dur-s) var(--ease)}
.card:hover{border-color:color-mix(in oklab,var(--brand-500) 25%,var(--line))}
.ch{display:flex;align-items:flex-start;justify-content:space-between;gap:12px;margin-bottom:18px}
.tw{display:flex;align-items:center;gap:12px;min-width:0}
.tw>.material-symbols-rounded{color:#fff;background:linear-gradient(135deg,var(--brand-500),var(--brand-400));padding:9px;border-radius:var(--r-md);font-size:20px;flex:0 0 auto}
.tw.rr>.material-symbols-rounded{background:linear-gradient(135deg,var(--ok-500),#34d399)}
.ct{margin:0;font-family:'Inter Tight','Inter',sans-serif;font-size:17px;font-weight:800;letter-spacing:-.01em;color:var(--ink-900)}
.cs{margin-top:2px;font-size:12.5px;color:var(--ink-500);font-weight:500}
.ca{display:flex;gap:4px;align-items:center;flex-wrap:wrap}
.ca-btn{appearance:none;border:none;background:transparent;width:34px;height:34px;border-radius:var(--r-md);color:var(--ink-500);display:grid;place-items:center;cursor:pointer;transition:background var(--dur-s) var(--ease)}
.ca-btn:hover{background:var(--surface-3)}
.ca-btn .material-symbols-rounded{font-size:18px}

/* ---- KPI ---- */
.kpi-grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:var(--gap)}
.kpi{background:var(--surface-1);border:1px solid var(--line);border-radius:var(--r-xl);padding:20px;display:grid;gap:10px;position:relative;overflow:hidden;min-width:0;box-shadow:var(--shadow-2);transition:border-color var(--dur-s) var(--ease),transform var(--dur-s) var(--ease)}
.kpi:hover{border-color:var(--line-2);transform:translateY(-1px)}
.kpi::before{content:'';position:absolute;inset:auto -30% -60% auto;width:200px;height:200px;background:radial-gradient(circle,color-mix(in oklab,var(--brand-500) 14%,transparent),transparent 60%);pointer-events:none}
.kpi.hr{border-top:3px solid var(--brand-500)}
.kpi.hr::before{background:radial-gradient(circle,color-mix(in oklab,var(--brand-500) 22%,transparent),transparent 60%)}
.kpi.rr{border-top:3px solid var(--ok-500)}
.kpi.rr::before{background:radial-gradient(circle,color-mix(in oklab,var(--ok-500) 22%,transparent),transparent 60%)}
.k-lab{display:flex;align-items:center;justify-content:space-between;gap:8px}
.k-lab-t{font-size:11px;font-weight:800;letter-spacing:.12em;text-transform:uppercase;color:var(--ink-400)}
.k-ic{width:32px;height:32px;border-radius:var(--r-md);display:grid;place-items:center;background:var(--surface-3)}
.k-ic .material-symbols-rounded{font-size:18px;color:var(--ink-500)}
.k-val-r{display:flex;align-items:baseline;gap:6px}
.k-val{font-family:'Inter Tight','Inter',sans-serif;font-size:48px;font-weight:900;letter-spacing:-.035em;line-height:1;color:var(--ink-900)}
.k-u{font-size:14px;font-weight:700;color:var(--ink-400)}
.k-spark{height:36px;width:100%}
.k-sub{font-size:12px;font-weight:600;color:var(--ink-400)}

/* ---- METRICS / TABLES ---- */
.mr{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px;margin-bottom:16px}
.mr.three{grid-template-columns:repeat(3,minmax(0,1fr))}
.m{display:flex;flex-direction:column;gap:4px;background:var(--surface-3);padding:14px;border-radius:var(--r-md)}
.m-l{font-size:11px;font-weight:800;color:var(--ink-400);text-transform:uppercase;letter-spacing:.1em}
.m-vr{display:flex;align-items:baseline;gap:4px}
.m-v{font-family:'JetBrains Mono',monospace;font-size:28px;font-weight:900;color:var(--brand-500);letter-spacing:-.02em;line-height:1}
.m.rr .m-v{color:var(--ok-500)}
.m-u{font-size:13px;color:var(--ink-400);font-weight:700}
table{width:100%;border-collapse:collapse}
td{padding:var(--tpad);border-bottom:1px solid var(--line);font-size:12.5px;vertical-align:middle}
tr:last-child td{border-bottom:none}
td:first-child{color:var(--ink-500);font-weight:500}
td:last-child{color:var(--ink-900);text-align:right;font-weight:600;font-family:'JetBrains Mono',monospace;cursor:copy;position:relative}
td:last-child:hover{color:var(--brand-500)}
td.copied::after{content:'copied';position:absolute;right:6px;top:50%;transform:translateY(-50%);background:var(--ink-900);color:#fff;padding:2px 6px;border-radius:var(--r-sm);font-size:10px;font-family:inherit;animation:tOut 1.4s forwards}
@keyframes tOut{0%,70%{opacity:1}100%{opacity:0}}

/* ---- FAULTS ---- */
.faults{display:grid;gap:10px}
.flt{border-radius:var(--r-lg);padding:14px 16px;display:grid;gap:6px;position:relative;overflow:hidden;border-left:4px solid var(--line-2)}
.flt.good{background:color-mix(in oklab,var(--ok-100) 45%,var(--surface-1));color:var(--ok-500);border-left-color:var(--ok-500)}
.flt.warn{background:color-mix(in oklab,var(--ember-100) 45%,var(--surface-1));color:var(--ember-500);border-left-color:var(--ember-500)}
.flt.bad{background:color-mix(in oklab,var(--bad-100) 45%,var(--surface-1));color:var(--bad-500);border-left-color:var(--bad-500)}
.flt-t{display:flex;align-items:center;gap:8px;font-weight:800;font-size:13.5px}
.flt-t .material-symbols-rounded{font-size:18px}
.flt-c{font-size:12.5px;opacity:.9;line-height:1.45;font-weight:500}

/* ---- CHARTS ---- */
.cp{background:var(--surface-3);border-radius:var(--r-lg);padding:14px;min-height:260px}
.cp.s{min-height:220px}
canvas{width:100%!important;height:230px!important}
#rawHrBucketChart{height:200px!important}
pre{background:var(--surface-3);color:var(--ink-800);border:1px solid var(--line);border-radius:var(--r-lg);padding:14px;white-space:pre-wrap;font-family:'JetBrains Mono',monospace;font-size:12px;line-height:1.55;margin:0;overflow-x:auto;max-height:360px;overflow-y:auto}

/* ---- RADAR ---- */
.rad{display:grid;grid-template-columns:minmax(0,1.3fr) minmax(320px,.9fr);gap:var(--gap)}
.rp{background:var(--surface-3);border-radius:var(--r-lg);padding:18px;display:grid;gap:14px}
.rp-h{display:flex;align-items:flex-start;justify-content:space-between;gap:12px}
.rp-n{font-size:13px;line-height:1.5;color:var(--ink-500);max-width:60ch}
.tgt{flex:0 0 auto;padding:8px 14px;border-radius:var(--r-pill);background:var(--brand-100);color:var(--brand-500);font-weight:800;font-size:13px}
.rad-w{position:relative;background:radial-gradient(120% 120% at 50% 120%,#0b1220,#111a2c 50%,#0a0f1c 80%);border:1px solid var(--line);border-radius:var(--r-lg);padding:12px;min-height:340px;overflow:hidden}
.rad-w::before{content:'';position:absolute;inset:0;background:radial-gradient(500px 250px at 50% 100%,color-mix(in oklab,var(--signal-500) 10%,transparent),transparent 60%);animation:sheen 5s ease-in-out infinite alternate;pointer-events:none}
@keyframes sheen{from{opacity:.5}to{opacity:1}}
#radarCanvas{height:320px!important}
.side{display:grid;gap:var(--gap);min-width:0}
.tb{background:var(--surface-3);border-radius:var(--r-lg);padding:18px}
.tbig{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:10px;margin-bottom:18px}
.tl{background:var(--surface-1);border-radius:var(--r-md);padding:14px;display:grid;gap:4px;border:1px solid var(--line)}
.tl .l{font-size:11px;font-weight:800;text-transform:uppercase;letter-spacing:.08em;color:var(--ink-400)}
.tl .v{font-family:'JetBrains Mono',monospace;font-size:26px;line-height:1;font-weight:900;color:var(--brand-500);letter-spacing:-.02em}
.tl .v.rr{color:var(--ok-500)}
.kv{display:grid;grid-template-columns:minmax(0,1fr) auto;gap:8px 16px;align-items:center}
.kv>div:nth-child(odd){color:var(--ink-500);font-size:12.5px;font-weight:500}
.kv>div:nth-child(even){text-align:right;font-family:'JetBrains Mono',monospace;font-size:12.5px;font-weight:600;color:var(--ink-900);word-break:break-word}

/* ---- WAVES ---- */
.wg{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:var(--gap)}
.wc{background:var(--surface-3);border-radius:var(--r-lg);padding:18px}
.wt{margin:0;font-family:'Inter Tight','Inter',sans-serif;font-size:15px;font-weight:800}
.ws{margin:4px 0 12px;font-size:12px;color:var(--ink-500);font-weight:500}
#breathChart,#heartChart{height:180px!important}
.ft{display:flex;align-items:center;gap:8px;color:var(--ink-400);font-size:12px;font-family:'JetBrains Mono',monospace;padding:16px 4px 0;font-weight:500}
.ft .material-symbols-rounded{font-size:16px}

/* ---- DRAWERS / OVERLAYS ---- */
.dv-ov{position:fixed;inset:0;background:rgba(10,15,28,.4);backdrop-filter:blur(4px);opacity:0;pointer-events:none;transition:opacity var(--dur-m) var(--ease);z-index:90}
.dv-ov.open{opacity:1;pointer-events:auto}
.dv{position:fixed;right:0;top:0;bottom:0;width:min(420px,100vw);background:var(--surface-1);border-left:1px solid var(--line);transform:translateX(100%);transition:transform var(--dur-l) var(--ease-emph);z-index:100;display:flex;flex-direction:column;box-shadow:var(--shadow-3)}
.dv.open{transform:translateX(0)}
.dv-h{padding:20px 24px;display:flex;align-items:center;justify-content:space-between;border-bottom:1px solid var(--line)}
.dv-h h3{margin:0;font-family:'Inter Tight','Inter',sans-serif;font-size:18px;font-weight:800}
.dv-b{flex:1;overflow-y:auto;padding:16px 20px;display:flex;flex-direction:column;gap:10px}
.dv-e{margin:40px auto;text-align:center;color:var(--ink-500);font-size:14px;font-weight:600}
.dv-e .material-symbols-rounded{display:block;font-size:48px;margin-bottom:8px;color:var(--ok-500);font-variation-settings:'FILL' 1}

/* ---- SNAPSHOTS ---- */
.snaps{display:grid;gap:10px}
.snap{display:grid;grid-template-columns:auto 1fr auto;gap:12px;align-items:center;padding:12px 14px;background:var(--surface-3);border-radius:var(--r-md)}
.snap-t{font-family:'JetBrains Mono',monospace;font-size:12px;color:var(--ink-500);font-weight:600}
.snap-v{display:flex;gap:14px;font-family:'JetBrains Mono',monospace;font-size:13px;font-weight:700;flex-wrap:wrap}
.snap-v .hr{color:var(--brand-500)}
.snap-v .rr{color:var(--ok-500)}
.snap-v .n{color:var(--ink-400)}
.snap-e{text-align:center;padding:20px;color:var(--ink-500);font-size:13px}

/* ---- PALETTE ---- */
.p-ov{position:fixed;inset:0;background:rgba(10,15,28,.52);backdrop-filter:blur(6px);display:none;z-index:200;align-items:flex-start;justify-content:center;padding-top:13vh}
.p-ov.open{display:flex}
.p{width:min(600px,94vw);background:var(--surface-1);border:1px solid var(--line);border-radius:var(--r-xl);box-shadow:var(--shadow-3);overflow:hidden;animation:pIn var(--dur-m) var(--ease-emph)}
@keyframes pIn{from{opacity:0;transform:translateY(-10px) scale(.98)}to{opacity:1;transform:none}}
.p-iw{display:flex;align-items:center;gap:10px;padding:16px 20px;border-bottom:1px solid var(--line)}
.p-iw .material-symbols-rounded{color:var(--ink-400);font-size:22px}
.p-in{flex:1;background:transparent;border:none;outline:none;font:inherit;font-size:16px;color:var(--ink-900)}
.p-in::placeholder{color:var(--ink-400)}
.p-l{max-height:50vh;overflow-y:auto;padding:8px}
.p-i{display:flex;align-items:center;gap:12px;padding:11px 14px;border-radius:var(--r-md);cursor:pointer;color:var(--ink-700);font-size:14px;font-weight:600}
.p-i .material-symbols-rounded{color:var(--ink-400);font-size:20px}
.p-i .cmd-sub{margin-left:auto;color:var(--ink-400);font-size:12px;font-weight:500}
.p-i.sel,.p-i:hover{background:var(--brand-100);color:var(--ink-900)}
.p-i.sel .material-symbols-rounded{color:var(--brand-500)}
.p-hint{padding:10px 18px;border-top:1px solid var(--line);font-size:11px;font-family:'JetBrains Mono',monospace;color:var(--ink-400);display:flex;gap:14px}
kbd{background:var(--surface-3);padding:2px 7px;border-radius:var(--r-sm);border:1px solid var(--line);font-family:'JetBrains Mono',monospace;font-size:11px;font-weight:700;color:var(--ink-500)}

/* ---- TOAST ---- */
.toast-host{position:fixed;left:50%;transform:translateX(-50%);bottom:24px;display:flex;flex-direction:column;gap:8px;z-index:300;pointer-events:none}
.toast{padding:12px 18px;background:var(--ink-900);color:#fff;border-radius:var(--r-pill);box-shadow:var(--shadow-3);font-size:13px;font-weight:600;display:flex;align-items:center;gap:8px;animation:tPop var(--dur-m) var(--ease-emph),tFade .4s var(--ease) forwards;animation-delay:0s,3s}
.toast .material-symbols-rounded{font-size:18px;color:var(--signal-500)}
@keyframes tPop{from{opacity:0;transform:translateY(10px)}to{opacity:1;transform:none}}
@keyframes tFade{to{opacity:0;transform:translateY(10px)}}

/* ---- FULLSCREEN ---- */
body.fs-open .card{overflow:visible}
.card.fs{position:fixed;inset:40px;z-index:150;background:var(--surface-1);overflow:auto}
.card.fs .cp{min-height:calc(100vh - 200px)}
.card.fs canvas{height:calc(100vh - 240px)!important}

/* ---- SETTINGS MODAL ---- */
.set-ov{position:fixed;inset:0;background:rgba(10,15,28,.46);backdrop-filter:blur(4px);display:none;z-index:220;align-items:center;justify-content:center;padding:20px}
.set-ov.open{display:flex}
.set-md{width:min(600px,96vw);max-height:min(82vh,760px);overflow:auto;background:var(--surface-1);border:1px solid var(--line);border-radius:var(--r-xl);box-shadow:var(--shadow-3);padding:22px}
.set-h{display:flex;align-items:flex-start;justify-content:space-between;gap:12px;margin-bottom:16px}
.set-tt{margin:0;font-family:'Inter Tight','Inter',sans-serif;font-size:20px;font-weight:900}
.set-st{margin-top:4px;color:var(--ink-500);font-size:13px;font-weight:500}
.set-g{background:var(--surface-3);border:1px solid var(--line);border-radius:var(--r-lg);padding:18px;display:grid;gap:12px}
.set-g + .set-g{margin-top:14px}
.set-gl{margin:0;font-size:11px;font-weight:900;letter-spacing:.12em;text-transform:uppercase;color:var(--ink-400)}
.set-r{display:grid;grid-template-columns:minmax(0,1fr) auto;gap:16px;align-items:center;padding:10px 0;border-bottom:1px solid var(--line)}
.set-r:last-child{border-bottom:none;padding-bottom:0}
.set-copy{display:grid;gap:4px}
.set-copy strong{font-size:14px;font-weight:800;color:var(--ink-900)}
.set-copy span{font-size:12.5px;line-height:1.45;color:var(--ink-500);font-weight:500}
.sw{appearance:none;width:54px;height:30px;border:none;border-radius:var(--r-pill);background:var(--line-2);cursor:pointer;position:relative;transition:background var(--dur-m) var(--ease)}
.sw::after{content:'';position:absolute;left:3px;top:3px;width:24px;height:24px;border-radius:50%;background:#fff;box-shadow:var(--shadow-1);transition:transform var(--dur-m) var(--ease-emph)}
.sw.on{background:var(--brand-500)}
.sw.on::after{transform:translateX(24px)}
.set-actions{display:flex;justify-content:flex-end;gap:10px;margin-top:16px}
.txt-btn{appearance:none;border:none;background:var(--brand-100);color:var(--brand-500);padding:10px 16px;border-radius:var(--r-md);font:700 14px/1 inherit;cursor:pointer}
.mode-badge{display:inline-flex;align-items:center;gap:6px;padding:6px 10px;border-radius:var(--r-md);background:var(--surface-3);border:1px solid var(--line);font-size:12px;font-weight:800;color:var(--ink-500)}
.mode-badge.demo{background:var(--ember-100);color:var(--ember-500);border-color:transparent}
.mode-badge.stale{background:var(--bad-100);color:var(--bad-500);border-color:transparent}
.mode-badge.live{background:var(--ok-100);color:var(--ok-500);border-color:transparent}

/* ---- CONTROL MODE / SESSION ---- */
body:not([data-ctl="on"]) .nav-ctl,
body:not([data-ctl="on"]) #ses-bar,
body:not([data-ctl="on"]) #liveCoach,
body:not([data-ctl="on"]) #view-home,
body:not([data-ctl="on"]) #view-report,
body:not([data-ctl="on"]) #view-help{ display:none !important; }
#view-home,#view-report,#view-help{ display:none; }
body[data-ctl="on"][data-view="home"] #view-home,
body[data-ctl="on"][data-view="report"] #view-report,
body[data-ctl="on"][data-view="help"] #view-help{ display:block; }
body[data-ctl="on"]:not([data-view="live"]) .tab,
body[data-ctl="on"]:not([data-view="live"]) #fwBadge,
body[data-ctl="on"]:not([data-view="live"]) #liveCoach,
body[data-ctl="on"]:not([data-view="live"]) #chips,
body[data-ctl="on"]:not([data-view="live"]) .r-item:not(.nav-ctl):not(.rail-action){ display:none !important; }
.nav-ctl{font-weight:800}
.nav-ctl.active{background:color-mix(in oklab,var(--brand-500) 10%,var(--surface-1));color:var(--ink-900);border:1px solid color-mix(in oklab,var(--brand-500) 18%,transparent)}
.ses-bar{display:flex;align-items:center;gap:12px;padding:12px 16px;margin-bottom:14px;border-radius:var(--r-lg);background:var(--surface-1);border:1px solid color-mix(in oklab,var(--brand-500) 28%,var(--line));font-size:13px;box-shadow:var(--shadow-2)}
.ses-bar .ses-id{font-weight:900;color:var(--ink-900)}
.ses-bar .ses-meta{color:var(--ink-500);display:flex;gap:14px;flex-wrap:wrap}
.ses-bar .ses-spacer{flex:1}
.ses-bar button{appearance:none;border:none;padding:8px 14px;border-radius:var(--r-md);font:700 13px/1 inherit;cursor:pointer;background:var(--bad-100);color:var(--bad-500)}
.ses-bar button:disabled{opacity:0.5;cursor:not-allowed}
.ses-bar .ses-progress{flex:1 1 180px;min-width:140px;max-width:320px;height:6px;border-radius:var(--r-pill);background:var(--surface-3);overflow:hidden;position:relative}
.ses-bar .ses-progress .ses-progress-fill{height:100%;width:0%;background:linear-gradient(90deg,var(--brand-500),var(--signal-500));border-radius:inherit;transition:width var(--dur-m) var(--ease)}
.ses-bar .ses-progress[data-manual="1"] .ses-progress-fill{background:var(--line-2)}
.ses-bar .ses-progress[data-almost="1"] .ses-progress-fill{background:var(--ember-500)}
.ses-bar .ses-copy-btn{appearance:none;border:none;background:transparent;color:var(--ink-400);padding:2px 4px;margin-left:4px;border-radius:var(--r-md);cursor:pointer;display:inline-flex;align-items:center}
.ses-bar .ses-copy-btn:hover{background:var(--surface-3);color:var(--ink-700)}
.ses-bar .ses-copy-btn .material-symbols-rounded{font-size:16px}
.ses-bar button.confirm{background:var(--bad-500);color:#fff;animation:sesStopPulse 1.2s var(--ease) infinite}
@keyframes sesStopPulse{0%,100%{box-shadow:0 0 0 0 rgba(185,28,28,.45)}50%{box-shadow:0 0 0 6px rgba(185,28,28,0)}}

/* ---- SETUP / PREFLIGHT ---- */
.page-container{display:flex;flex-direction:column;gap:var(--gap)}
.home-layout{display:grid;grid-template-columns:minmax(0,5fr) minmax(360px,3fr);gap:var(--gap);align-items:start}
.welcome-hero{position:relative;overflow:hidden;margin-bottom:var(--gap);padding:36px 32px;min-height:220px;border-radius:var(--r-xl);background:radial-gradient(420px 260px at 100% 0%,color-mix(in oklab,var(--signal-500) 16%,transparent),transparent 62%),linear-gradient(145deg,var(--surface-1),var(--surface-2));border:1px solid var(--line);box-shadow:var(--shadow-3)}
.welcome-hero::before{content:'';position:absolute;inset:0;background:linear-gradient(90deg,color-mix(in oklab,var(--line) 28%,transparent) 1px,transparent 1px) 0 0/28px 28px,linear-gradient(0deg,color-mix(in oklab,var(--line) 28%,transparent) 1px,transparent 1px) 0 0/28px 28px;mask-image:linear-gradient(90deg,transparent,black 22%,black 76%,transparent);opacity:.5;pointer-events:none}
.welcome-hero>*{position:relative}
.hero-title{margin:0 0 10px;font-family:'Inter Tight','Inter',sans-serif;font-size:48px;line-height:1.04;font-weight:900;color:var(--ink-900);letter-spacing:-.02em}
.hero-subtitle{font-size:15px;color:var(--ink-500);max-width:62ch;line-height:1.55;font-weight:500}
.card-header{display:flex;align-items:center;gap:12px;margin-bottom:22px}
.card-header .material-symbols-rounded{font-size:23px;color:#fff;background:linear-gradient(135deg,var(--brand-500),var(--brand-400));padding:10px;border-radius:var(--r-md)}
.card-header h3{margin:0;font-family:'Inter Tight','Inter',sans-serif;font-size:18px;font-weight:800}
.card-header.space-between{justify-content:space-between}
.title-with-icon{display:flex;align-items:center;gap:12px}
.sys-card{border-radius:var(--r-xl);background:color-mix(in oklab,var(--surface-1) 96%,transparent);border:1px solid var(--line);padding:var(--pad);box-shadow:var(--shadow-2)}
.sys-card:hover{border-color:color-mix(in oklab,var(--brand-500) 22%,var(--line))}
.setup-form{display:grid;gap:26px}
.setup-group{display:flex;flex-direction:column;gap:14px}
.group-label{font-size:11px;font-weight:900;letter-spacing:.13em;text-transform:uppercase;color:var(--ink-400);border-bottom:1px solid var(--line);padding-bottom:8px}
.hardware-row,.meta-row{display:grid;grid-template-columns:160px minmax(0,1fr);align-items:center;gap:14px}
.hardware-row label,.meta-row label{font-weight:700;color:var(--ink-800)}
.setup-form input[type=text],.setup-form input[type=number],.setup-form select,.sess-filter input,.setup input[type=text],.setup input[type=number],.setup select{
  width:100%;padding:11px 15px;border:1px solid var(--line);border-radius:var(--r-md);
  background:var(--surface-2);color:var(--ink-900);font:600 14px/1.35 'Inter',sans-serif;
  transition:border-color var(--dur-s) var(--ease),box-shadow var(--dur-s) var(--ease);
}
.setup-form input:focus,.setup-form select:focus,.setup input:focus,.setup select:focus{outline:none;border-color:var(--brand-500);box-shadow:0 0 0 3px color-mix(in oklab,var(--brand-500) 16%,transparent)}
.input-with-action{display:flex;gap:10px;width:100%}
.input-with-action input{flex:1}
.action-btn,.ghost-btn{appearance:none;border:1px solid var(--line);border-radius:var(--r-md);background:var(--surface-3);color:var(--ink-700);padding:0 16px;min-height:42px;font:700 13px/1 'Inter',sans-serif;cursor:pointer;transition:background var(--dur-s) var(--ease)}
.ghost-btn{min-height:auto;padding:8px 12px;color:var(--brand-500);background:transparent}
.action-btn:hover,.ghost-btn:hover{background:var(--surface-3);border-color:var(--line-2)}
.inline-msg{grid-column:2;font-size:12px;color:var(--ink-400);font-family:'JetBrains Mono',monospace}
.advanced-section{border:1px solid var(--line);border-radius:var(--r-md);overflow:hidden}
.advanced-section summary{padding:14px 18px;background:var(--surface-3);cursor:pointer;font-weight:800;font-size:13px;color:var(--ink-500)}
.adv-grid{padding:18px;background:var(--surface-2);display:grid;gap:14px}
.pf-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:10px}
.pf-tile{padding:12px;border-radius:var(--r-md);background:var(--surface-2);border:1px solid var(--line);border-left:4px solid var(--line-2)}
.pf-tile.ok,.pf-tile.good{border-left-color:var(--ok-500)}
.pf-tile.warn{border-left-color:var(--ember-500);background:color-mix(in oklab,var(--ember-100) 30%,var(--surface-1))}
.pf-tile.fail{border-left-color:var(--bad-500);background:color-mix(in oklab,var(--bad-100) 30%,var(--surface-1))}
.pf-tile .pf-top{display:flex;align-items:center;gap:8px;font-weight:800;flex-wrap:wrap}
.pf-tile .pf-detail{margin-top:6px;font-size:12px;color:var(--ink-500);word-break:break-word}
.pf-tile .pf-rem{margin-top:6px;font-size:12px;font-weight:650;background:color-mix(in oklab,var(--ember-500) 8%,transparent);padding:8px;border-radius:var(--r-sm)}
.pf-tile button{margin-top:8px;appearance:none;border:1px solid var(--line);background:transparent;color:inherit;padding:5px 10px;border-radius:var(--r-pill);font:700 12px/1 inherit;cursor:pointer}
.pf-tag{display:inline-flex;align-items:center;gap:4px;padding:2px 8px;border-radius:var(--r-pill);font-size:10px;font-weight:900;letter-spacing:.06em;text-transform:uppercase;background:var(--surface-3);color:var(--ink-400);border:1px solid var(--line)}
.pf-tag.req{background:var(--brand-100);color:var(--brand-500);border-color:transparent}
.setup{display:flex;flex-direction:column;gap:14px}
.chip-group{display:flex;flex-wrap:wrap;gap:8px;align-items:center}
.chip-btn{appearance:none;border:1px solid var(--line);background:var(--surface-1);padding:8px 14px;border-radius:var(--r-md);font:700 13px/1 inherit;cursor:pointer;color:var(--ink-700);display:inline-flex;gap:6px;align-items:center;transition:all var(--dur-s) var(--ease)}
.chip-btn:hover{background:var(--surface-3);border-color:var(--line-2)}
.chip-btn.active{background:var(--brand-500);color:#fff;border-color:var(--brand-500);box-shadow:0 4px 10px -3px color-mix(in oklab,var(--brand-500) 55%,transparent)}
.chip-btn .material-symbols-rounded{font-size:15px}
.launch-panel{border-radius:var(--r-xl);background:var(--surface-1);border:1px solid var(--line);padding:22px;box-shadow:var(--shadow-3)}
.launch-btn,.setup-start{width:100%;border:none;border-radius:var(--r-md);padding:16px 22px;font:900 16px/1 'Inter',sans-serif;cursor:pointer;background:linear-gradient(135deg,var(--brand-500),color-mix(in oklab,var(--brand-500) 72%,var(--signal-500)));color:#fff;box-shadow:0 10px 24px -12px var(--brand-500);transition:transform var(--dur-s) var(--ease)}
.launch-btn:hover:not(:disabled),.setup-start:hover:not(:disabled){transform:translateY(-1px)}
.launch-btn:disabled,.setup-start:disabled{opacity:.4;cursor:not-allowed;background:var(--line-2);box-shadow:none}
.setup-start.in-progress{background:var(--surface-3);color:var(--ink-700);opacity:1;cursor:pointer;box-shadow:none}
.setup-alert{padding:10px 14px;border-radius:var(--r-md);background:var(--bad-100);color:var(--bad-500);font-size:13px;margin-top:10px}
.sandbox-note{margin-top:14px;padding:12px 14px;border:1px solid var(--line-2);border-left:4px solid var(--ember-500);border-radius:var(--r-md);background:color-mix(in oklab,var(--ember-100) 30%,var(--surface-1));color:var(--ink-700);font-size:13px;line-height:1.45}
.sandbox-note strong{display:block;font-size:11px;text-transform:uppercase;letter-spacing:.08em;color:var(--ember-500);margin-bottom:3px}
.sandbox-note.connected{border-left-color:var(--ok-500);background:color-mix(in oklab,var(--ok-100) 28%,var(--surface-1))}
.sandbox-note.connected strong{color:var(--ok-500)}
.setup-help{font-size:12px;font-weight:500;color:var(--ink-400);line-height:1.45;margin:-4px 0 2px}
.setup-help.ok{color:var(--ok-500)}
.setup-help.warn{color:var(--ember-500)}
.setup input.invalid-soft{border-color:var(--ember-500);box-shadow:inset 0 0 0 1px var(--ember-500)}

/* ---- SESSIONS TABLE ---- */
.sess-filter{display:flex;gap:10px;margin-bottom:12px;align-items:center;flex-wrap:wrap}
.sess-filter .count{color:var(--ink-400);font-size:12px;font-weight:700}
.sess-table{border-collapse:separate;border-spacing:0;width:100%;font-size:13px}
.sess-table th{font-size:11px;letter-spacing:.1em;text-transform:uppercase;color:var(--ink-400);padding:10px 12px;border-bottom:1px solid var(--line)}
.sess-table td{padding:11px 12px;border-bottom:1px solid var(--line);text-align:left}
.sess-table tr{cursor:pointer}
.sess-table tr:hover td{background:var(--surface-3)}
.sess-table tr td:last-child::after{content:"open ›";margin-left:8px;font-size:11px;font-weight:700;opacity:0;transition:opacity var(--dur-s) var(--ease);color:var(--brand-500);font-family:inherit}
.sess-table tr:hover td:last-child::after,.sess-table tr:focus-within td:last-child::after{opacity:1}
.sess-empty{display:grid;gap:8px;padding:16px 18px;border-radius:var(--r-md);background:var(--surface-3);border:1px dashed var(--line);color:var(--ink-500);font-size:13px;font-weight:500}
.sess-empty strong{color:var(--ink-900);font-weight:900}
.mini-radar-card{overflow:hidden}
.home-radar-wrap{height:230px;border-radius:var(--r-lg);background:radial-gradient(120% 120% at 50% 120%,#0b1220,#111a2c 50%,#0a0f1c 80%);border:1px solid var(--line);overflow:hidden}
.home-radar-wrap canvas{width:100%!important;height:230px!important}
.sys-preflight{position:sticky;top:92px;box-shadow:var(--shadow-3)}

/* ---- COACH CARDS ---- */
.coach-grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:12px;margin-bottom:var(--gap);align-items:stretch}
.coach-grid .flt{border-radius:var(--r-lg);min-height:88px;padding:14px 16px 14px 20px;box-sizing:border-box;align-content:start;overflow:visible;box-shadow:var(--shadow-1)}
.coach-grid .flt-t,.coach-grid .flt-c{min-width:0;overflow-wrap:anywhere}
.coach-grid .flt-t{align-items:flex-start;line-height:1.25}
.coach-grid .flt-t .material-symbols-rounded{flex:0 0 auto}

/* ---- REPORT ---- */
.report-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:var(--gap);margin-top:var(--gap)}
.report-grid.three{grid-template-columns:repeat(3,minmax(0,1fr))}
.verdict-card{border-radius:var(--r-xl);padding:22px;background:var(--surface-1);border:1px solid var(--line);display:grid;gap:14px;box-shadow:var(--shadow-2)}
.verdict-card.ready{background:color-mix(in oklab,var(--ok-100) 45%,var(--surface-1));border-color:color-mix(in oklab,var(--ok-500) 18%,transparent)}
.verdict-card.degraded_signal,.verdict-card.provenance_warning{background:color-mix(in oklab,var(--ember-100) 45%,var(--surface-1));border-color:color-mix(in oklab,var(--ember-500) 18%,transparent)}
.verdict-card.schema_warning{background:color-mix(in oklab,var(--brand-100) 45%,var(--surface-1));border-color:color-mix(in oklab,var(--brand-500) 14%,transparent)}
.verdict-card.deferred{background:var(--surface-3)}
.verdict-card.firmware_rejected,.verdict-card.not_ready{background:color-mix(in oklab,var(--bad-100) 45%,var(--surface-1));border-color:color-mix(in oklab,var(--bad-500) 18%,transparent)}
.verdict-head{display:flex;align-items:flex-start;justify-content:space-between;gap:16px;flex-wrap:wrap}
.verdict-title{margin:0;font-family:'Inter Tight','Inter',sans-serif;font-size:22px;font-weight:900;letter-spacing:-.01em}
.verdict-kind{display:inline-flex;align-items:center;gap:6px;padding:6px 10px;border-radius:var(--r-pill);font-size:12px;font-weight:900;text-transform:uppercase;letter-spacing:.06em}
.verdict-kind .material-symbols-rounded{font-size:16px}
.verdict-kind.ready{background:var(--ok-100);color:var(--ok-500)}
.verdict-kind.degraded_signal,.verdict-kind.provenance_warning{background:var(--ember-100);color:var(--ember-500)}
.verdict-kind.schema_warning{background:var(--brand-100);color:var(--brand-500)}
.verdict-kind.deferred{background:var(--surface-3);color:var(--ink-500)}
.verdict-kind.firmware_rejected,.verdict-kind.not_ready{background:var(--bad-100);color:var(--bad-500)}
.report-list{display:grid;gap:8px}
.report-list .flt{border-radius:var(--r-md)}
.stat-list{display:grid;gap:8px}
.stat-list .stat{display:grid;grid-template-columns:minmax(0,1fr) auto;gap:12px;align-items:baseline;padding:10px 12px;border-radius:var(--r-md);background:var(--surface-3);border:1px solid var(--line)}
.stat-list .stat .k{font-size:13px;font-weight:700;color:var(--ink-500)}
.stat-list .stat .v{font-family:'JetBrains Mono',monospace;font-size:13px;font-weight:700;text-align:right;word-break:break-word;color:var(--ink-900)}
.stat-list .stat.good{background:var(--ok-100);color:var(--ok-500);border-color:transparent}
.stat-list .stat.warn{background:var(--ember-100);color:var(--ember-500);border-color:transparent}
.stat-list .stat.bad{background:var(--bad-100);color:var(--bad-500);border-color:transparent}
.mini-bars{display:grid;gap:8px}
.mini-bar{display:grid;grid-template-columns:minmax(120px,1fr) minmax(100px,2fr) auto;gap:10px;align-items:center;font-size:12px}
.mini-bar-track{height:8px;border-radius:var(--r-pill);background:var(--surface-3);overflow:hidden}
.mini-bar-fill{height:100%;background:linear-gradient(90deg,var(--brand-500),var(--signal-500));border-radius:inherit}
.verdict-actions{display:flex;flex-wrap:wrap;gap:8px;align-items:center}
.compare-grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:10px;margin-top:10px}
.compare-card{padding:12px;border-radius:var(--r-md);background:var(--surface-3);border:1px solid var(--line)}
.compare-card h3{margin:0 0 8px;font-size:14px}
.contract-list{display:grid;gap:8px;margin-top:10px}
.contract-list .flt{border-radius:var(--r-md)}

/* ---- HELP ---- */
[data-help]{cursor:help}
.help-toolbar{display:flex;gap:10px;flex-wrap:wrap;align-items:center;margin-bottom:16px}
.help-search{min-width:260px;flex:1;padding:11px 14px;border-radius:var(--r-md);border:1px solid var(--line);background:var(--surface-2);color:var(--ink-900);font:500 14px/1.3 inherit}
.help-section{margin-top:12px;padding:14px;border:1px solid var(--line);border-radius:var(--r-lg);background:var(--surface-1)}
.help-section summary{font-weight:900;cursor:pointer;font-size:14px}
.help-section dl{display:grid;gap:10px;margin:10px 0 0}
.help-section dt{font-weight:900;font-size:13px}
.help-section dd{margin:2px 0 0;color:var(--ink-500);line-height:1.45;font-size:13px}
.tip-pop{position:fixed;z-index:1000;max-width:320px;padding:10px 12px;border-radius:var(--r-md);background:var(--ink-900);color:#fff;box-shadow:var(--shadow-3);font-size:12px;line-height:1.4;pointer-events:none;display:none}

/* ---- FOCUS / A11Y ---- */
:where(button,a,input,select,textarea,.r-item,.chip-btn,.pf-tile,.sw,.ca-btn,.ic-btn,.fab,.setup-start,[tabindex]):focus-visible{
  outline:2px solid var(--brand-500);outline-offset:2px;border-radius:inherit;
}
.countdown-dots{display:inline-flex;gap:6px;margin-left:10px;vertical-align:middle}
.countdown-dots span{width:8px;height:8px;border-radius:50%;background:var(--line-2);transition:background var(--dur-s) var(--ease)}
.countdown-dots span.on{background:var(--brand-500);box-shadow:0 0 0 3px color-mix(in oklab,var(--brand-500) 20%,transparent)}
.kbd-hints{display:flex;flex-wrap:wrap;gap:14px;padding:10px 4px 0;font-size:11px;color:var(--ink-400);font-family:'JetBrains Mono',monospace}
.kbd-hints .khint{display:inline-flex;align-items:center;gap:6px}
.sticky-cta{position:fixed;left:50%;transform:translateX(-50%);bottom:20px;z-index:140;display:none;align-items:center;gap:10px;padding:8px 10px 8px 16px;border-radius:var(--r-pill);background:var(--surface-1);box-shadow:var(--shadow-2);border:1px solid var(--line);font-size:13px;font-weight:700;color:var(--ink-700)}
.sticky-cta.visible{display:inline-flex}
.sticky-cta button{appearance:none;border:none;padding:10px 18px;border-radius:var(--r-pill);background:var(--brand-500);color:#fff;font:900 13px/1 inherit;cursor:pointer}
.sticky-cta button:disabled{opacity:.45;cursor:not-allowed}
.sticky-cta .sticky-note{color:var(--ink-400);font-weight:500;font-size:12px}

/* ---- RESPONSIVE ---- */
@media (max-width:1200px){
  .app{grid-template-columns:1fr}
  .rail{position:sticky;top:0;height:auto;flex-direction:row;overflow-x:auto;padding:10px 12px;border-right:none;border-bottom:1px solid var(--line)}
  .brand{width:48px;height:48px;margin:0 8px 0 0;flex:0 0 auto}
  .r-item{grid-template-columns:32px auto;min-width:max-content}
  .r-item .nav-kbd{display:none}
  .rail-foot{margin:0 0 0 auto;flex-direction:row;border-top:none;padding-top:0}
  .operator{display:none}
  .r-item.active::before{display:none}
  .home-layout{grid-template-columns:1fr}
  .sys-preflight{position:relative;top:auto}
  .kpi-grid{grid-template-columns:repeat(2,minmax(0,1fr))}
  .rad{grid-template-columns:1fr}
  .c3,.c4{grid-template-columns:repeat(2,minmax(0,1fr))}
}
@media (max-width:900px){
  .report-grid,.report-grid.three{grid-template-columns:1fr}
  .coach-grid,.compare-grid{grid-template-columns:1fr}
}
@media (max-width:760px){
  body{overflow-x:hidden}
  .app{width:100%;max-width:100vw;overflow-x:hidden}
  .rail{width:100%;max-width:100vw;min-width:0}
  .main{width:100%;max-width:100vw;min-width:0;padding:16px;overflow-x:hidden}
  .topbar{display:grid;grid-template-columns:1fr;align-items:start}
  .topbar .sp{display:none}
  .topbar h1{font-size:30px;overflow-wrap:anywhere}
  .welcome-hero{padding:28px 18px}
  .hero-title{font-size:34px;max-width:100%;overflow-wrap:anywhere}
  .hero-subtitle,.sandbox-note{max-width:100%;overflow-wrap:anywhere}
  .k-val{font-size:40px}
  .tl .v{font-size:22px}
  .hardware-row,.meta-row{grid-template-columns:1fr;gap:6px}
  .kpi-grid,.c2,.c3,.c4,.wg,.tbig{grid-template-columns:1fr}
  .coach-grid{grid-template-columns:1fr}
}
</style>
</head>
<body>

<div class="app">
<aside class="rail" aria-label="Primary">
  <div class="brand" title="Radar Vital Trainer">
    <div class="brand-mark"><span class="brand-ring"></span><span class="brand-ring r2"></span><span class="brand-dot"></span></div>
    <div class="brand-label"><span class="bl-top">RADAR</span><span class="bl-bot">VITAL · v15.1</span></div>
  </div>
  <nav class="nav-groups" aria-label="Views">
    <div class="nav-group" id="workflowGroup">
      <div class="nav-heading">Workflow</div>
    </div>
    <div class="nav-group" id="liveTabsGroup">
      <div class="nav-heading">Live views</div>
      <button class="r-item sub active" data-target="tab-overview" onclick="switchTab('tab-overview')"><span class="r-ic"><span class="material-symbols-rounded">dashboard</span></span>Overview</button>
      <button class="r-item sub" data-target="tab-waves" onclick="switchTab('tab-waves')"><span class="r-ic"><span class="material-symbols-rounded">show_chart</span></span>Waves</button>
      <button class="r-item sub" data-target="tab-hr" onclick="switchTab('tab-hr')"><span class="r-ic"><span class="material-symbols-rounded">ecg_heart</span></span>HR funnel</button>
      <button class="r-item sub" data-target="tab-rr" onclick="switchTab('tab-rr')"><span class="r-ic"><span class="material-symbols-rounded">pulmonology</span></span>RR funnel</button>
      <button class="r-item sub" data-target="tab-snaps" onclick="switchTab('tab-snaps')"><span class="r-ic"><span class="material-symbols-rounded">bookmark</span></span>Snapshots</button>
      <button class="r-item sub" data-target="tab-audit" onclick="switchTab('tab-audit')"><span class="r-ic"><span class="material-symbols-rounded">fact_check</span></span>Audit</button>
    </div>
  </nav>
  <div class="rail-foot">
    <button class="r-item rail-action" onclick="openPalette()" title="Command palette (⌘K)"><span class="r-ic"><span class="material-symbols-rounded">search</span></span>Command<span class="nav-kbd">⌘K</span></button>
    <div class="operator">
      <div class="op-avatar" id="opAvatar">OA</div>
      <div class="op-meta">
        <div class="op-name" id="opName">Operator A</div>
        <div class="op-role" id="opRole">Lab · Station 3</div>
      </div>
    </div>
  </div>
</aside>

<main class="main">

  <div class="topbar">
    <div>
      <div class="eyebrow">Live collection · v14 / v9</div>
      <h1>Radar Vital Trainer</h1>
    </div>
    <div class="sp"></div>
    <div class="conn" id="conn"><span class="dot"></span><span id="connT">Connecting…</span></div>
    <div class="mode-badge" id="modeBadge"><span class="material-symbols-rounded">wifi_tethering_off</span><span id="modeBadgeT">waiting</span></div>
    <div class="seg" role="group">
      <button class="active" data-density="comfortable" onclick="setDensity('comfortable')" title="Comfortable"><span class="material-symbols-rounded">density_medium</span></button>
      <button data-density="compact" onclick="setDensity('compact')" title="Compact"><span class="material-symbols-rounded">density_small</span></button>
    </div>
    <button class="ic-btn" onclick="togglePause()" title="Pause / resume live polling"><span class="material-symbols-rounded" id="pauseIcon">pause</span></button>
    <button class="ic-btn" onclick="toggleTheme()" title="Toggle dark mode"><span class="material-symbols-rounded" id="themeIcon">dark_mode</span></button>
    <button class="ic-btn" onclick="openSettings()" title="Settings"><span class="material-symbols-rounded">tune</span></button>
    <button class="ic-btn" onclick="openDrawer()" title="Alerts"><span class="material-symbols-rounded">notifications</span><span class="ic-badge" id="alertsBadge" style="display:none;">0</span></button>
    <button class="fab" onclick="exportData()" title="Export JSON payload"><span class="material-symbols-rounded">download</span>Export</button>
  </div>

  <div class="fw warn" id="fwBadge" data-help="fw_truthfulness" tabindex="0">
    <span class="material-symbols-rounded">gpp_maybe</span>
    <div>
      <div class="fw-title">Firmware truthfulness</div>
      <div class="fw-line"><span>sketch=--</span><span>module=--</span><span>module_valid=--</span><span>schema=--</span></div>
    </div>
    <button class="ca-btn" onclick="switchTab('tab-audit')" title="View audit details"><span class="material-symbols-rounded">arrow_forward</span></button>
  </div>

  <div class="chips" id="chips"></div>
  <div style="height:16px"></div>

  <!-- OVERVIEW -->
  <div id="tab-overview" class="tab active">
    <div class="kpi-grid" style="margin-bottom:var(--gap)">
      <div class="kpi hr" data-help="kpi.hr" tabindex="0">
        <div class="k-lab"><span class="k-lab-t">Heart rate</span><div class="k-ic"><span class="material-symbols-rounded">ecg_heart</span></div></div>
        <div class="k-val-r"><span class="k-val" id="kpiHr">--</span><span class="k-u">bpm</span></div>
        <canvas class="k-spark" id="kpiHrSpark"></canvas>
        <div class="k-sub" id="kpiHrSub">waiting</div>
      </div>
      <div class="kpi rr" data-help="kpi.rr" tabindex="0">
        <div class="k-lab"><span class="k-lab-t">Respiration</span><div class="k-ic"><span class="material-symbols-rounded">air</span></div></div>
        <div class="k-val-r"><span class="k-val" id="kpiRr">--</span><span class="k-u">br/min</span></div>
        <canvas class="k-spark" id="kpiRrSpark"></canvas>
        <div class="k-sub" id="kpiRrSub">waiting</div>
      </div>
      <div class="kpi" data-help="kpi.fps" tabindex="0">
        <div class="k-lab"><span class="k-lab-t">Frame rate</span><div class="k-ic"><span class="material-symbols-rounded">speed</span></div></div>
        <div class="k-val-r"><span class="k-val" id="kpiFps">--</span><span class="k-u">Hz</span></div>
        <canvas class="k-spark" id="kpiFpsSpark"></canvas>
        <div class="k-sub" id="kpiFpsSub">DSP loop</div>
      </div>
      <div class="kpi" data-help="kpi.distance" tabindex="0">
        <div class="k-lab"><span class="k-lab-t">Range</span><div class="k-ic"><span class="material-symbols-rounded">radar</span></div></div>
        <div class="k-val-r"><span class="k-val" id="kpiDist">--</span><span class="k-u">cm</span></div>
        <canvas class="k-spark" id="kpiDistSpark"></canvas>
        <div class="k-sub" id="kpiDistSub">target distance</div>
      </div>
    </div>

    <article class="card" style="margin-bottom:var(--gap)">
      <div class="ch">
        <div class="tw"><span class="material-symbols-rounded">explore</span><div><h2 class="ct">Position / module view</h2><div class="cs">Target-info-first spatial telemetry with live module side</div></div></div>
        <div class="ca"><button class="ca-btn" onclick="snapshotNow()" title="Pin snapshot"><span class="material-symbols-rounded">bookmark_add</span></button></div>
      </div>
      <div class="rad">
        <div class="rp">
          <div class="rp-h">
            <div><div class="eyebrow" style="color:var(--ink-400)">Spatial view</div><div class="rp-n">Semicircle position uses target-info-first. Module telemetry stays in sync below.</div></div>
            <div class="tgt" id="tgtCount">Target: --</div>
          </div>
          <div class="rad-w"><canvas id="radarCanvas"></canvas></div>
        </div>
        <div class="side">
          <div class="tb">
            <div class="tbig">
              <div class="tl"><span class="l">FPS</span><span class="v" id="fpsV">--</span></div>
              <div class="tl"><span class="l">Breath</span><span class="v rr" id="brV">--</span></div>
              <div class="tl"><span class="l">Heart</span><span class="v" id="hrV">--</span></div>
            </div>
            <div class="kv">
              <div>Person detected</div><div id="personV">--</div>
              <div>Distance</div><div id="distV">--</div>
              <div>X position</div><div id="xV">--</div>
              <div>Y position</div><div id="yV">--</div>
              <div>Radius</div><div id="radV">--</div>
              <div>Firmware</div><div id="fwV">--</div>
              <div>Spatial source</div><div id="srcV">--</div>
              <div>Spatial age</div><div id="srcAgeV">--</div>
              <div>Doppler index</div><div id="dopV">--</div>
              <div>Doppler speed</div><div id="dopCmsV">--</div>
            </div>
          </div>
          <div class="tb">
            <div class="ct" style="font-size:14px;margin-bottom:10px;">Position source summary</div>
            <table><tbody id="posTable"></tbody></table>
          </div>
        </div>
      </div>
    </article>

    <article class="card" style="margin-bottom:var(--gap)">
      <div class="ch"><div class="tw"><span class="material-symbols-rounded">rule_settings</span><div><h2 class="ct">Physiology vs policy</h2><div class="cs">Signal quality and publish policy in one view</div></div></div></div>
      <div class="grid c2">
        <div class="tb"><h3 style="margin:0 0 10px;font-size:13px;font-weight:900;letter-spacing:.06em;text-transform:uppercase;color:var(--ink-400)">Physiology</h3><table><tbody id="physTable"></tbody></table></div>
        <div class="tb"><h3 style="margin:0 0 10px;font-size:13px;font-weight:900;letter-spacing:.06em;text-transform:uppercase;color:var(--ink-400)">Policy</h3><table><tbody id="polTable"></tbody></table></div>
      </div>
    </article>

    <div class="grid c4">
      <article class="card">
        <div class="ch"><div class="tw"><span class="material-symbols-rounded">radar</span><div><h2 class="ct">Radar</h2><div class="cs">Latest parsed row</div></div></div></div>
        <div class="mr"><div class="m"><span class="m-l">HR</span><div class="m-vr"><span class="m-v" id="radHr">--</span><span class="m-u">bpm</span></div></div><div class="m rr"><span class="m-l">RR</span><div class="m-vr"><span class="m-v" id="radRr">--</span><span class="m-u">br/m</span></div></div></div>
        <table><tbody id="radarTable"></tbody></table>
      </article>
      <article class="card">
        <div class="ch"><div class="tw"><span class="material-symbols-rounded">bluetooth_connected</span><div><h2 class="ct">BLE Reference</h2><div class="cs">Oximeter packet</div></div></div></div>
        <div class="mr"><div class="m"><span class="m-l">HR</span><div class="m-vr"><span class="m-v" id="bleHr">--</span><span class="m-u">bpm</span></div></div><div class="m rr"><span class="m-l">RR</span><div class="m-vr"><span class="m-v" id="bleRr">--</span><span class="m-u">br/m</span></div></div></div>
        <table><tbody id="bleTable"></tbody></table>
      </article>
      <article class="card">
        <div class="ch"><div class="tw"><span class="material-symbols-rounded">difference</span><div><h2 class="ct">Mismatch</h2><div class="cs">Operator-facing delta</div></div></div></div>
        <div class="faults" id="faults"></div>
      </article>
      <article class="card">
        <div class="ch"><div class="tw"><span class="material-symbols-rounded">my_location</span><div><h2 class="ct">Spatial</h2><div class="cs">Point cloud & Doppler</div></div></div></div>
        <div class="mr three"><div class="m"><span class="m-l">Targets</span><div class="m-vr"><span class="m-v" id="numT">--</span></div></div><div class="m rr"><span class="m-l">|Dop|</span><div class="m-vr"><span class="m-v" id="maxDop">--</span></div></div><div class="m"><span class="m-l">cm/s</span><div class="m-vr"><span class="m-v" id="maxDopSp">--</span></div></div></div>
        <table><tbody id="spTable"></tbody></table>
      </article>
    </div>
  </div>

  <!-- WAVES -->
  <div id="tab-waves" class="tab">
    <article class="card" style="margin-bottom:var(--gap)">
      <div class="ch"><div class="tw"><span class="material-symbols-rounded">monitor_heart</span><div><h2 class="ct">Live phase waveforms</h2><div class="cs">breath_phase and heart_phase from 0x0A13</div></div></div><div class="ca"><button class="ca-btn" onclick="fsCard(this)" title="Fullscreen"><span class="material-symbols-rounded">fullscreen</span></button></div></div>
      <div class="wg"><div class="wc"><h3 class="wt">Breath</h3><div class="ws">Live breath_phase history</div><canvas id="breathChart"></canvas></div><div class="wc"><h3 class="wt">Heartbeat</h3><div class="ws">Live heart_phase history</div><canvas id="heartChart"></canvas></div></div>
    </article>
    <div class="grid c2">
      <article class="card">
        <div class="ch"><div class="tw"><span class="material-symbols-rounded">show_chart</span><div><h2 class="ct">Heart-rate trend</h2><div class="cs">Reported, candidate, raw, corrected raw, BLE, anchor</div></div></div>
          <div class="ca">
            <div class="seg" style="margin-right:6px">
              <button data-range="30" onclick="setRange(this,30)">30s</button>
              <button class="active" data-range="120" onclick="setRange(this,120)">2m</button>
              <button data-range="300" onclick="setRange(this,300)">5m</button>
              <button data-range="all" onclick="setRange(this,'all')">All</button>
            </div>
            <button class="ca-btn" onclick="fsCard(this)" title="Fullscreen"><span class="material-symbols-rounded">fullscreen</span></button>
          </div>
        </div>
        <div class="cp"><canvas id="hrChart"></canvas></div>
      </article>
      <article class="card">
        <div class="ch"><div class="tw rr"><span class="material-symbols-rounded">show_chart</span><div><h2 class="ct">Respiration trend</h2><div class="cs">Reported, candidate, final, anchor, BLE</div></div></div><div class="ca"><button class="ca-btn" onclick="fsCard(this)" title="Fullscreen"><span class="material-symbols-rounded">fullscreen</span></button></div></div>
        <div class="cp"><canvas id="rrChart"></canvas></div>
      </article>
    </div>
  </div>

  <!-- HR -->
  <div id="tab-hr" class="tab">
    <article class="card" style="margin-bottom:var(--gap)">
      <div class="ch"><div class="tw"><span class="material-symbols-rounded">scatter_plot</span><div><h2 class="ct">Raw-HR vs ref-HR rate-bucketed scatter</h2><div class="cs">Mean raw-minus-reference HR bias per 10-bpm reference bucket</div></div></div></div>
      <div class="cp s"><canvas id="rawHrBucketChart"></canvas></div>
      <table style="margin-top:12px"><tbody id="bucketTable"></tbody></table>
    </article>
    <div class="grid c2">
      <article class="card">
        <div class="ch"><div class="tw"><span class="material-symbols-rounded">conversion_path</span><div><h2 class="ct">HR funnel telemetry</h2><div class="cs">Arbiter → reject-phase → blend → coherence → publish</div></div></div></div>
        <div class="pills" id="hrPills" style="margin-bottom:14px"></div>
        <table><tbody id="hrFunTable"></tbody></table>
      </article>
      <article class="card">
        <div class="ch"><div class="tw"><span class="material-symbols-rounded">analytics</span><div><h2 class="ct">HR stage values</h2><div class="cs">Values carried through the HR path</div></div></div></div>
        <table><tbody id="hrStageTable"></tbody></table>
      </article>
    </div>
  </div>

  <!-- RR -->
  <div id="tab-rr" class="tab">
    <div class="grid c2" style="margin-bottom:var(--gap)">
      <article class="card">
        <div class="ch"><div class="tw rr"><span class="material-symbols-rounded">air</span><div><h2 class="ct">RR funnel telemetry</h2><div class="cs">Selector → accept → blend → Kalman → publish</div></div></div></div>
        <div class="pills" id="rrPills" style="margin-bottom:14px"></div>
        <table><tbody id="rrFunTable"></tbody></table>
      </article>
      <article class="card">
        <div class="ch"><div class="tw rr"><span class="material-symbols-rounded">timeline</span><div><h2 class="ct">RR stage values</h2><div class="cs">Anchor, recovery, final stage values</div></div></div></div>
        <table><tbody id="rrStageTable"></tbody></table>
      </article>
    </div>
    <div class="grid c2">
      <article class="card">
        <div class="ch"><div class="tw rr"><span class="material-symbols-rounded">monitoring</span><div><h2 class="ct">RR recovery diagnostics</h2><div class="cs">Confidence, recovery, seed consistency</div></div></div><div class="ca"><button class="ca-btn" onclick="fsCard(this)" title="Fullscreen"><span class="material-symbols-rounded">fullscreen</span></button></div></div>
        <div class="cp"><canvas id="rrDiagChart"></canvas></div>
      </article>
      <article class="card">
        <div class="ch"><div class="tw"><span class="material-symbols-rounded">notification_important</span><div><h2 class="ct">RR / stale-state warnings</h2><div class="cs">Anchor, candidate aging, integrity</div></div></div></div>
        <div class="faults" id="rrFaults"></div>
      </article>
    </div>
  </div>

  <!-- SNAPSHOTS -->
  <div id="tab-snaps" class="tab">
    <article class="card">
      <div class="ch">
        <div class="tw"><span class="material-symbols-rounded">bookmark</span><div><h2 class="ct">Pinned snapshots</h2><div class="cs">Capture an HR/RR/BLE/delta snapshot at the current frame to compare later</div></div></div>
        <div class="ca">
          <button class="fab" style="padding:8px 14px;font-size:13px" onclick="snapshotNow()"><span class="material-symbols-rounded">add</span>Pin current</button>
          <button class="ca-btn" onclick="clearSnaps()" title="Clear all"><span class="material-symbols-rounded">delete_sweep</span></button>
        </div>
      </div>
      <div class="snaps" id="snapsList"></div>
    </article>
  </div>

  <!-- AUDIT -->
  <div id="tab-audit" class="tab">
    <div class="grid c2" style="margin-bottom:var(--gap)">
      <article class="card"><div class="ch"><div class="tw"><span class="material-symbols-rounded">fact_check</span><div><h2 class="ct">Analysis audit</h2><div class="cs">Gates, firmware, funnel survival</div></div></div></div><table><tbody id="audTable"></tbody></table></article>
      <article class="card"><div class="ch"><div class="tw"><span class="material-symbols-rounded">stacked_bar_chart</span><div><h2 class="ct">Reason histograms</h2><div class="cs">Top block reasons + AGC</div></div></div></div><table><tbody id="histTable"></tbody></table></article>
    </div>
    <div class="grid c2">
      <article class="card"><div class="ch"><div class="tw"><span class="material-symbols-rounded">bluetooth_searching</span><div><h2 class="ct">BLE ref quality</h2><div class="cs">Packet, decode, perfusion</div></div></div></div><table><tbody id="bleQTable"></tbody></table></article>
      <article class="card"><div class="ch"><div class="tw"><span class="material-symbols-rounded">history</span><div><h2 class="ct">Recent events</h2><div class="cs">Live tail from current session</div></div></div></div><pre id="evLog">Waiting for data...</pre></article>
    </div>
    <div class="ft" id="foot"></div>
  </div>

</main>
</div>

<div class="dv-ov" id="dvOv" onclick="closeDrawer()"></div>
<aside class="dv" id="dv" aria-hidden="true">
  <div class="dv-h"><h3>Alerts</h3><button class="ic-btn" onclick="closeDrawer()" title="Close"><span class="material-symbols-rounded">close</span></button></div>
  <div class="dv-b" id="dvBody"></div>
</aside>

<div class="p-ov" id="pOv" onclick="if(event.target===this)closePalette()">
  <div class="p">
    <div class="p-iw"><span class="material-symbols-rounded">search</span><input class="p-in" id="pIn" placeholder="Type a command…" autocomplete="off"></div>
    <div class="p-l" id="pL"></div>
    <div class="p-hint"><span><kbd>↑</kbd> <kbd>↓</kbd> navigate</span><span><kbd>⏎</kbd> run</span><span><kbd>esc</kbd> close</span></div>
  </div>
</div>

<div class="set-ov" id="settingsOv" onclick="if(event.target===this)closeSettings()">
  <div class="set-md">
    <div class="set-h">
      <div>
        <h3 class="set-tt">Dashboard settings</h3>
        <div class="set-st">Keep live mode honest. Demo mode is explicit, not silent.</div>
      </div>
      <button class="ic-btn" onclick="closeSettings()" title="Close settings"><span class="material-symbols-rounded">close</span></button>
    </div>
    <section class="set-g">
      <h4 class="set-gl">Source mode</h4>
      <div class="set-r"><div class="set-copy"><strong>Demo mode</strong><span>Use generated sample telemetry instead of the trainer JSON feed. Intended for UI demos only.</span></div><button class="sw" id="demoModeSwitch" onclick="toggleDemoMode()" aria-pressed="false" title="Toggle demo mode"></button></div>
      <div class="set-r"><div class="set-copy"><strong>Auto-enable demo on disconnect</strong><span>If live polling fails repeatedly, switch to demo automatically. Default is off.</span></div><button class="sw" id="autoDemoSwitch" onclick="toggleAutoDemo()" aria-pressed="false" title="Toggle auto demo on disconnect"></button></div>
    </section>
    <section class="set-g">
      <h4 class="set-gl">Disconnect behavior</h4>
      <div class="set-r"><div class="set-copy"><strong>Freeze on stale live data</strong><span>When live data disappears, keep the last real payload visible and mark the dashboard stale instead of simulating new activity.</span></div><button class="sw on" id="freezeOnStaleSwitch" onclick="toggleFreezeOnStale()" aria-pressed="true" title="Toggle freeze on stale"></button></div>
    </section>
    <div class="set-actions"><button class="txt-btn" onclick="closeSettings()">Done</button></div>
  </div>
</div>

<div class="toast-host" id="toasts"></div>

<script>
const S = {
  charts:{}, dataUrl:'./live_dashboard.json', refreshMs:1000, lastPayload:null, lastLivePayload:null,
  disc:0, hardStaleN:5, hardStaleS:5, paused:false, rangeS:120,
  snaps: JSON.parse(localStorage.getItem('rvt-snaps')||'[]'),
  alertKeys:new Set(),
  spark:{ hr:[], rr:[], fps:[], dist:[] },
  demoT:0,
  demoMode:false,
  autoDemoOnDisconnect:false,
  freezeOnStale:true,
  autoDemoActive:false,
  ctl:{on:false,status:null,current:null,defaults:null,preflight:null,help:null,lastSessionId:null,stopPending:false,helpQuery:'',helpAdvanced:localStorage.getItem('rvt-help-advanced')==='1'},
  setup:{duration_s:30,customDuration:30,customUnit:'s',radar_port:'COM10',ble_address:'10:22:33:9E:8F:63',ble_profile:'ailink_oximeter',notify_char:'0000ffe2-0000-1000-8000-00805f9b34fb',subject_label:'',operator_label:'',skip_countdown:false},
};

function num(v){ const n=Number(v); return Number.isFinite(n)?n:null; }
function fmt(v,d=1){ const n=num(v); return n===null?'--':n.toFixed(d); }
function boolish(v){
  if(v===true||v===false) return v;
  if(v==null||v==='') return false;
  if(typeof v==='string'){ const s=v.trim().toLowerCase(); if(!s||s==='--'||s==='null'||s==='nan'||s==='none') return false; if(['false','no','n','off','fail','failed','invalid'].includes(s)) return false; if(['true','yes','y','on','pass','passed','valid'].includes(s)) return true; }
  const n=num(v); return n!==null?(n>=0.5):!!v;
}
function textOrDash(v){ if(v==null) return '--'; if(typeof v==='number'&&!Number.isFinite(v)) return '--'; const s=String(v); return (!s.trim()||s.trim().toLowerCase()==='nan')?'--':s; }
function esc(v){ return textOrDash(v).replace(/[&<>"']/g, ch=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[ch])); }
function safeCls(c){ return ['good','warn','bad'].includes(c)?c:''; }
function safeIcon(i){ return String(i||'info').replace(/[^a-z0-9_]/gi,'')||'info'; }
function row(l,v){ return `<tr><td>${esc(l)}</td><td>${esc(v)}</td></tr>`; }
function validSeriesOrNull(vals,flags){ if(!Array.isArray(vals)) return []; return vals.map((v,i)=>{ const ok=Array.isArray(flags)?boolish(flags[i]):true; const n=num(v); if(!ok) return null; return (n!==null&&n>0)?n:null; }); }
function livePub(p,k){ const v=getR(p,k); const n=num(v); if(n===null||n<=0) return null; if(k==='reported_hr'&&!boolish(getR(p,'logged_hr_valid'))) return null; if(k==='reported_rr'&&!boolish(getR(p,'logged_rr_valid'))) return null; return n; }
function chip(l,c,i){ return `<div class="chip ${safeCls(c)}"><span class="material-symbols-rounded">${safeIcon(i)}</span>${esc(l)}</div>`; }
function pill(l,v){ let c=''; const n=typeof v==='string'?v.trim().toLowerCase():v; if(n===1||n===true||n==='1'||n==='yes'||n==='triggered') c='good'; else if(n===0||n===false||n==='0'||n==='no') c='warn'; return `<div class="pill ${c}">${esc(l)}: ${esc(v)}</div>`; }
function flt(s,t,c){ const lv=safeCls(s)||'warn'; const i=lv==='bad'?'error':lv==='warn'?'warning':'check_circle'; return `<div class="flt ${lv}"><div class="flt-t"><span class="material-symbols-rounded">${i}</span>${esc(t)}</div><div class="flt-c">${esc(c)}</div></div>`; }
function topHist(h,lim=3){ if(!h||typeof h!=='object') return '--'; const a=Object.entries(h).sort((a,b)=>(Number(b[1])||0)-(Number(a[1])||0)); return a.length?a.slice(0,lim).map(([k,v])=>`${k}=${v}`).join(' | '):'--'; }
function first(...vs){ for(const v of vs) if(v!==null&&v!==undefined&&textOrDash(v)!=='--') return v; return null; }
function shortHash(v){ const s=textOrDash(v); return s==='--'?'--':(s.length>16?`${s.slice(0,12)}...`:s); }
function featureSchema(p){ const a=p.analysis||{}; const m=a.feature_manifest||a.manifest||a.model_manifest||{}; return first(m.feature_schema_hash,m.schema_hash,m.hash,a.feature_schema_hash,a.schema_hash,p.meta?.feature_schema_hash,p.radar?.feature_schema_hash); }
function fwFromKeys(o,k){ const p=k.map(x=>num(o?.[x])); return p.every(v=>v!==null&&v>=0)?p.map(v=>Math.round(v)).join('.'):null; }
function fwTruth(p){
  const r=p.radar||{}; const a=p.analysis||{}; const fw=a.fw_truthfulness||{};
  const s=first(r.sketch_firmware_version,r.firmware_version,fw.version,fwFromKeys(r,['sketch_major','sketch_sub','sketch_mod']));
  const m=first(r.module_firmware_version,fw.module_version,fwFromKeys(r,['module_fw_major','module_fw_sub','module_fw_mod']),'unknown');
  const mvRaw=first(r.module_fw_valid,fw.module_version_valid); const mvKnown=mvRaw!==null&&mvRaw!==undefined&&textOrDash(mvRaw)!=='--'; const mv=mvKnown?boolish(mvRaw):false;
  return { sketch:textOrDash(s), moduleFw:textOrDash(m), moduleValid:mv, moduleValidText:mv?'True':'False', schemaHash:shortHash(featureSchema(p)), sev:mv?'good':(mvKnown||textOrDash(m).toLowerCase()==='unknown'?'bad':'warn') };
}
function renderFwBadge(p){ const i=fwTruth(p); const b=document.getElementById('fwBadge'); b.className=`fw ${i.sev}`; const ic=i.sev==='good'?'verified':i.sev==='bad'?'gpp_maybe':'help'; b.querySelector('.material-symbols-rounded').textContent=ic; b.querySelector('div div + div').innerHTML=`<span>sketch=${esc(i.sketch)}</span><span>module=${esc(i.moduleFw)}</span><span>module_valid=${esc(i.moduleValidText)}</span><span>schema=${esc(i.schemaHash)}</span>`; }
function normalize(r){ return { meta:r?.meta||{}, radar:r?.radar||{}, ble:r?.ble||{}, thresholds:r?.thresholds||{}, faults:Array.isArray(r?.faults)?r.faults:[], events:Array.isArray(r?.events)?r.events:[], series:r?.series||{}, analysis:r?.analysis||null }; }
function lastSeries(s,k){ const a=Array.isArray(s?.[k])?s[k]:[]; return a.length?a[a.length-1]:null; }
function getR(p,k){ const r=p.radar||{}; if(r[k]!==undefined) return r[k]; const d=lastSeries(p.series,k); return d!==null?d:null; }
function stageOr(p,prim,fb=[]){ const v=getR(p,prim); if(v!=null) return v; for(const k of fb){ const x=getR(p,k); if(x!=null) return x; } return null; }
function rrStage(p,prim,fb=[]){ if(!boolish(getR(p,'rr_anchor_fresh'))&&prim==='rr_post_kalman') return null; return stageOr(p,prim,fb); }
function sourceName(v){ return ({0:'none',1:'target_info',2:'point_cloud',3:'frame_0x0A17'}[Math.round(num(v)??0)]||textOrDash(v)); }
function phaseName(v){ return ({0:'absent',1:'warmup',2:'settling',3:'locked',4:'post_motion',5:'leaving'}[Math.round(num(v)??0)]||textOrDash(v)); }
function harmonicSum(v){ const n=Math.round(num(v)??0); if(!n) return 'none'; const t=[]; if(n&1) t.push('rr_raw'); if(n&2) t.push('hr_arb'); if(n&4) t.push('hr_half'); if(n&8) t.push('hr_amb'); if(n&16) t.push('rr_sub'); if(n&32) t.push('rr_harm'); return t.join('|'); }
function hrPathName(v,fb){ if(fb) return String(fb); const n=Math.round(num(v)??-999); return ({0:'none',1:'auto publish',2:'spectral path',3:'raw blend',4:'raw fallback',5:'raw only (no phase)',6:'direct raw fast path'})[n]||textOrDash(v); }
function hrAnchorName(v,fb){ if(fb) return String(fb); const n=Math.round(num(v)??-999); return ({0:'none',1:'trusted_phase',2:'tracking_raw',3:'latched_raw',4:'arbiter_anchor',5:'rejectphase_anchor'})[n]||textOrDash(v); }
function computeFps(p){ const e=num(getR(p,'fps_hz')); if(e!==null&&e>0) return e; const t=Array.isArray(p?.series?.t)?p.series.t.map(num).filter(v=>v!==null):[]; if(t.length<3) return null; const d=[]; for(let i=1;i<t.length;i++){ const x=t[i]-t[i-1]; if(x>0) d.push(x); } if(!d.length) return null; d.sort((a,b)=>a-b); const m=d[Math.floor(d.length/2)]; return m>0?1/m:null; }
function hardStale(p){ const age=num(p?.radar?.last_seen_age_s); return S.disc>=S.hardStaleN||(age!==null&&age>=S.hardStaleS); }
function bucketData(p){
  const a=p.analysis||{}; const ad=a.raw_hr_bias_audit||a.raw_hr_bias_estimate||{};
  const ab=Array.isArray(ad.rate_buckets)?ad.rate_buckets:[];
  if(ab.length){ return ab.filter(b=>num(b.bias_bpm)!==null&&num(b.n)!==null&&num(b.n)>0).map(b=>{ const lo=num(b.ref_hr_min),hi=num(b.ref_hr_max); const x=(lo!==null&&hi!==null)?((lo+hi)/2):num(b.ref_hr_mean); return {x:x??0,y:num(b.bias_bpm),bucket:textOrDash(b.bucket),count:num(b.n)}; }); }
  const s=p.series||{}; const raw=Array.isArray(s.raw_hr)?s.raw_hr:[]; const ref=Array.isArray(s.ref_hr)?s.ref_hr:(Array.isArray(s.ble_hr)?s.ble_hr:[]);
  const b=new Map(); const n=Math.min(raw.length,ref.length);
  for(let i=0;i<n;i++){ const rh=num(raw[raw.length-n+i]), rf=num(ref[ref.length-n+i]); if(rh===null||rf===null||rh<=0||rf<=0) continue; const k=Math.floor(rf/10)*10; const it=b.get(k)||{bucket:k,count:0,sum:0}; it.count++; it.sum+=rh-rf; b.set(k,it); }
  return [...b.values()].sort((a,b)=>a.bucket-b.bucket).map(i=>({x:i.bucket+5,y:i.sum/i.count,bucket:`${i.bucket}-${i.bucket+9}`,count:i.count}));
}

function setDensity(d){ document.documentElement.setAttribute('data-density',d); document.querySelectorAll('.seg button[data-density]').forEach(el=>el.classList.toggle('active', el.dataset.density===d)); localStorage.setItem('rvt-density',d); }
function toggleTheme(){ const h=document.documentElement; const t=h.getAttribute('data-theme'); const nt=t==='dark'?'light':'dark'; h.setAttribute('data-theme',nt); document.getElementById('themeIcon').textContent=nt==='dark'?'light_mode':'dark_mode'; localStorage.setItem('rvt-theme',nt); repaintCharts(); }
function repaintCharts(){ const dark=document.documentElement.getAttribute('data-theme')==='dark'; const g=dark?'rgba(255,255,255,.08)':'rgba(116,120,132,.12)'; const t=dark?'#c4c6d0':'#5b6070'; Object.values(S.charts).forEach(c=>{ if(!c||!c.options) return; if(c.options.scales) Object.values(c.options.scales).forEach(s=>{ if(s.ticks) s.ticks.color=t; if(s.grid) s.grid.color=g; if(s.title) s.title.color=t; }); if(c.options.plugins?.legend?.labels) c.options.plugins.legend.labels.color=t; c.update('none'); }); if(S.lastPayload) drawRadar(S.lastPayload,hardStale(S.lastPayload)); }
function togglePause(){ S.paused=!S.paused; document.getElementById('pauseIcon').textContent=S.paused?'play_arrow':'pause'; toast(S.paused?'Polling paused':'Polling resumed'); if(!S.paused) poll(); }
function switchTab(id){ document.querySelectorAll('.tab').forEach(e=>e.classList.remove('active')); document.querySelectorAll('.r-item:not(.nav-ctl)').forEach(e=>e.classList.remove('active')); document.getElementById(id)?.classList.add('active'); document.querySelector(`.r-item[data-target="${id}"]`)?.classList.add('active'); if(S.ctl.on) document.querySelector('.nav-ctl[data-view="live"]')?.classList.add('active'); Object.values(S.charts).forEach(c=>c&&c.resize()); if(S.lastPayload) drawRadar(S.lastPayload,hardStale(S.lastPayload)); if(id==='tab-snaps') renderSnaps(); }
function exportData(){ if(!S.lastPayload) return toast('No data to export'); const d="data:text/json;charset=utf-8,"+encodeURIComponent(JSON.stringify(S.lastPayload,null,2)); const a=document.createElement('a'); a.href=d; a.download=`radar_payload_${Date.now()}.json`; document.body.appendChild(a); a.click(); a.remove(); toast('Exported JSON payload'); }

function toast(m,i='check_circle'){ const h=document.getElementById('toasts'); const e=document.createElement('div'); e.className='toast'; e.innerHTML=`<span class="material-symbols-rounded" style="font-size:18px">${safeIcon(i)}</span>${esc(m)}`; h.appendChild(e); setTimeout(()=>e.remove(),3500); }

function helpLookup(key){
  const h=S.ctl.help||S.help||{};
  if(h.tooltips && h.tooltips[key]) return h.tooltips[key];
  const g=Array.isArray(h.glossary)?h.glossary.find(x=>x.id===key):null;
  if(g) return S.ctl.helpAdvanced ? (g.advanced||g.long||g.short) : (g.short||g.long||g.advanced);
  return '';
}
function installHelpTips(){
  if(document.getElementById('tipPop')) return;
  const pop=document.createElement('div'); pop.id='tipPop'; pop.className='tip-pop'; document.body.appendChild(pop);
  const move=(ev)=>{ if(pop.style.display==='block'){ const x=Math.min(window.innerWidth-340,ev.clientX+14); const y=Math.min(window.innerHeight-120,ev.clientY+14); pop.style.left=Math.max(8,x)+'px'; pop.style.top=Math.max(8,y)+'px'; } };
  const show=(el,ev)=>{ const key=el?.getAttribute('data-help'); const txt=key?helpLookup(key):''; if(!txt) return; pop.textContent=txt; pop.style.display='block'; if(ev) move(ev); };
  const hide=()=>{ pop.style.display='none'; };
  document.addEventListener('mouseover',ev=>{ const el=ev.target.closest?.('[data-help]'); if(el) show(el,ev); });
  document.addEventListener('mousemove',move);
  document.addEventListener('mouseout',ev=>{ if(ev.target.closest?.('[data-help]')) hide(); });
  document.addEventListener('focusin',ev=>{ const el=ev.target.closest?.('[data-help]'); if(el) show(el,{clientX:el.getBoundingClientRect().left,clientY:el.getBoundingClientRect().bottom}); });
  document.addEventListener('focusout',ev=>{ if(ev.target.closest?.('[data-help]')) hide(); });
}
// ─── BLE Validation ────────────────────────────────────────────────
function bleValidateMac(addr){
  if(!addr||typeof addr!=='string') return {ok:false,msg:'empty'};
  const s=addr.trim();
  if(/^[0-9A-Fa-f]{2}(:[0-9A-Fa-f]{2}){5}$/.test(s)) return {ok:true,msg:'valid MAC'};
  if(/^[0-9A-Fa-f]{8}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{4}-[0-9A-Fa-f]{12}$/i.test(s)) return {ok:true,msg:'valid UUID'};
  if(s.length>=12) return {ok:false,msg:'unusual format'};
  return {ok:false,msg:'too short'};
}
// ─── Setup Persistence ─────────────────────────────────────────────
function loadSetupPrefs(){
  try{
    const p=JSON.parse(localStorage.getItem('rvt-setup-prefs')||'{}');
    for(const k of ['radar_port','ble_address','ble_profile','notify_char','subject_label','operator_label']){
      if(p[k]) S.setup[k]=p[k];
    }
    if(p.duration_s) S.setup.duration_s=p.duration_s;
  }catch(_){}
}
function persistSetupPrefs(){
  try{ localStorage.setItem('rvt-setup-prefs',JSON.stringify({radar_port:S.setup.radar_port,ble_address:S.setup.ble_address,ble_profile:S.setup.ble_profile,notify_char:S.setup.notify_char,subject_label:S.setup.subject_label,operator_label:S.setup.operator_label,duration_s:S.setup.duration_s})); }catch(_){}
}
function updateRailOperator(){
  const cur=S.ctl.current;
  const name=(cur?.params?.operator_label||cur?.operator_label||cur?.operator||S.setup.operator_label||'Operator A').trim()||'Operator A';
  const initials=((name.match(/[A-Za-z0-9]+/g)||[]).slice(0,2).map(w=>w[0]).join('')||'OA').toUpperCase();
  const role=cur?.session_id?`Session ${cur.session_id}`:'Lab · Station 3';
  const avatar=document.getElementById('opAvatar'); if(avatar) avatar.textContent=initials.slice(0,2);
  const opName=document.getElementById('opName'); if(opName) opName.textContent=name;
  const opRole=document.getElementById('opRole'); if(opRole) opRole.textContent=role;
}
function durationGuidance(s){
  const n=Number(s); if(!n||n<=0) return '';
  if(n<30) return 'Very short — warmup alone is 10-15 s. Results may be deferred.';
  if(n<60) return 'Quick check — enough for settling but not full-lock coverage.';
  if(n<=300) return 'Standard — good for training runs and regression checks.';
  if(n<=600) return 'Extended — improves locked coverage and HR/RR statistics.';
  return 'Long session — strong ML-readiness signal if reference is stable.';
}
function copySessionId(){
  const sid=S.ctl.current?.session_id||S.ctl.lastSessionId; if(!sid) return;
  navigator.clipboard?.writeText(sid).then(()=>toast(`Copied ${sid}`,'content_copy')).catch(()=>{});
}
// ─── Sandbox Fallback System ───────────────────────────────────────
function sandboxStoreKey(){ return 'rvt-sandbox-sessions'; }
function sandboxApiPath(u){ const raw=String(u||''); if(raw.startsWith('/api/')) return raw; try{ const parsed=new URL(raw,location.href); if(location.protocol==='file:' && parsed.pathname.match(/^[\/][A-Za-z]:[\/]api[\/]/)) return parsed.pathname.replace(/^[\/][A-Za-z]:/,'').replace(/\\/g,'/'); return parsed.pathname; }catch(_){ return raw; } }
function sandboxId(){ return `sandbox_${new Date().toISOString().replace(/[-:T]/g,'').slice(0,15)}`; }
function sandboxSummary(item){
  item=item||{};
  return {
    session_id:item.session_id||'sandbox',started_at:item.started_at||new Date().toISOString(),duration_s:item.duration_s||30,subject_label:item.subject||'sandbox-subject',operator_label:item.operator||'sandbox-operator',verdict:item.verdict||'ready',status:'complete',trainer_version:'sandbox',dashboard_version:'v11.0-sandbox',
    ml_readiness_verdict:{verdict:item.verdict||'ready',readiness_kind:item.verdict||'ready',headline:`Session ${item.session_id||'sandbox'} verdict`,limitation_kind:'none',next_action:'review',passed:['Sandbox telemetry flowing','Schema fields present'],failed:[],categories:[{id:'sandbox',status:'pass',label:'Sandbox',detail:'Generated data; not a real measurement.'}]},
    hr_metrics:{rmse:2.14,mae:1.68,bias:0.42,r:0.962,coverage_pct:94.2,n:280},
    rr_metrics:{rmse:0.82,mae:0.61,bias:-0.18,r:0.943,coverage_pct:91.7,n:275},
    signal_quality:{pqi_lock_pct:88.4,coverage_locked:78,coverage_settling:22,session_quality_score:0.91,internal_consistency_score:0.87},
    reference_quality:{status:'good',raw_packets:340,parsed_rows:320,distilled_rows_pct_of_raw:94.1,packet_loss_pct:2.3,decode_error_pct:0.2,pi_median:3.1,pi_below_threshold_pct:1.8,pi_threshold:0.5,coverage_pct:94.1},
    gates:{primary:{status:'PASS',passed:true,r:0.962,rmse:2.14,n:310},secondary:{status:'PASS',passed:true,r:0.95,rmse:2.4,n:295},combined:{status:'PASS',passed:true,r:0.958,rmse:2.22,n:308},locked:{status:'PASS',passed:true,r:0.973,rmse:1.86,n:240},settling:{status:'PASS',passed:true,r:0.91,rmse:2.82,n:68},golden_check:{status:'PASS',passed:true,r:0.97,rmse:1.9,n:180}},
    histograms:{top_hr_gate:{pass:310,low_pqi:8},top_rr_gate:{pass:305,outlier:12},top_hr_publish:{published:320,STALE_FROZEN:4},top_rr_publish:{published:318,LATCH_STALE:3}},
    truthfulness:{sketch_fw:'v15.1.0',module_fw:'2.1.0',module_version_valid:true,contract_length:219,schema_hash:'sandbox-v15.1',scoring_weights_hash:'sandbox-weights'},
    downloads:[{label:'analyse_summary.json',href:'#sandbox-summary'},{label:'live_dashboard.json',href:'#sandbox-live'}],
    analysis:{
      raw_hr_correction_coverage_pct:95.1,raw_hr_corrected_bias_bpm:.6,raw_hr_uncorrected_bias_bpm:2.4,hr_rescue_publish_count:22,hr_rescue_publish_coverage_pct:8.1,
      oracle_audit:{hr_oracle_candidate_mae_bpm:1.8,hr_oracle_candidate_coverage_pct:82.4,rr_oracle_candidate_mae_bpm:.7,rr_oracle_candidate_coverage_pct:88.1,hr_best_candidate_source_histogram:{reported_hr:180,candidate_hr:90,raw_hr_corrected:120},rr_best_candidate_source_histogram:{reported_rr:210,candidate_rr:60,raw_rr_effective:40}},
      coverage_loss_ledger:{hr_coverage_loss_by_stage:{NO_PHASE:84,LOW_PQI:12},rr_coverage_loss_by_stage:{LOW_PQI:18,PUBLISH_POLICY:4},hr_salvageable_frames_pct:68.2,rr_salvageable_frames_pct:74.9}
    }
  };
}
function sandboxLoadSessions(){
  if(Array.isArray(S.ctl.sandboxItems)) return S.ctl.sandboxItems;
  let items=[];
  try{ items=JSON.parse(localStorage.getItem(sandboxStoreKey())||'[]'); }catch(_){ items=[]; }
  if(!Array.isArray(items)||!items.length){
    const iso=mins=>new Date(Date.now()-mins*60000).toISOString();
    items=[
      {session_id:'sandbox_20260420_091800',started_at:iso(64),duration_s:480,subject:'demo-A',operator:'codex',verdict:'ready'},
      {session_id:'sandbox_20260419_141803',started_at:iso(1240),duration_s:300,subject:'demo-B',operator:'codex',verdict:'conditional'}
    ];
    items=items.map(i=>({...i,summary:sandboxSummary(i)}));
  }
  S.ctl.sandboxItems=items;
  try{ localStorage.setItem(sandboxStoreKey(),JSON.stringify(items)); }catch(_){}
  return items;
}
function sandboxSaveSessions(items){
  S.ctl.sandboxItems=items;
  try{ localStorage.setItem(sandboxStoreKey(),JSON.stringify(items)); }catch(_){}
}
function sandboxDefaults(){
  return {radar_port:S.setup.radar_port||'COM10',ble_address:S.setup.ble_address||'10:22:33:9E:8F:63',ble_profile:S.setup.ble_profile||'ailink_oximeter',notify_char:S.setup.notify_char,durations_s:[30,60,300,480,1200],sessions_root:'local sandbox storage',sandbox:true};
}
function sandboxHelpSchema(){
  return {
    faq:[
      {q:'Why am I in sandbox mode?',a:'The trainer API did not answer /api/status, so the dashboard enabled a local functional preview. Real hardware is not touched.'},
      {q:'Can I still test Start and Stop?',a:'Yes. Sandbox Start creates a local session, drives live telemetry, and Stop creates a report and history row.'},
      {q:'What changes when the backend is running?',a:'The same UI calls the real /api endpoints automatically; the sandbox fallback is bypassed.'}
    ],
    glossary:[
      {id:'preflight',term:'Preflight',category:'setup',short:'A required/advisory check set before session start.',advanced:'Structural failures block Start. Probe warnings guide hardware cleanup but do not erase the workflow.'},
      {id:'stale',term:'Stale telemetry',category:'live',short:'The payload age exceeded the freshness threshold.',advanced:'Stale mode blanks live publish values and surfaces recovery actions so old measurements are not trusted.'},
      {id:'fw_truthfulness',term:'Firmware truthfulness',category:'audit',short:'A contract check between observed firmware/schema and expected analysis fields.'}
    ],
    dsp_steps:[
      {n:1,title:'Acquire',summary:'Read radar and BLE frames.',detail:'The trainer aligns radar publishes with BLE reference rows and records policy gates.'},
      {n:2,title:'Validate',summary:'Reject stale, low confidence, or motion-contaminated frames.',detail:'The dashboard preserves gate reasons so the operator sees why HR/RR did not publish.'},
      {n:3,title:'Report',summary:'Compute readiness and export artifacts.',detail:'Report summaries include metric gates, reference quality, truthfulness, and remediation.'}
    ],
    troubleshooting:[
      {id:'ble',title:'BLE device not found',steps:['Confirm oximeter is on and advertising.','Check MAC or UUID format.','Run Test link again from setup.']},
      {id:'placement',title:'Weak radar placement',steps:['Move subject to 40-80 cm.','Aim at the upper torso.','Clear nearby reflectors and ask subject to sit still.']},
      {id:'stale',title:'Telemetry stale',steps:['Confirm trainer is running.','Check serial port.','Use demo/sandbox only for UI review, not measurement decisions.']}
    ],
    tooltips:{
      'preflight.python_env':'Confirms the Python environment or local sandbox runtime is ready.',
      'preflight.serial_port_probe':'Checks whether the selected radar port can be opened.',
      'preflight.ble_device_probe':'Checks the BLE reference endpoint or simulates it in sandbox mode.',
      'fw_truthfulness':'Shows firmware, module, and schema identity used by the analysis contract.',
      'metric.rmse':'Root mean square error against the reference signal.',
      'metric.r':'Correlation against the reference signal.',
      'reference.coverage':'How much of the session had usable BLE reference rows.',
      'hist.gate':'Frames blocked before publish, grouped by reason.',
      'hist.publish':'Publish policy reasons visible for audit.'
    }
  };
}
function sandboxBody(opts){
  if(!opts||opts.body===undefined) return {};
  if(typeof opts.body==='string'){ try{return JSON.parse(opts.body);}catch(_){return {}; } }
  return opts.body||{};
}
function sandboxError(status,message,body){
  const e=new Error(message); e.status=status; e.body=body||{error:{message}}; throw e;
}
function sandboxPreflight(url){
  const u=new URL(url,location.href);
  const ids=(u.searchParams.get('include')||allCheckIds().join(',')).split(',').map(x=>x.trim()).filter(Boolean);
  const details={
    python_env:'Browser sandbox runtime ready.',
    firmware_file_present:'Firmware contract fixture loaded for UI review.',
    serial_port_list:"ports=['COM10','COM11','COM12']",
    session_folder_writable:'Session history writes to localStorage in sandbox mode.',
    disk_space:'Browser storage is available for sample sessions.',
    schema_hash_consistency:'Expected v11.0/v15.1 schema hash present.',
    clock_monotonic_sanity:'Local clock appears monotonic for this preview.',
    ble_adapter:'Sandbox adapter active; real BLE is not touched.',
    serial_port_probe:`Simulated radar probe succeeded for ${S.setup.radar_port||'COM10'}.`,
    ble_device_probe:bleValidateMac(S.setup.ble_address).ok?`Simulated BLE link accepted ${S.setup.ble_address}.`:'BLE address format is unusual; simulated probe allowed with warning.'
  };
  const checks=ids.map(id=>{
    const warn=id==='ble_device_probe'&&!bleValidateMac(S.setup.ble_address).ok;
    return {id,label:checkLabel(id),status:warn?'warn':'ok',detail:details[id]||'Sandbox check complete.',remediation:warn?'Enter a MAC address like AA:BB:CC:DD:EE:FF, then test again.':''};
  });
  const summary={ok:checks.filter(c=>c.status==='ok').length,warn:checks.filter(c=>c.status==='warn').length,fail:checks.filter(c=>c.status==='fail').length};
  return {summary,checks,sandbox:true,generated_at:new Date().toISOString()};
}
function sandboxCurrent(){
  const cur=S.ctl.current;
  if(!cur) sandboxError(404,'No active sandbox session');
  const elapsed=Math.max(0,(Date.now()-cur.started_ms)/1000);
  const dur=Number(cur.duration_s)||Number(cur.params?.duration_s)||30;
  return {...cur,elapsed_s:elapsed,remaining_s:Math.max(0,dur-elapsed),duration_s:dur,preview:{elapsed_s:elapsed},status:'running',sandbox:true};
}
function sandboxLivePayload(){
  const p=demoPayload();
  p.meta.sandbox=true;
  p.ble.address=S.setup.ble_address;
  p.ble.profile=S.setup.ble_profile;
  if(S.ctl.current){
    const cur=sandboxCurrent();
    p.meta.status='running'; p.meta.elapsed_s=cur.elapsed_s; p.meta.remaining_s=cur.remaining_s; p.meta.session_dir=`sandbox://${cur.session_id}`;
  }else{
    p.meta.status='waiting'; p.meta.elapsed_s=0; p.meta.remaining_s=null; p.meta.note='Sandbox preview: no active recording yet.';
  }
  return p;
}
function sandboxStart(opts){
  const body=sandboxBody(opts); const duration=Number(body.duration_s)||sessionDuration()||30; const sid=sandboxId();
  S.ctl.current={session_id:sid,started_at:new Date().toISOString(),started_ms:Date.now(),duration_s:duration,params:{...body,duration_s:duration},sandbox:true};
  S.ctl.lastSessionId=sid;
  return sandboxCurrent();
}
function sandboxStop(){
  if(!S.ctl.current) sandboxError(409,'No active sandbox session');
  const cur=sandboxCurrent();
  const item={session_id:cur.session_id,started_at:cur.started_at,duration_s:cur.duration_s,subject:cur.params?.subject_label||'sandbox-subject',operator:cur.params?.operator_label||'sandbox-operator',verdict:'ready'};
  item.summary=sandboxSummary(item);
  const items=sandboxLoadSessions().filter(i=>i.session_id!==item.session_id);
  items.unshift(item); sandboxSaveSessions(items);
  S.ctl.current=null; S.ctl.lastSessionId=item.session_id;
  return {session_id:item.session_id,status:'stopped',sandbox:true};
}
async function sandboxApiJson(url,opts={}){
  const path=sandboxApiPath(url);
  if(path==='/api/status') return {ok:true,mode:'sandbox',message:'Local dashboard sandbox active'};
  if(path==='/api/defaults') return sandboxDefaults();
  if(path==='/api/preflight') return sandboxPreflight(url);
  let m=path.match(/^\/api\/preflight\/([^/]+)$/);
  if(m) return sandboxPreflight(`/api/preflight?include=${encodeURIComponent(decodeURIComponent(m[1]))}`).checks[0];
  if(path==='/api/help/schema') return sandboxHelpSchema();
  if(path==='/api/session/start') return sandboxStart(opts);
  if(path==='/api/session/current') return sandboxCurrent();
  if(path==='/api/session/stop') return sandboxStop();
  if(path==='/api/sessions') return {items:sandboxLoadSessions().map(({summary,...i})=>i),sandbox:true};
  m=path.match(/^\/api\/sessions\/([^/]+)\/summary$/);
  if(m){ const id=decodeURIComponent(m[1]); const item=sandboxLoadSessions().find(i=>i.session_id===id) || (S.ctl.current?.session_id===id?{...S.ctl.current,subject:S.ctl.current.params?.subject_label,operator:S.ctl.current.params?.operator_label,verdict:'ready'}:null); if(!item) sandboxError(404,`Sandbox session ${id} not found`); return sandboxSummary(item); }
  m=path.match(/^\/api\/sessions\/([^/]+)\/compare$/);
  if(m){ const id=decodeURIComponent(m[1]); const items=sandboxLoadSessions(); const selected=items.find(i=>i.session_id===id)||items[0]; return {selected:sandboxSummary(selected),previous:sandboxSummary(items[1]||items[0]||selected),best:sandboxSummary(items.find(i=>i.verdict==='ready')||selected),sandbox:true}; }
  m=path.match(/^\/api\/sessions\/([^/]+)\/analyse$/);
  if(m) return {pid:'sandbox',status:'queued',sandbox:true};
  sandboxError(404,`Sandbox endpoint not implemented: ${path}`);
}
function enableSandboxControlMode(reason='Trainer API unavailable'){
  S.ctl={...S.ctl,on:true,sandbox:true,status:{ok:true,mode:'sandbox',reason}};
  S.ctl.help=sandboxHelpSchema();
  sandboxLoadSessions();
  return true;
}
async function apiJson(url,opts={}){
  if(S.ctl?.sandbox && String(url).startsWith('/api/')) return sandboxApiJson(url,opts);
  const r=await fetch(url,Object.assign({cache:'no-store'},opts));
  let j=null; try{ j=await r.json(); }catch(e){}
  if(!r.ok){ const err=new Error(j?.error?.message||j?.error||`HTTP ${r.status}`); err.status=r.status; err.body=j; throw err; }
  return j;
}
async function detectControlMode(){
  try{
    const r=await fetch('/api/status',{cache:'no-store'});
    if(!r.ok) return enableSandboxControlMode(`Control API returned HTTP ${r.status}`);
    const j=await r.json();
    S.ctl={...S.ctl,on:true,sandbox:false,status:j};
    return true;
  }catch(e){ return enableSandboxControlMode(e.message||'Control API unavailable'); }
}
function ctlButton(view,icon,label,kbd){
  return `<button class="r-item nav-ctl" data-view="${view}" type="button"><span class="r-ic"><span class="material-symbols-rounded">${icon}</span></span>${label}${kbd?`<span class="nav-kbd">${kbd}</span>`:''}</button>`;
}
function setupControlMode(){
  document.body.dataset.ctl='on';
  if(!S.ctl.sandbox) S.dataUrl='/api/session/current/live_dashboard.json';
  loadSetupPrefs();
  const rail=document.querySelector('.rail'); const workflow=document.getElementById('workflowGroup');
  if(rail && workflow && !document.querySelector('.nav-ctl')){
    workflow.insertAdjacentHTML('beforeend',[
      ctlButton('home','home','Home','1'), ctlButton('live','monitoring','Live','2'), ctlButton('report','summarize','Report','3'), ctlButton('help','help','Help','4')
    ].join(''));
    rail.querySelectorAll('.nav-ctl[data-view]').forEach(b=>b.addEventListener('click',()=>switchView(b.dataset.view)));
  }
  updateRailOperator();
  const top=document.querySelector('.topbar');
  if(top && !document.getElementById('ses-bar')){
    top.insertAdjacentHTML('afterend','<div class="ses-bar" id="ses-bar" style="display:none"><span class="ses-id" id="sesId">Session --</span><button class="ses-copy-btn" onclick="copySessionId()" title="Copy session ID"><span class="material-symbols-rounded">content_copy</span></button><div class="ses-meta"><span id="sesElapsed">elapsed --</span><span id="sesRemain">remaining --</span><span id="sesLabels"></span></div><div class="ses-progress" id="sesProgress"><div class="ses-progress-fill" id="sesProgressFill"></div></div><span class="ses-spacer"></span><button id="sesStopBtn" type="button" onclick="stopActiveSession()">Stop</button></div>');
  }
  const chips=document.getElementById('chips');
  if(chips && !document.getElementById('liveCoach')){
    chips.insertAdjacentHTML('afterend','<div class="coach-grid" id="liveCoach" style="display:none"></div>');
  }
  const fw=document.getElementById('fwBadge');
  if(fw && !document.getElementById('view-home')) fw.insertAdjacentHTML('beforebegin',controlViewsHtml());
  renderPreflight();
  installHelpTips();
  window.addEventListener('hashchange',routeHash);
  routeHash();
  loadControlDefaults().then(()=>Promise.allSettled([loadSessionsList(), runPreflightBatch(), renderHelpView()])).finally(()=>routeHash());
  refreshSessionHeader();
}
function controlViewsHtml(){
  const sb=S.ctl.sandbox;
  const modeNote=sb
    ? '<div class="sandbox-note"><strong>Sandbox note</strong>The trainer API was unreachable, so the dashboard is running in a local functional preview. All Start / Stop / Report actions are simulated in-browser. Data is stored in localStorage.</div>'
    : '<div class="sandbox-note connected"><strong>Connected server mode</strong>The trainer API is reachable. Start, Stop, preflight, reports, and live telemetry are backed by the Python control server.</div>';
  return `
  <section id="view-home">
    <div class="welcome-hero">
      <div class="eyebrow">${sb?'Sandbox mode — local preview':'Connected server mode — live trainer'}</div>
      <h1 class="hero-title">Radar Vital Trainer</h1>
      <p class="hero-subtitle">Configure hardware, run preflight checks, record sessions, and review ML-readiness verdicts — all from one dashboard-first console.</p>
      ${modeNote}
    </div>
    <div class="home-layout">
      <div class="page-container">
        <article class="card" style="margin-bottom:var(--gap)" id="homeWelcome">
          <div class="ch"><div class="tw"><span class="material-symbols-rounded">fiber_manual_record</span><div><h2 class="ct">Record a new session</h2><div class="cs">Dashboard-first setup with truthful preflight checks before collection.</div></div></div></div>
          <div class="chips" id="lastSessionChip" style="margin-bottom:14px"></div>
          <div class="ca"><button class="fab" type="button" onclick="document.getElementById('setupCard')?.scrollIntoView({behavior:'smooth'})"><span class="material-symbols-rounded">play_arrow</span>Start new session</button><button class="chip-btn" type="button" onclick="switchView('live')">Open live dashboard</button><button class="chip-btn" type="button" onclick="switchView('help')">Open help</button></div>
        </article>
        <article class="card" style="margin-bottom:var(--gap)" id="setupCard">
          <div class="ch"><div class="tw"><span class="material-symbols-rounded">tune</span><div><h2 class="ct">Session setup</h2><div class="cs">Defaults use ${esc(S.setup.radar_port)} and ${esc(S.setup.ble_address)} unless changed.</div></div></div></div>
          <div class="setup">
            <div><div class="m-l" style="margin-bottom:8px">Duration</div><div class="chip-group" id="durationChips"></div><div class="setup-help" id="durationHint"></div></div>
            <div class="setup-row"><label for="radarPort">Radar port</label><select id="radarPort" onchange="updateSetupFromForm(true)"></select></div>
            <div class="setup-row"><label for="bleAddress">BLE address</label><div style="display:flex;gap:8px;flex-wrap:wrap"><input id="bleAddress" type="text" style="min-width:250px" oninput="updateSetupFromForm(true)"><button class="chip-btn" type="button" onclick="runOnePreflight('ble_device_probe')">Test</button><span id="bleInline" class="cs"></span></div></div>
            <div class="setup-row"><label for="subjectLabel">Subject</label><input id="subjectLabel" type="text" oninput="updateSetupFromForm(false)" placeholder="optional"></div>
            <div class="setup-row"><label for="operatorLabel">Operator</label><input id="operatorLabel" type="text" oninput="updateSetupFromForm(false)" placeholder="optional"></div>
            <details class="advanced-section"><summary>Advanced</summary><div class="adv-grid"><div class="setup-row"><label for="notifyChar">Notify UUID</label><input id="notifyChar" type="text" oninput="updateSetupFromForm(false)"></div><div class="setup-row"><label for="bleProfile">BLE profile</label><input id="bleProfile" type="text" oninput="updateSetupFromForm(false)"></div><div class="setup-row"><label for="skipCountdown">Stillness countdown</label><label class="cs"><input id="skipCountdown" type="checkbox" onchange="updateSetupFromForm(false)"> Skip 3-second countdown</label></div><div class="setup-row"><span>Sessions root</span><span class="cs" id="sessionsRoot">sessions</span></div></div></details>
            <div id="startAlert" class="setup-alert" style="display:none"></div>
            <button class="setup-start" id="startBtn" type="button" onclick="startSession()">Start session</button>
          </div>
        </article>
        <article class="card"><div class="ch"><div class="tw"><span class="material-symbols-rounded">history</span><div><h2 class="ct">Past sessions</h2><div class="cs">Open a previous report summary.</div></div></div></div><div class="sess-filter"><input class="help-search" type="search" id="sessSearch" placeholder="Filter sessions" oninput="renderSessionsFilter()" style="min-width:200px;max-width:340px"><span class="count" id="sessCount"></span></div><div id="sessionsList"></div></article>
      </div>
      <div>
        <article class="sys-card sys-preflight" style="margin-bottom:var(--gap)">
          <div class="card-header space-between"><div class="title-with-icon"><span class="material-symbols-rounded" style="font-size:23px;color:#fff;background:linear-gradient(135deg,var(--brand-500),var(--brand-400));padding:10px;border-radius:var(--r-md)">fact_check</span><h3>Hardware quickcheck</h3></div><button class="ghost-btn" type="button" onclick="runPreflightBatch()">Re-run all</button></div>
          <div class="pf-grid" id="pfGrid"></div>
          <div class="chips" id="pfSummary" style="margin-top:12px"></div>
        </article>
      </div>
    </div>
  </section>
  <section id="view-report"><article class="card"><div class="ch"><div class="tw"><span class="material-symbols-rounded">summarize</span><div><h2 class="ct">Report</h2><div class="cs" id="reportSub">Select a session from Home.</div></div></div></div><div id="reportBody">No session selected.</div></article></section>
  <section id="view-help"><article class="card"><div class="ch"><div class="tw"><span class="material-symbols-rounded">help</span><div><h2 class="ct">Help</h2><div class="cs">FAQ, glossary, DSP notes, and troubleshooting from the trainer.</div></div></div></div><div class="help-toolbar"><input id="helpSearch" class="help-search" type="search" placeholder="Search help" oninput="S.ctl.helpQuery=this.value;renderHelpView()"><button class="chip-btn" id="helpModeBtn" type="button" onclick="toggleHelpAdvanced()">Beginner</button></div><div id="helpBody">Loading help...</div></article></section>`;
}
function switchView(id,push=true){
  if(!S.ctl.on) return;
  const view=['home','live','report','help'].includes(id)?id:'home';
  document.body.dataset.view=view;
  document.querySelectorAll('.nav-ctl[data-view]').forEach(b=>b.classList.toggle('active',b.dataset.view===view));
  if(view==='live') document.querySelector('.nav-ctl[data-view="live"]')?.classList.add('active');
  if(push){
    const h=view==='report'&&S.ctl.lastSessionId?`#/report/${S.ctl.lastSessionId}`:`#/${view}`;
    if(location.hash!==h) history.replaceState(null,'',h);
  }
  if(view==='help') renderHelpView();
  if(view==='report') renderReportView(S.ctl.lastSessionId);
  Object.values(S.charts).forEach(c=>c&&c.resize());
}
function routeHash(){
  if(!S.ctl.on) return;
  const parts=location.hash.replace(/^#\/?/,'').split('/').filter(Boolean);
  if(!parts.length){ switchView('home',false); return; }
  if(parts[0]==='live'){ switchView('live',false); if(parts[1]) switchTab(parts[1].startsWith('tab-')?parts[1]:`tab-${parts[1]}`); return; }
  if(parts[0]==='report'){ S.ctl.lastSessionId=parts[1]||S.ctl.lastSessionId; switchView('report',false); renderReportView(S.ctl.lastSessionId); return; }
  switchView(parts[0],false);
}
async function loadControlDefaults(){
  try{
    const d=await apiJson('/api/defaults'); S.ctl.defaults=d;
    S.setup.radar_port=d.radar_port||S.setup.radar_port; S.setup.ble_address=d.ble_address||S.setup.ble_address; S.setup.ble_profile=d.ble_profile||S.setup.ble_profile; S.setup.notify_char=d.notify_char||S.setup.notify_char; S.setup.duration_s=(Array.isArray(d.durations_s)&&d.durations_s[0])||30;
    renderSetupForm();
  }catch(e){ toast(`Defaults unavailable: ${e.message}`,'warning'); renderSetupForm(); }
}
function durationLabel(s){ const n=Number(s); if(n===30) return '30 s'; if(n%60===0) return `${n/60} min`; return `${n} s`; }
function renderSetupForm(){
  const ds=(S.ctl.defaults?.durations_s||[30,60,300,480,1200]).map(Number);
  const dc=document.getElementById('durationChips'); if(dc) dc.innerHTML=ds.map(s=>`<button class="chip-btn ${S.setup.duration_s===s?'active':''}" type="button" onclick="setDuration(${s})">${esc(durationLabel(s))}</button>`).join('')+`<button class="chip-btn ${S.setup.duration_s==='custom'?'active':''}" type="button" onclick="setDuration('custom')">Custom</button><span id="customDurationWrap" style="${S.setup.duration_s==='custom'?'':'display:none'}"><input id="customDuration" type="number" min="1" value="${esc(S.setup.customDuration)}" oninput="updateSetupFromForm(false)" style="width:90px"><select id="customUnit" onchange="updateSetupFromForm(false)"><option value="s">s</option><option value="min">min</option></select></span>`;
  const port=document.getElementById('radarPort'); if(port){ const ports=detectedPorts(); if(!ports.includes(S.setup.radar_port)) ports.unshift(S.setup.radar_port); port.innerHTML=ports.map(p=>`<option value="${esc(p)}">${esc(p)}</option>`).join(''); port.value=S.setup.radar_port; }
  for(const [id,key] of [['bleAddress','ble_address'],['subjectLabel','subject_label'],['operatorLabel','operator_label'],['notifyChar','notify_char'],['bleProfile','ble_profile']]){ const el=document.getElementById(id); if(el) el.value=S.setup[key]||''; }
  const skip=document.getElementById('skipCountdown'); if(skip) skip.checked=!!S.setup.skip_countdown;
  const root=document.getElementById('sessionsRoot'); if(root) root.textContent=S.ctl.defaults?.sessions_root||'sessions';
  updateStartGate();
  const dh=document.getElementById('durationHint'); if(dh){ const dur=sessionDuration(); dh.textContent=durationGuidance(dur); dh.className='setup-help'+(dur>=300?' ok':dur<30?' warn':''); }
  const addr=document.getElementById('bleAddress'); if(addr){ const v=bleValidateMac(addr.value); if(addr.value.trim()) addr.classList.toggle('invalid-soft',!v.ok); else addr.classList.remove('invalid-soft'); }
}
function setDuration(v){ S.setup.duration_s=v; renderSetupForm(); }
let pfTimer=null;
function updateSetupFromForm(debounce){
  const port=document.getElementById('radarPort'); const addr=document.getElementById('bleAddress');
  if(port) S.setup.radar_port=port.value||'COM10'; if(addr) S.setup.ble_address=addr.value||'10:22:33:9E:8F:63';
  for(const [id,key] of [['subjectLabel','subject_label'],['operatorLabel','operator_label'],['notifyChar','notify_char'],['bleProfile','ble_profile']]){ const el=document.getElementById(id); if(el) S.setup[key]=el.value; }
  const skip=document.getElementById('skipCountdown'); if(skip) S.setup.skip_countdown=!!skip.checked;
  const cd=document.getElementById('customDuration'); const cu=document.getElementById('customUnit'); if(cd) S.setup.customDuration=Number(cd.value)||1; if(cu) S.setup.customUnit=cu.value;
  if(debounce){ clearTimeout(pfTimer); pfTimer=setTimeout(runPreflightBatch,450); }
  persistSetupPrefs();
  updateRailOperator();
  updateStartGate();
}
function renderSessionsFilter(){
  const q=document.getElementById('sessSearch')?.value?.trim().toLowerCase()||'';
  const rows=document.querySelectorAll('#sessionsList .sess-table tbody tr');
  let shown=0;
  rows.forEach(tr=>{ const match=!q||tr.textContent.toLowerCase().includes(q); tr.style.display=match?'':'none'; if(match) shown++; });
  const ct=document.getElementById('sessCount'); if(ct) ct.textContent=`${shown} of ${rows.length}`;
}
function structuralChecks(){ return ['python_env','firmware_file_present','serial_port_list','session_folder_writable','disk_space','schema_hash_consistency','clock_monotonic_sanity','ble_adapter']; }
function allCheckIds(){ return [...structuralChecks(),'serial_port_probe','ble_device_probe']; }
function checkLabel(id){ return ({python_env:'Python environment',firmware_file_present:'Firmware file',serial_port_list:'Serial ports',session_folder_writable:'Session folder',disk_space:'Disk space',schema_hash_consistency:'Schema hash',clock_monotonic_sanity:'Clock sanity',ble_adapter:'BLE adapter',serial_port_probe:'Serial probe',ble_device_probe:'BLE device'})[id]||id; }
function statusClass(s){ return s==='ok'?'good':(s==='fail'?'bad':(s==='warn'?'warn':'')); }
function renderPreflight(){
  const checks=new Map((S.ctl.preflight?.checks||[]).map(c=>[c.id,c]));
  const html=allCheckIds().map(id=>{
    const c=checks.get(id)||{id,label:checkLabel(id),status:'skip',detail:id.endsWith('_probe')?'Not run in batch. Use Test now.':'Not run yet.',remediation:''};
    const cls=statusClass(c.status); const action=id.endsWith('_probe')?'<button type="button">Test now</button>':'<button type="button">Retry</button>';
    return `<div class="pf-tile ${c.status==='fail'?'fail':c.status==='warn'?'warn':''}" data-check="${esc(id)}" data-help="preflight.${esc(id)}" tabindex="0"><div class="pf-top"><span class="pill ${cls}">${esc(c.status||'skip')}</span><span>${esc(c.label||checkLabel(id))}</span></div><div class="pf-detail">${esc(c.detail)}</div>${c.remediation?`<div class="pf-rem">${esc(c.remediation)}</div>`:''}${action}</div>`;
  }).join('');
  const grid=document.getElementById('pfGrid'); if(grid){ grid.innerHTML=html; grid.querySelectorAll('.pf-tile button').forEach(b=>b.addEventListener('click',()=>runOnePreflight(b.closest('.pf-tile').dataset.check))); }
  const sum=S.ctl.preflight?.summary||{}; const ps=document.getElementById('pfSummary'); if(ps) ps.innerHTML=`<span class="pill good">ok ${sum.ok||0}</span><span class="pill warn">warn ${sum.warn||0}</span><span class="pill bad">fail ${sum.fail||0}</span>`;
  renderSetupForm();
}
async function runPreflightBatch(){
  if(!S.ctl.on) return;
  const include=encodeURIComponent(structuralChecks().join(','));
  try{ S.ctl.preflight=await apiJson(`/api/preflight?port=${encodeURIComponent(S.setup.radar_port)}&address=${encodeURIComponent(S.setup.ble_address)}&include=${include}`); }
  catch(e){ S.ctl.preflight={summary:{fail:1},checks:[{id:'api',label:'Preflight API',status:'fail',detail:e.message,remediation:'Confirm the control server is running.'}]}; }
  renderPreflight();
}
async function runOnePreflight(id){
  const grid=document.getElementById('pfGrid'); const tile=grid?.querySelector(`[data-check="${CSS.escape(id)}"]`); if(tile) tile.querySelector('.pf-detail').textContent='Running...';
  try{
    const c=await apiJson(`/api/preflight/${encodeURIComponent(id)}`,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({port:S.setup.radar_port,address:S.setup.ble_address})});
    const checks=(S.ctl.preflight?.checks||[]).filter(x=>x.id!==id); checks.push(c); S.ctl.preflight={summary:S.ctl.preflight?.summary||{},checks};
    if(id==='ble_device_probe'){ const el=document.getElementById('bleInline'); if(el) el.textContent=c.detail||''; }
  }catch(e){
    const checks=(S.ctl.preflight?.checks||[]).filter(x=>x.id!==id); checks.push({id,label:checkLabel(id),status:'fail',detail:e.status===409?'Stopped - a session is running':e.message,remediation:e.body?.error?.message||''}); S.ctl.preflight={summary:S.ctl.preflight?.summary||{},checks};
  }
  renderPreflight();
}
function detectedPorts(){
  const c=(S.ctl.preflight?.checks||[]).find(x=>x.id==='serial_port_list'); const d=String(c?.detail||'');
  const m=d.match(/ports=\[(.*)\]/); if(!m) return [];
  return m[1].split(',').map(s=>s.replace(/['"]/g,'').trim()).filter(Boolean);
}
function structuralFailures(){ const checks=S.ctl.preflight?.checks||[]; const ids=new Set(structuralChecks()); return checks.filter(c=>ids.has(c.id)&&c.status==='fail'); }
function updateStartGate(){
  const btn=document.getElementById('startBtn'); if(!btn) return;
  const fails=structuralFailures(); btn.disabled=!!S.ctl.current||fails.length>0;
  const alert=document.getElementById('startAlert');
  if(alert && fails.length){ alert.style.display='block'; alert.innerHTML=fails.map(f=>`${esc(f.label||f.id)}: ${esc(f.remediation||f.detail)}`).join('<br>'); }
  else if(alert && !btn.dataset.serverError){ alert.style.display='none'; alert.textContent=''; }
}
function sessionDuration(){
  if(S.setup.duration_s!=='custom') return Number(S.setup.duration_s)||30;
  return Math.max(1,Number(S.setup.customDuration)||1)*(S.setup.customUnit==='min'?60:1);
}
function sleep(ms){ return new Promise(resolve=>setTimeout(resolve,ms)); }
async function stillnessCountdown(){
  if(S.setup.skip_countdown) return;
  const alert=document.getElementById('startAlert');
  if(!alert) return;
  alert.dataset.serverError='';
  alert.style.display='block';
  for(let n=3;n>=1;n--){
    alert.textContent=`Stillness countdown: ${n}. Sit still, oximeter on finger, radar aimed at the chest.`;
    await sleep(1000);
  }
  alert.textContent='Starting session...';
}
async function startSession(){
  updateSetupFromForm(false);
  const btn=document.getElementById('startBtn'); const alert=document.getElementById('startAlert'); if(btn) btn.disabled=true;
  if(alert){ alert.dataset.serverError=''; alert.style.display='none'; alert.textContent=''; }
  try{
    await stillnessCountdown();
    const body={duration_s:sessionDuration(),radar_port:S.setup.radar_port,ble_address:S.setup.ble_address,ble_profile:S.setup.ble_profile,subject_label:S.setup.subject_label,operator_label:S.setup.operator_label,advanced:{notify_char:S.setup.notify_char}};
    const r=await apiJson('/api/session/start',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
    S.ctl.current=r; toast(`Started ${r.session_id}`,'play_arrow'); switchView('live'); refreshSessionHeader();
  }catch(e){
    const failed=e.body?.error?.failed||[]; if(alert){ alert.dataset.serverError='1'; alert.style.display='block'; alert.innerHTML=failed.length?failed.map(f=>`${esc(f.label||f.id)}: ${esc(f.remediation||f.detail)}`).join('<br>'):esc(e.message); }
    if(failed.length){ const checks=(S.ctl.preflight?.checks||[]).filter(c=>!failed.some(f=>f.id===c.id)).concat(failed.map(f=>({...f,status:'fail'}))); S.ctl.preflight={summary:S.ctl.preflight?.summary||{},checks}; renderPreflight(); }
  }finally{ updateStartGate(); }
}
async function refreshSessionHeader(){
  if(!S.ctl.on) return;
  try{
    const cur=await apiJson('/api/session/current'); S.ctl.current=cur;
    const bar=document.getElementById('ses-bar'); if(bar) bar.style.display='flex';
    const sid=cur.session_id||'--'; document.getElementById('sesId').textContent=`Session ${sid}`;
    const elapsed=num(cur.elapsed_s??cur.preview?.elapsed_s); const remain=num(cur.remaining_s);
    document.getElementById('sesElapsed').textContent=`elapsed ${elapsed===null?'--':fmt(elapsed,0)+' s'}`;
    document.getElementById('sesRemain').textContent=`remaining ${remain===null?'--':fmt(remain,0)+' s'}`;
    document.getElementById('sesLabels').textContent=[cur.params?.subject_label,cur.params?.operator_label].filter(Boolean).join(' / ');
    const dur=num(cur.duration_s); const pct=(elapsed!==null&&dur>0)?Math.min(100,(elapsed/dur)*100):0;
    const fill=document.getElementById('sesProgressFill'); if(fill) fill.style.width=pct+'%';
    const prog=document.getElementById('sesProgress'); if(prog){ prog.dataset.manual=dur<=0?'1':'0'; prog.dataset.almost=pct>=90?'1':'0'; }
    updateRailOperator();
  }catch(e){
    const was=S.ctl.current; S.ctl.current=null; const bar=document.getElementById('ses-bar'); if(bar) bar.style.display='none';
    updateRailOperator();
    if(S.ctl.stopPending && was?.session_id){ S.ctl.stopPending=false; S.ctl.lastSessionId=was.session_id; switchView('report'); renderReportView(was.session_id); }
  }
  updateStartGate();
}
async function stopActiveSession(){
  const btn=document.getElementById('sesStopBtn'); if(btn) btn.disabled=true; S.ctl.stopPending=true;
  try{ const r=await apiJson('/api/session/stop',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({reason:'user_request'})}); if(r.session_id) S.ctl.lastSessionId=r.session_id; toast('Stop requested','stop_circle'); }
  catch(e){ S.ctl.stopPending=false; toast(e.message,'warning'); }
  finally{ if(btn) btn.disabled=false; refreshSessionHeader(); }
}
async function loadSessionsList(){
  try{
    const s=await apiJson('/api/sessions'); const items=Array.isArray(s.items)?s.items:[];
    items.sort((a,b)=>String(b.started_at||'').localeCompare(String(a.started_at||'')));
    const chip=document.getElementById('lastSessionChip'); if(chip){ const last=items[0]; chip.innerHTML=last?`<button class="chip good" type="button" onclick="openReport('${esc(last.session_id)}')">Last session: ${esc(last.session_id)} • ${esc(durationLabel(last.duration_s||0))} • ${esc(last.verdict||'unknown')}</button>`:'<span class="chip">No past sessions yet</span>'; }
    const el=document.getElementById('sessionsList'); if(el) el.innerHTML=items.length?`<table class="sess-table"><thead><tr><th>id</th><th>started</th><th>duration</th><th>subject</th><th>verdict</th></tr></thead><tbody>${items.map(i=>`<tr onclick="openReport('${esc(i.session_id)}')"><td>${esc(i.session_id)}</td><td>${esc(i.started_at)}</td><td>${esc(durationLabel(i.duration_s||0))}</td><td>${esc(i.subject||'')}</td><td><span class="pill ${i.verdict==='ready'?'good':i.verdict==='not_ready'?'bad':'warn'}">${esc(i.verdict||'unknown')}</span></td></tr>`).join('')}</tbody></table>`:'No sessions found.';
  }catch(e){ const el=document.getElementById('sessionsList'); if(el) el.textContent=e.message; }
}
function openReport(id){ S.ctl.lastSessionId=id; location.hash=`#/report/${id}`; }
async function renderReportView(id){
  const body=document.getElementById('reportBody'); if(!body) return;
  if(!id){ body.textContent='No session selected.'; return; }
  body.textContent='Loading report...';
  try{
    const s=await apiJson(`/api/sessions/${encodeURIComponent(id)}/summary`);
    renderReportSummary(s,id,body);
  }catch(e){ body.textContent=e.message; }
}
function reportPillLegacy(v){ const c=v==='ready'?'good':(v==='not_ready'?'bad':(v==='conditional'?'warn':'')); return `<span class="pill ${c}">${esc(v||'not computed')}</span>`; }
function reportMetric(label,value,unit='',d=1){ return `<div class="m"><span class="m-l">${esc(label)}</span><div class="m-vr"><span class="m-v">${esc(value===null||value===undefined||value===''?'--':(typeof value==='number'?fmt(value,d):value))}</span>${unit?`<span class="m-u">${esc(unit)}</span>`:''}</div></div>`; }
function metricRows(prefix,m){ m=m||{}; return [
  row(`${prefix} RMSE`,fmt(m.rmse,2)), row(`${prefix} MAE`,fmt(m.mae,2)), row(`${prefix} bias`,fmt(m.bias,2)),
  row(`${prefix} r`,fmt(m.r,3)), row(`${prefix} coverage`,`${fmt(m.coverage_pct,1)}%`), row(`${prefix} n`,textOrDash(m.n))
].join(''); }
function histBars(items){
  if(!items) return '<div class="cs">No histogram entries.</div>';
  const list=Array.isArray(items)?items:(typeof items==='object'?Object.entries(items).map(([reason,count])=>({reason,count})):[]);
  if(!list.length) return '<div class="cs">No histogram entries.</div>';
  const max=Math.max(...list.map(i=>Number(i.count??i.value??0)||0),1);
  return `<div class="mini-bars">${list.map(i=>`<div class="mini-bar"><span>${esc(i.reason??i.label??i.stage??i.kind??'unknown')}</span><span class="mini-bar-track"><span class="mini-bar-fill" style="width:${Math.max(2,(Number(i.count??i.value??0)||0)/max*100)}%"></span></span><span>${esc(i.count??i.value??0)}</span></div>`).join('')}</div>`;
}
function readinessMeta(kind){
  const k=String(kind||'').trim().toLowerCase();
  const map={
    ready:{label:'ready',icon:'verified',cls:'ready'},
    degraded_signal:{label:'degraded signal',icon:'report',cls:'degraded_signal'},
    provenance_warning:{label:'provenance warning',icon:'gpp_maybe',cls:'provenance_warning'},
    schema_warning:{label:'schema warning',icon:'rule',cls:'schema_warning'},
    firmware_rejected:{label:'firmware rejected',icon:'error',cls:'firmware_rejected'},
    deferred:{label:'deferred',icon:'schedule',cls:'deferred'},
    not_ready:{label:'not ready',icon:'error',cls:'firmware_rejected'},
    conditional:{label:'conditional',icon:'warning',cls:'degraded_signal'}
  };
  return map[k]||{label:k||'unknown',icon:'help',cls:'degraded_signal'};
}
function verdictKindClass(v){ return readinessMeta(v?.readiness_kind||v?.verdict||v).cls; }
function reportPill(v){
  const meta=readinessMeta(v?.readiness_kind||v?.verdict||v);
  return `<span class="pill ${meta.cls}"><span class="material-symbols-rounded">${meta.icon}</span>${esc(meta.label)}</span>`;
}
function verdictActionHtml(v){
  v=v||{}; const kind=String(v.limitation_kind||'').toLowerCase(); const next=String(v.next_action||'').toLowerCase();
  const readiness=String(v.readiness_kind||v.verdict||'').toLowerCase();
  if(readiness==='ready') return `<div class="verdict-actions"><a class="chip" href="/api/report/export?session=${encodeURIComponent(S.ctl.lastSessionId||'')}">Export thesis HTML</a></div>`;
  if(readiness==='deferred') return '<div class="verdict-actions"><button class="chip-btn" type="button" onclick="prepareNextSession(1200)">Start a longer session</button></div>';
  if(kind==='data'||next.includes('longer')||next.includes('20 min')) return '<div class="verdict-actions"><button class="chip-btn" type="button" onclick="prepareNextSession(1200)">Start a longer session</button></div>';
  if(kind==='hygiene') return `<div class="verdict-actions"><button class="chip-btn" type="button" onclick="switchView('live')">Open live coach</button><button class="chip-btn" type="button" onclick="location.hash='#/help';S.ctl.helpQuery='radar placement';renderHelpView()">Placement help</button></div>`;
  if(kind==='reference') return `<div class="verdict-actions"><button class="chip-btn" type="button" onclick="location.hash='#/help';S.ctl.helpQuery='BLE device';renderHelpView()">BLE troubleshooting</button></div>`;
  if(kind==='policy') return '<div class="verdict-actions"><button class="chip-btn" type="button" onclick="prepareNextSession(1200)">Extend warmup/run time</button></div>';
  return `<div class="verdict-actions"><button class="chip-btn" type="button" onclick="switchView('home')">Start a new session</button></div>`;
}
function prepareNextSession(seconds){
  S.setup.duration_s=Number(seconds)||1200;
  switchView('home');
  renderSetupForm();
  setTimeout(()=>document.getElementById('setupCard')?.scrollIntoView({behavior:'smooth'}),50);
}
function contractDiagnosisHtml(d){
  d=d||{}; const bad=d.status==='mismatch'; const unknown=d.status==='unknown';
  const rows=[row('Expected columns',d.expected_contract_length),row('Observed columns',d.observed_contract_length),row('Expected FW',d.expected_firmware),row('Observed FW',d.observed_firmware),row('Expected schema',shortHash(d.expected_schema_hash)),row('Observed schema',shortHash(d.observed_schema_hash))].join('');
  const issues=(d.mismatches||[]).map(m=>flt('bad',m.field||'mismatch',`expected ${textOrDash(m.expected)} / actual ${textOrDash(m.actual)}. ${m.remediation||''}`)).join('');
  return `<article class="card" style="margin-top:var(--gap)" data-help="fw_truthfulness"><div class="ch"><div class="tw"><span class="material-symbols-rounded">${bad?'gpp_maybe':unknown?'help':'verified_user'}</span><div><h2 class="ct">Contract diagnosis</h2><div class="cs">Expected-vs-observed firmware/schema contract.</div></div></div><span class="pill ${bad?'bad':unknown?'warn':'good'}">${esc(d.status||'unknown')}</span></div><table><tbody>${rows}</tbody></table><div class="contract-list">${issues||(unknown?flt('warn','Analysis unavailable','Contract diagnosis needs analyse_summary.json.'):flt('good','No detected mismatch','Observed fields match the expected v15/v11 contract.'))}</div></article>`;
}
function compareMetricBlock(label,s){
  s=s||{}; return `<div class="compare-card"><h3>${esc(label)}</h3><table><tbody>${[row('Session',s.session_id),row('Verdict',s.verdict),row('HR RMSE',fmt(s.hr_rmse??s.hr_metrics?.rmse,2)),row('HR r',fmt(s.hr_r??s.hr_metrics?.r,3)),row('RR RMSE',fmt(s.rr_rmse??s.rr_metrics?.rmse,2)),row('BLE coverage',fmt(s.reference_quality?.distilled_rows_pct_of_raw??s.reference_quality?.coverage_pct??s.ref_coverage_pct,1))].join('')}</tbody></table></div>`;
}
async function renderComparePanel(id){
  const el=document.getElementById('comparePanel'); if(!el) return;
  try{
    const c=await apiJson(`/api/sessions/${encodeURIComponent(id)}/compare`);
    el.innerHTML=`<div class="ch"><div class="tw"><span class="material-symbols-rounded">compare_arrows</span><div><h2 class="ct">Compare</h2><div class="cs">Selected session against previous and best available session.</div></div></div></div><div class="compare-grid">${compareMetricBlock('Selected',c.selected)}${compareMetricBlock('Previous',c.previous)}${compareMetricBlock('Best',c.best)}</div>`;
  }catch(e){
    el.innerHTML=`<div class="ch"><div class="tw"><span class="material-symbols-rounded">compare_arrows</span><div><h2 class="ct">Compare</h2><div class="cs">${esc(e.message)}</div></div></div></div>`;
  }
}
async function rerunAnalysis(id){
  try{ const r=await apiJson(`/api/sessions/${encodeURIComponent(id)}/analyse`,{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'}); toast(`Analysis rerun started (pid ${r.pid||'--'})`,'play_arrow'); }
  catch(e){ toast(e.message,'warning'); }
}
function verdictCard(v){
  v=v||{}; const kind=String(v.readiness_kind||v.verdict||'not_ready').toLowerCase(); const meta=readinessMeta(kind);
  const passed=(v.passed||[]).map(x=>`<div class="flt good"><div class="flt-t"><span class="material-symbols-rounded">check_circle</span>${esc(x)}</div></div>`).join('');
  const failed=(v.failed||[]).map(x=>`<div class="flt bad"><div class="flt-t"><span class="material-symbols-rounded">error</span>${esc(x)}</div></div>`).join('');
   const cats=(v.categories||[]).map(c=>`<div data-help="verdict.${esc(kind)}">${flt(c.status==='pass'?'good':c.status==='warn'?'warn':'bad',c.label||c.id,`${c.detail||''}${c.remediation?' · '+c.remediation:''}`)}</div>`).join('');
  return `<section class="verdict-card ${esc(meta.cls)}" data-help="verdict.${esc(kind)}" tabindex="0"><div class="verdict-head"><div><div class="eyebrow">ML readiness</div><h2 class="verdict-title">${esc(v.headline||'Analysis verdict')}</h2></div><div class="verdict-kind ${esc(meta.cls)}"><span class="material-symbols-rounded">${esc(meta.icon)}</span><span>${esc(meta.label)}</span></div></div><div class="cs">Limitation: ${esc(v.limitation_kind||'none')} · Next action: ${esc(v.next_action||'review')}</div><div class="report-grid"><div><h3 class="ct">Passed</h3><div class="report-list">${passed||'<div class="cs">No passed criteria reported.</div>'}</div></div><div><h3 class="ct">Failed</h3><div class="report-list">${failed||'<div class="cs">No failed criteria reported.</div>'}</div></div></div><details><summary>Why</summary><div class="report-list" style="margin-top:10px">${cats||'<div class="cs">No category details.</div>'}</div></details></section>`;
}
function renderReportSummary(s,id,body){
  const verdict=s.ml_readiness_verdict||{verdict:s.verdict,headline:'Analysis verdict'};
  const hr=s.hr_metrics||{}; const rr=s.rr_metrics||{}; const sig=s.signal_quality||{}; const ref=s.reference_quality||s.ble_ref_quality||{}; const gates=s.gates||{}; const hist=s.histograms||{}; const truth=s.truthfulness||{};
  const a=s.analysis||{};
  const corrCov=first(s.raw_hr_correction_coverage_pct,a.raw_hr_correction_coverage_pct);
  const rawCorrBias=first(s.raw_hr_corrected_bias_bpm,a.raw_hr_corrected_bias_bpm);
  const rawUncorrBias=first(s.raw_hr_uncorrected_bias_bpm,a.raw_hr_uncorrected_bias_bpm);
  const rescueCount=first(s.hr_rescue_publish_count,a.hr_rescue_publish_count);
  const rescueCov=first(s.hr_rescue_publish_coverage_pct,a.hr_rescue_publish_coverage_pct);
  const oa=s.oracle_audit||a.oracle_audit||{};
  const hrOracleMae=first(s.hr_oracle_candidate_mae_bpm,oa.hr_oracle_candidate_mae_bpm,a.hr_oracle_candidate_mae_bpm);
  const hrOracleCov=first(s.hr_oracle_candidate_coverage_pct,oa.hr_oracle_candidate_coverage_pct,a.hr_oracle_candidate_coverage_pct);
  const rrOracleMae=first(s.rr_oracle_candidate_mae_bpm,oa.rr_oracle_candidate_mae_bpm,a.rr_oracle_candidate_mae_bpm);
  const rrOracleCov=first(s.rr_oracle_candidate_coverage_pct,oa.rr_oracle_candidate_coverage_pct,a.rr_oracle_candidate_coverage_pct);
  const hrOracleHist=first(s.hr_best_candidate_source_histogram,oa.hr_best_candidate_source_histogram,a.hr_best_candidate_source_histogram);
  const rrOracleHist=first(s.rr_best_candidate_source_histogram,oa.rr_best_candidate_source_histogram,a.rr_best_candidate_source_histogram);
  const cl=s.coverage_loss_ledger||a.coverage_loss_ledger||{};
  const hrCoverageLoss=first(s.hr_coverage_loss_by_stage,a.hr_coverage_loss_by_stage,cl.hr_coverage_loss_by_stage,cl.hr);
  const rrCoverageLoss=first(s.rr_coverage_loss_by_stage,a.rr_coverage_loss_by_stage,cl.rr_coverage_loss_by_stage,cl.rr);
  const hrSalvage=first(s.hr_salvageable_frames_pct,a.hr_salvageable_frames_pct,cl.hr_salvageable_frames_pct);
  const rrSalvage=first(s.rr_salvageable_frames_pct,a.rr_salvageable_frames_pct,cl.rr_salvageable_frames_pct);
  const statList=(rows)=>`<div class="stat-list">${rows.filter(r=>r&&r[1]!==null&&r[1]!==undefined&&textOrDash(r[1])!=='--').map(([label,value,cls])=>`<div class="stat ${cls||''}"><div class="k">${esc(label)}</div><div class="v">${esc(value)}</div></div>`).join('')||'<div class="cs">No summary fields available.</div>'}</div>`;
  document.getElementById('reportSub').textContent=`${s.session_id||id} · ${s.status||'unknown'}`;
  if(s.status==='incomplete'){
    body.innerHTML=`${verdictCard(verdict)}<div style="margin-top:10px">${verdictActionHtml(verdict)}</div><div class="setup-alert" style="display:block;margin-top:var(--gap)">Analysis is incomplete for this session. Rerun analysis from the session list or use the CLI.</div>${downloadsHtml(s,id)}`;
    return;
  }
  body.innerHTML=[
    `<div class="report-grid three">${reportMetric('Session',s.session_id||id,'',0)}${reportMetric('Duration',durationLabel(s.duration_s||0),'',0)}${reportMetric('Started',s.started_at||'--','',0)}</div>`,
    `<table style="margin-top:12px"><tbody>${[row('Subject',s.subject_label||s.subject||''),row('Operator',s.operator_label||s.operator||''),row('Started from',s.started_from||''),row('Trainer / Dashboard',`${textOrDash(s.trainer_version)} / ${textOrDash(s.dashboard_version)}`)].join('')}</tbody></table>`,
    verdictCard(verdict),
    `<div style="margin-top:10px">${verdictActionHtml(verdict)}</div>`,
    `<div class="report-grid three"><div data-help="metric.rmse">${reportMetric('HR RMSE',hr.rmse,'bpm',2)}</div><div data-help="metric.r">${reportMetric('HR r',hr.r,'',3)}</div><div data-help="reference.coverage">${reportMetric('HR coverage',hr.coverage_pct,'%',1)}</div><div data-help="metric.rmse">${reportMetric('RR RMSE',rr.rmse,'br/min',2)}</div><div data-help="metric.r">${reportMetric('RR r',rr.r,'',3)}</div><div data-help="reference.coverage">${reportMetric('BLE coverage',ref.distilled_rows_pct_of_raw??ref.coverage_pct,'%',1)}</div></div>`,
    `<div class="report-grid three"><article class="card" data-help="correction.rescue"><div class="ch"><div class="tw"><span class="material-symbols-rounded">hearing</span><div><h2 class="ct">HR correction / rescue</h2><div class="cs">Corrected raw versus uncorrected raw and rescue publishes.</div></div></div></div>${statList([['Correction coverage',corrCov===null||corrCov===undefined?'--':`${fmt(corrCov,1)}%`,corrCov!==null&&Number(corrCov)>=90?'good':corrCov!==null&&Number(corrCov)>=50?'warn':'bad'],['Corrected raw bias',rawCorrBias===null||rawCorrBias===undefined?'--':`${fmt(rawCorrBias,2)} bpm`,rawCorrBias!==null&&Number(rawCorrBias)===0?'good':''],['Uncorrected raw bias',rawUncorrBias===null||rawUncorrBias===undefined?'--':`${fmt(rawUncorrBias,2)} bpm`,''],['Rescue publishes',rescueCount===null||rescueCount===undefined?'--':textOrDash(rescueCount),'' ],['Rescue share',rescueCov===null||rescueCov===undefined?'--':`${fmt(rescueCov,1)}%`,rescueCov!==null&&Number(rescueCov)>0?'warn':'']])}</article><article class="card" data-help="oracle.audit"><div class="ch"><div class="tw"><span class="material-symbols-rounded">psychology_alt</span><div><h2 class="ct">Oracle audit</h2><div class="cs">Best-candidate replay metrics if a blocked signal existed.</div></div></div></div>${statList([['HR oracle MAE',hrOracleMae===null||hrOracleMae===undefined?'--':`${fmt(hrOracleMae,2)} bpm`,hrOracleMae!==null&&Number(hrOracleMae)<8?'good':''],['HR oracle coverage',hrOracleCov===null||hrOracleCov===undefined?'--':`${fmt(hrOracleCov,1)}%`,hrOracleCov!==null&&Number(hrOracleCov)>=50?'good':''],['RR oracle MAE',rrOracleMae===null||rrOracleMae===undefined?'--':`${fmt(rrOracleMae,2)} br/min`,rrOracleMae!==null&&Number(rrOracleMae)<4?'good':''],['RR oracle coverage',rrOracleCov===null||rrOracleCov===undefined?'--':`${fmt(rrOracleCov,1)}%`,rrOracleCov!==null&&Number(rrOracleCov)>=70?'good':'']])}${hrOracleHist||rrOracleHist?`<h3 class="ct" style="font-size:14px;margin-top:14px">Best HR source</h3>${histBars(hrOracleHist)}<h3 class="ct" style="font-size:14px;margin-top:14px">Best RR source</h3>${histBars(rrOracleHist)}`:''}</article><article class="card" data-help="coverage.loss"><div class="ch"><div class="tw"><span class="material-symbols-rounded">analytics</span><div><h2 class="ct">Coverage loss</h2><div class="cs">First failing stage when HR or RR did not publish.</div></div></div></div>${statList([['HR salvageable',hrSalvage===null||hrSalvage===undefined?'--':`${fmt(hrSalvage,1)}%`,hrSalvage!==null&&Number(hrSalvage)>=50?'good':''],['RR salvageable',rrSalvage===null||rrSalvage===undefined?'--':`${fmt(rrSalvage,1)}%`,rrSalvage!==null&&Number(rrSalvage)>=50?'good':''],['HR ledgers',hrCoverageLoss?'present':'--',hrCoverageLoss?'warn':''],['RR ledgers',rrCoverageLoss?'present':'--',rrCoverageLoss?'warn':'']])}${hrCoverageLoss?`<h3 class="ct" style="font-size:14px;margin-top:14px">HR loss by stage</h3>${histBars(hrCoverageLoss)}`:''}${rrCoverageLoss?`<h3 class="ct" style="font-size:14px;margin-top:14px">RR loss by stage</h3>${histBars(rrCoverageLoss)}`:''}</article></div>`,
    `<div class="report-grid"><article class="card" data-help="metric.rmse"><div class="ch"><div class="tw"><span class="material-symbols-rounded">monitoring</span><div><h2 class="ct">Baseline metrics</h2><div class="cs">Valid-only alignment metrics.</div></div></div></div><table><tbody>${metricRows('HR',hr)+metricRows('RR',rr)}</tbody></table></article><article class="card" data-help="reference.coverage"><div class="ch"><div class="tw"><span class="material-symbols-rounded">bluetooth_connected</span><div><h2 class="ct">Reference quality</h2><div class="cs">BLE packet and oximeter quality.</div></div></div></div><table><tbody>${[row('Status',ref.status),row('Raw packets',ref.raw_packets),row('Parsed rows',ref.parsed_rows),row('Coverage',`${fmt(ref.distilled_rows_pct_of_raw??ref.coverage_pct,1)}%`),row('Packet loss',`${fmt(ref.packet_loss_pct,1)}%`),row('Decode error',`${fmt(ref.decode_error_pct,1)}%`),row('PI median',fmt(ref.pi_median,2))].join('')}</tbody></table></article></div>`,
    `<div class="report-grid"><article class="card"><div class="ch"><div class="tw"><span class="material-symbols-rounded">signal_cellular_alt</span><div><h2 class="ct">Signal quality</h2><div class="cs">Radar and session quality indicators.</div></div></div></div><table><tbody>${[row('PQI lock',`${fmt(sig.pqi_lock_pct,1)}%`),row('Locked coverage',`${fmt(sig.coverage_locked,1)}%`),row('Settling coverage',`${fmt(sig.coverage_settling,1)}%`),row('Session quality score',fmt(sig.session_quality_score,1)),row('Internal consistency',fmt(sig.internal_consistency_score,1)),row('Review flags',(s.review_flags||[]).join(', ')||'none')].join('')}</tbody></table></article><article class="card"><div class="ch"><div class="tw"><span class="material-symbols-rounded">rule</span><div><h2 class="ct">Gates</h2><div class="cs">Primary, secondary, and phase subset gates.</div></div></div></div><table><tbody>${[row('Primary',gateLine(gates.primary)),row('Secondary',gateLine(gates.secondary)),row('Combined',gateLine(gates.combined)),row('Locked',gateLine(gates.locked)),row('Settling',gateLine(gates.settling)),row('Golden check',gateLine(gates.golden_check))].join('')}</tbody></table></article></div>`,
    `<div class="report-grid"><article class="card" data-help="hist.gate"><div class="ch"><div class="tw"><span class="material-symbols-rounded">stacked_bar_chart</span><div><h2 class="ct">Gate histograms</h2><div class="cs">Top HR/RR gate reasons remain visible.</div></div></div></div><h3 class="ct" style="font-size:14px">HR gate</h3>${histBars(hist.top_hr_gate)}<h3 class="ct" style="font-size:14px;margin-top:14px">RR gate</h3>${histBars(hist.top_rr_gate)}</article><article class="card" data-help="hist.publish"><div class="ch"><div class="tw"><span class="material-symbols-rounded">history</span><div><h2 class="ct">Publish histograms</h2><div class="cs">Suppression reasons are not hidden.</div></div></div></div><h3 class="ct" style="font-size:14px">HR publish</h3>${histBars(hist.top_hr_publish)}<h3 class="ct" style="font-size:14px;margin-top:14px">RR publish</h3>${histBars(hist.top_rr_publish)}</article></div>`,
    contractDiagnosisHtml(s.contract_diagnosis),
    `<article class="card" style="margin-top:var(--gap)" id="comparePanel"><div class="ch"><div class="tw"><span class="material-symbols-rounded">compare_arrows</span><div><h2 class="ct">Compare</h2><div class="cs">Loading comparison...</div></div></div></div></article>`,
    `<article class="card" style="margin-top:var(--gap)" data-help="fw_truthfulness"><div class="ch"><div class="tw"><span class="material-symbols-rounded">verified_user</span><div><h2 class="ct">Truthfulness / manifest</h2><div class="cs">Firmware, schema, and scoring identity.</div></div></div></div><table><tbody>${[row('Sketch FW',truth.sketch_fw),row('Module FW',truth.module_fw),row('Module valid',truth.module_version_valid),row('Contract length',truth.contract_length),row('Schema hash',truth.schema_hash),row('Scoring weights hash',truth.scoring_weights_hash)].join('')}</tbody></table></article>`,
    downloadsHtml(s,id)
  ].join('');
  renderComparePanel(id);
}
function gateLine(g){ if(!g||typeof g!=='object') return '--'; const st=g.status||(g.passed===true?'PASS':g.passed===false?'FAIL':'--'); return `${textOrDash(st)} · r=${fmt(g.r,3)} · rmse=${fmt(g.rmse,2)} · n=${textOrDash(g.n)}`; }
function downloadsHtml(s,id){
  const downloads=s.downloads||[];
  return `<article class="card" style="margin-top:var(--gap)"><div class="ch"><div class="tw"><span class="material-symbols-rounded">download</span><div><h2 class="ct">Downloads</h2><div class="cs">Raw files and analysis artifacts.</div></div></div><div class="ca"><button class="chip-btn" type="button" onclick="rerunAnalysis('${esc(id)}')">Rerun analysis</button></div></div><div class="chips">${downloads.map(d=>`<a class="chip" href="${esc(d.href||d.url||'#')}">${esc(d.label||d.relpath||'file')}</a>`).join('')}<a class="chip" href="/api/report/export?session=${encodeURIComponent(id)}">Export thesis HTML</a><a class="chip" href="/api/sessions/${encodeURIComponent(id)}/files/analysis/analyse_report.html">Legacy report</a></div></article>`;
}
async function renderHelpView(){
  const body=document.getElementById('helpBody'); if(!body || !S.ctl.on) return;
  if(!S.ctl.help){ try{ S.ctl.help=await apiJson('/api/help/schema'); }catch(e){ body.textContent=e.message; return; } }
  const h=S.ctl.help; const q=String(S.ctl.helpQuery||'').trim().toLowerCase();
  const btn=document.getElementById('helpModeBtn'); if(btn) btn.textContent=S.ctl.helpAdvanced?'Advanced':'Beginner';
  const search=document.getElementById('helpSearch'); if(search && search.value!==S.ctl.helpQuery) search.value=S.ctl.helpQuery;
  const match=(...parts)=>!q || parts.join(' ').toLowerCase().includes(q);
  const faq=(h.faq||[]).filter(x=>match(x.q,x.a)).map(x=>`<dt>${esc(x.q||x.question)}</dt><dd>${esc(x.a||x.answer)}</dd>`).join('');
  const gloss=(h.glossary||[]).filter(x=>match(x.id,x.term,x.short,x.long,x.advanced,x.category)).map(x=>`<dt>${esc(x.term||x.id)} <span class="pill">${esc(x.category||'')}</span></dt><dd>${esc(S.ctl.helpAdvanced?(x.advanced||x.long||x.short):(x.short||x.long||x.advanced))}</dd>`).join('');
  const dsp=(h.dsp_steps||[]).filter(x=>match(x.n,x.title,x.summary,x.detail)).map(x=>`<dt>${esc(`${x.n||''} ${x.title||''}`)}</dt><dd>${esc(S.ctl.helpAdvanced?(x.detail||x.summary):(x.summary||x.detail))}</dd>`).join('');
  const tr=(h.troubleshooting||[]).filter(x=>match(x.id,x.title,(x.steps||[]).join(' '))).map(x=>`<dt>${esc(x.title||x.id)}</dt><dd>${esc((x.steps||[]).join(' '))}</dd>`).join('');
  const tips=Object.entries(h.tooltips||{}).filter(([k,v])=>match(k,v)).slice(0,80).map(([k,v])=>`<dt><code>${esc(k)}</code></dt><dd>${esc(v)}</dd>`).join('');
  body.innerHTML=`<section class="help-section"><details open><summary>FAQ</summary><dl>${faq||'<dd>No matching FAQ entries.</dd>'}</dl></details></section><section class="help-section"><details open><summary>Glossary</summary><dl>${gloss||'<dd>No matching glossary entries.</dd>'}</dl></details></section><section class="help-section"><details><summary>DSP Process</summary><dl>${dsp||'<dd>No matching DSP steps.</dd>'}</dl></details></section><section class="help-section"><details><summary>Troubleshooting</summary><dl>${tr||'<dd>No matching troubleshooting entries.</dd>'}</dl></details></section><section class="help-section"><details><summary>Tooltip Keys</summary><dl>${tips||'<dd>No matching tooltip keys.</dd>'}</dl></details></section>`;
}
function toggleHelpAdvanced(){ S.ctl.helpAdvanced=!S.ctl.helpAdvanced; localStorage.setItem('rvt-help-advanced',S.ctl.helpAdvanced?'1':'0'); renderHelpView(); }

function drawRadar(p,stale=false){
  const c=document.getElementById('radarCanvas'); if(!c||c.offsetParent===null) return;
  const ctx=c.getContext('2d'); const dpr=window.devicePixelRatio||1; const rect=c.getBoundingClientRect(); if(!rect.width||!rect.height) return;
  c.width=rect.width*dpr; c.height=rect.height*dpr; ctx.setTransform(dpr,0,0,dpr,0,0);
  const w=rect.width,h=rect.height; ctx.clearRect(0,0,w,h);
  const dark=document.documentElement.getAttribute('data-theme')==='dark';
  const ox=w/2,oy=h-20; const R=Math.min(w*.46,h*.84); const maxM=6;
  const g=ctx.createRadialGradient(ox,oy,0,ox,oy,R);
  g.addColorStop(0,dark?'rgba(179,197,255,0.06)':'rgba(47,95,201,0.06)'); g.addColorStop(1,'transparent');
  ctx.fillStyle=g; ctx.beginPath(); ctx.arc(ox,oy,R,Math.PI,2*Math.PI); ctx.fill();
  ctx.strokeStyle=dark?'rgba(255,255,255,0.1)':'rgba(116,120,132,.2)'; ctx.lineWidth=1.5; ctx.setLineDash([]);
  for(let i=1;i<=6;i++){ const r=R*(i/maxM); ctx.beginPath(); ctx.arc(ox,oy,r,Math.PI,2*Math.PI); ctx.stroke(); ctx.fillStyle=dark?'#8e9099':'#7b8494'; ctx.font='11px Roboto Mono'; ctx.fillText(`${i}m`,ox+6,oy-r-4); }
  ctx.strokeStyle=dark?'rgba(255,255,255,.06)':'rgba(116,120,132,.1)';
  for(let a=0;a<=180;a+=30){ const rad=(a*Math.PI)/180; ctx.beginPath(); ctx.moveTo(ox,oy); ctx.lineTo(ox+Math.cos(Math.PI+rad)*R, oy+Math.sin(Math.PI+rad)*R); ctx.stroke(); }
  ctx.strokeStyle=dark?'#c4c6d0':'#3b434f'; ctx.lineWidth=1.5;
  ctx.beginPath(); ctx.moveTo(ox,oy); ctx.lineTo(ox,oy-R); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(0,oy); ctx.lineTo(w,oy); ctx.stroke();
  ctx.fillStyle=dark?'#e3e2e6':'#171c24'; ctx.beginPath(); ctx.arc(ox,oy,5,0,Math.PI*2); ctx.fill();
  const sweep=((Date.now()/40)%180)*Math.PI/180;
  const sg=ctx.createLinearGradient(ox,oy,ox+Math.cos(Math.PI+sweep)*R,oy+Math.sin(Math.PI+sweep)*R);
  sg.addColorStop(0,dark?'rgba(179,197,255,.5)':'rgba(47,95,201,.6)'); sg.addColorStop(1,'transparent');
  ctx.strokeStyle=sg; ctx.lineWidth=2; ctx.beginPath(); ctx.moveTo(ox,oy); ctx.lineTo(ox+Math.cos(Math.PI+sweep)*R,oy+Math.sin(Math.PI+sweep)*R); ctx.stroke();
  const x=num(getR(p,'primary_x')), y=num(getR(p,'primary_y'));
  if(!stale && x!==null && y!==null && y>=0){
    const px=ox+(x/maxM)*R, py=oy-(y/maxM)*R;
    const pulse=(Math.sin(Date.now()/400)+1)/2;
    ctx.fillStyle=dark?'rgba(179,197,255,.25)':'rgba(47,95,201,.2)'; ctx.beginPath(); ctx.arc(px,py,14+pulse*8,0,Math.PI*2); ctx.fill();
    ctx.fillStyle=dark?'#b3c5ff':'#2f5fc9'; ctx.beginPath(); ctx.arc(px,py,8,0,Math.PI*2); ctx.fill();
    ctx.strokeStyle=dark?'#1a1b1e':'#ffffff'; ctx.lineWidth=3; ctx.stroke();
    ctx.fillStyle=dark?'#e3e2e6':'#171c24'; ctx.font='600 12px Roboto Mono'; ctx.fillText(`(${x.toFixed(2)}, ${y.toFixed(2)}) m`, Math.min(w-140,px+14), Math.max(18,py-12));
  } else {
    ctx.fillStyle=dark?'#8e9099':'#7b8494'; ctx.font='14px Roboto'; ctx.fillText(stale?'Spatial fix expired':'No fresh spatial fix',16,26);
  }
}

function chartCommon(){
  const dark=document.documentElement.getAttribute('data-theme')==='dark';
  const g=dark?'rgba(255,255,255,0.08)':'rgba(116,120,132,.12)'; const t=dark?'#c4c6d0':'#5b6070';
  return { responsive:true,maintainAspectRatio:false,animation:false,interaction:{mode:'index',intersect:false},
    plugins:{legend:{position:'top',labels:{color:t,boxWidth:10,boxHeight:10,font:{size:11,weight:'600'}}}},
    scales:{ x:{ticks:{color:t,maxTicksLimit:6,font:{size:10}},grid:{color:g}}, y:{ticks:{color:t,font:{size:10}},grid:{color:g}} } };
}
function buildCharts(){
  const co=chartCommon();
  const P={p:'#2f5fc9',pl:'#7da0e5',t:'#1e9a67',tl:'#67d9a1',err:'#ba1a1a',pu:'#7e57c2',sl:'#8d99ae',teal:'#0ea5a4',amb:'#f59e0b',ink:'#0f766e'};
  S.charts.hr=new Chart(document.getElementById('hrChart'),{type:'line',data:{labels:[],datasets:[
    {label:'Reported HR',data:[],borderColor:P.p,backgroundColor:P.p+'22',borderWidth:2.5,tension:.3,fill:true,pointRadius:0},
    {label:'Candidate HR',data:[],borderColor:P.pu,backgroundColor:P.pu,borderWidth:1.5,tension:.25,borderDash:[7,4],pointRadius:0},
    {label:'Raw HR',data:[],borderColor:P.sl,backgroundColor:P.sl,borderWidth:1.5,tension:.25,borderDash:[3,4],pointRadius:0},
    {label:'Corrected raw HR',data:[],borderColor:P.amb,backgroundColor:P.amb,borderWidth:1.5,tension:.25,borderDash:[2,3],pointRadius:0},
    {label:'BLE HR',data:[],borderColor:P.err,backgroundColor:P.err,borderWidth:1.8,tension:.25,pointRadius:0},
    {label:'Trusted anchor',data:[],borderColor:P.ink,backgroundColor:P.ink,borderWidth:1.5,tension:.25,borderDash:[4,4],pointRadius:0}
  ]},options:JSON.parse(JSON.stringify(co))});
  S.charts.rr=new Chart(document.getElementById('rrChart'),{type:'line',data:{labels:[],datasets:[
    {label:'Reported RR',data:[],borderColor:P.t,backgroundColor:P.t+'22',borderWidth:2.5,tension:.3,fill:true,pointRadius:0},
    {label:'Candidate RR',data:[],borderColor:P.tl,backgroundColor:P.tl,borderWidth:1.5,tension:.25,borderDash:[7,4],pointRadius:0},
    {label:'Final publish',data:[],borderColor:P.teal,backgroundColor:P.teal,borderWidth:1.5,tension:.25,borderDash:[3,4],pointRadius:0},
    {label:'RR anchor',data:[],borderColor:P.amb,backgroundColor:P.amb,borderWidth:1.5,tension:.2,pointRadius:0},
    {label:'BLE RR',data:[],borderColor:P.err,backgroundColor:P.err,borderWidth:1.5,tension:.25,pointRadius:0}
  ]},options:JSON.parse(JSON.stringify(co))});
  const diag=JSON.parse(JSON.stringify(co));
  diag.scales.y1={position:'left',min:0,max:1.2,grid:{color:co.scales.y.grid.color},ticks:{color:co.scales.y.ticks.color}};
  diag.scales.y2={position:'right',min:0,max:6,grid:{drawOnChartArea:false},ticks:{color:co.scales.y.ticks.color}};
  delete diag.scales.y;
  S.charts.rrDiag=new Chart(document.getElementById('rrDiagChart'),{type:'line',data:{labels:[],datasets:[
    {label:'RR candidate conf',data:[],borderColor:P.p,backgroundColor:P.p,borderWidth:2,tension:.3,yAxisID:'y1',pointRadius:0},
    {label:'Recovery count',data:[],borderColor:P.pu,backgroundColor:P.pu,borderWidth:2,tension:.1,yAxisID:'y2',pointRadius:0},
    {label:'Seed consistency',data:[],borderColor:P.t,backgroundColor:P.t,borderWidth:2,tension:.1,yAxisID:'y2',pointRadius:0}
  ]},options:diag});
  S.charts.breath=new Chart(document.getElementById('breathChart'),{type:'line',data:{labels:[],datasets:[{label:'breath_phase',data:[],borderColor:'#e05a2a',backgroundColor:'#e05a2a22',borderWidth:2,pointRadius:0,tension:.2,fill:true}]},options:JSON.parse(JSON.stringify(co))});
  S.charts.heart=new Chart(document.getElementById('heartChart'),{type:'line',data:{labels:[],datasets:[{label:'heart_phase',data:[],borderColor:'#c93232',backgroundColor:'#c9323222',borderWidth:2,pointRadius:0,tension:.2,fill:true}]},options:JSON.parse(JSON.stringify(co))});
  const bo=JSON.parse(JSON.stringify(co)); bo.interaction={mode:'nearest',intersect:true}; bo.plugins.legend.display=false;
  bo.scales.x={type:'linear',title:{display:true,text:'Ref HR bucket midpoint (bpm)',color:co.scales.x.ticks.color},ticks:{color:co.scales.x.ticks.color,stepSize:10},grid:{color:co.scales.x.grid.color}};
  bo.scales.y={title:{display:true,text:'Raw HR bias (bpm)',color:co.scales.y.ticks.color},ticks:{color:co.scales.y.ticks.color},grid:{color:co.scales.y.grid.color}};
  S.charts.bucket=new Chart(document.getElementById('rawHrBucketChart'),{type:'scatter',data:{datasets:[{label:'Raw HR bias',data:[],borderColor:P.err,backgroundColor:P.err+'cc',pointRadius:[],pointHoverRadius:8}]},options:bo});

  const so={responsive:true,maintainAspectRatio:false,animation:false,plugins:{legend:{display:false},tooltip:{enabled:false}},scales:{x:{display:false},y:{display:false}},elements:{point:{radius:0}}};
  S.charts.kHr=new Chart(document.getElementById('kpiHrSpark'),{type:'line',data:{labels:[],datasets:[{data:[],borderColor:'currentColor',backgroundColor:'rgba(0,0,0,0.12)',borderWidth:2,tension:.35,fill:true}]},options:so});
  S.charts.kRr=new Chart(document.getElementById('kpiRrSpark'),{type:'line',data:{labels:[],datasets:[{data:[],borderColor:'currentColor',backgroundColor:'rgba(0,0,0,0.12)',borderWidth:2,tension:.35,fill:true}]},options:so});
  S.charts.kFps=new Chart(document.getElementById('kpiFpsSpark'),{type:'line',data:{labels:[],datasets:[{data:[],borderColor:'#2f5fc9',backgroundColor:'rgba(47,95,201,0.15)',borderWidth:2,tension:.35,fill:true}]},options:so});
  S.charts.kDist=new Chart(document.getElementById('kpiDistSpark'),{type:'line',data:{labels:[],datasets:[{data:[],borderColor:'#1e9a67',backgroundColor:'rgba(30,154,103,0.15)',borderWidth:2,tension:.35,fill:true}]},options:so});
}

function updateCharts(p){
  const s=p.series||{}; const labels=Array.isArray(s.t)?s.t:[];
  const apply=(c,ls,arr)=>{ if(!ls.length){ c.data.labels=[]; c.data.datasets.forEach((d,i)=>d.data=arr[i]||[]); } else { const last=num(ls[ls.length-1]); const cut=S.rangeS==='all'?-Infinity:last-S.rangeS; let i=0; while(i<ls.length&&num(ls[i])<cut) i++; c.data.labels=ls.slice(i).map(t=>num(t)!==null?num(t).toFixed(0)+'s':''); c.data.datasets.forEach((d,j)=>{ const a=arr[j]||[]; d.data=a.slice(i); }); } c.update('none'); };
  apply(S.charts.hr,labels,[
    Array.isArray(s.reported_hr)?validSeriesOrNull(s.reported_hr,s.radar_hr_valid):(Array.isArray(s.radar_hr)?validSeriesOrNull(s.radar_hr,s.radar_hr_valid):[]),
    Array.isArray(s.candidate_hr)?s.candidate_hr:[],
    Array.isArray(s.raw_hr_uncorrected)?s.raw_hr_uncorrected:(Array.isArray(s.raw_hr)?s.raw_hr:[]),
    Array.isArray(s.raw_hr_corrected)?s.raw_hr_corrected:[],
    Array.isArray(s.ble_hr)?s.ble_hr:[],
    Array.isArray(s.hr_trusted_phase_anchor)?s.hr_trusted_phase_anchor:[]
  ]);
  apply(S.charts.rr,labels,[
    Array.isArray(s.reported_rr)?validSeriesOrNull(s.reported_rr,s.radar_rr_valid):(Array.isArray(s.radar_rr)?validSeriesOrNull(s.radar_rr,s.radar_rr_valid):[]),
    Array.isArray(s.candidate_rr)?s.candidate_rr:[],
    Array.isArray(s.rr_final_publish_candidate)?s.rr_final_publish_candidate:[],
    Array.isArray(s.rr_anchor_value)?s.rr_anchor_value:[],
    Array.isArray(s.ble_rr)?s.ble_rr:[]
  ]);
  apply(S.charts.rrDiag,labels,[
    Array.isArray(s.candidate_rr_conf)?s.candidate_rr_conf:[],
    Array.isArray(s.rr_fundamental_recovery_count)?s.rr_fundamental_recovery_count:[],
    Array.isArray(s.rr_raw_seed_consistent_count)?s.rr_raw_seed_consistent_count:[]
  ]);
  const breath=Array.isArray(s.breath_phase)?s.breath_phase:[]; const heart=Array.isArray(s.heart_phase)?s.heart_phase:[];
  S.charts.breath.data.labels=labels.slice(-breath.length); S.charts.breath.data.datasets[0].data=breath; S.charts.breath.update('none');
  S.charts.heart.data.labels=labels.slice(-heart.length); S.charts.heart.data.datasets[0].data=heart; S.charts.heart.update('none');

  const bp=bucketData(p);
  S.charts.bucket.data.datasets[0].data=bp.map(x=>({x:x.x,y:x.y}));
  S.charts.bucket.data.datasets[0].pointRadius=bp.map(x=>Math.max(4,Math.min(12,3+Math.sqrt(x.count))));
  S.charts.bucket.update('none');
  document.getElementById('bucketTable').innerHTML = bp.length?bp.slice(-6).map(x=>row(`${x.bucket} bpm`, `${fmt(x.y,1)} bpm bias · n=${x.count}`)).join(''):row('Buckets','Waiting for raw_hr and ref/ble_hr series');

  const roll=a=>a.filter(v=>v!==null&&v!==undefined&&Number.isFinite(Number(v))).slice(-40);
  const hrArr=Array.isArray(s.reported_hr)?validSeriesOrNull(s.reported_hr,s.radar_hr_valid):[];
  const rrArr=Array.isArray(s.reported_rr)?validSeriesOrNull(s.reported_rr,s.radar_rr_valid):[];
  const fpsN=computeFps(p); if(fpsN!==null){ S.spark.fps.push(fpsN); if(S.spark.fps.length>40) S.spark.fps.shift(); }
  const distN=num(getR(p,'distance_cm')); if(distN!==null){ S.spark.dist.push(distN); if(S.spark.dist.length>40) S.spark.dist.shift(); }
  const ups=(c,d)=>{ c.data.labels=d.map((_,i)=>i); c.data.datasets[0].data=d; c.update('none'); };
  ups(S.charts.kHr,roll(hrArr)); ups(S.charts.kRr,roll(rrArr)); ups(S.charts.kFps,S.spark.fps); ups(S.charts.kDist,S.spark.dist);
}

function setRange(btn,s){ S.rangeS=s; btn.parentElement.querySelectorAll('button').forEach(b=>b.classList.remove('active')); btn.classList.add('active'); if(S.lastPayload) updateCharts(S.lastPayload); }

function openDrawer(){ document.getElementById('dv').classList.add('open'); document.getElementById('dvOv').classList.add('open'); }
function closeDrawer(){ document.getElementById('dv').classList.remove('open'); document.getElementById('dvOv').classList.remove('open'); }
function renderAlerts(a){
  const b=document.getElementById('dvBody'); const badge=document.getElementById('alertsBadge');
  if(!a.length){ b.innerHTML=`<div class="dv-e"><span class="material-symbols-rounded">task_alt</span>All clear<div style="margin-top:6px;font-size:12px;font-weight:500;">No active alerts in the current payload.</div></div>`; badge.style.display='none'; return; }
  b.innerHTML=a.map(x=>flt(x.severity,x.title,x.copy)).join('');
  const un=a.filter(x=>x.severity!=='good').length;
  if(un>0){ badge.style.display='grid'; badge.textContent=un; } else badge.style.display='none';
}

function snapshotNow(){
  if(!S.lastPayload) return toast('No data to snapshot');
  const p=S.lastPayload; const stl=hardStale(p);
  const hr=stl?null:livePub(p,'reported_hr'); const rr=stl?null:livePub(p,'reported_rr');
  const bh=p.ble.hr??lastSeries(p.series,'ble_hr'); const br=p.ble.rr??lastSeries(p.series,'ble_rr');
  const snap={ id:Date.now(), time:new Date().toLocaleTimeString([],{hour12:false}), hr, rr, bleHr:bh, bleRr:br,
    hrD:(num(hr)!==null&&num(bh)!==null)?num(hr)-num(bh):null, rrD:(num(rr)!==null&&num(br)!==null)?num(rr)-num(br):null,
    targets:textOrDash(getR(p,'num_targets')), fps:computeFps(p) };
  S.snaps.unshift(snap); if(S.snaps.length>30) S.snaps.pop();
  localStorage.setItem('rvt-snaps',JSON.stringify(S.snaps));
  renderSnaps(); toast(`Snapshot pinned · ${snap.time}`,'bookmark_added');
}
function clearSnaps(){ S.snaps=[]; localStorage.removeItem('rvt-snaps'); renderSnaps(); toast('Snapshots cleared'); }
function rmSnap(id){ S.snaps=S.snaps.filter(s=>s.id!==id); localStorage.setItem('rvt-snaps',JSON.stringify(S.snaps)); renderSnaps(); }
function renderSnaps(){
  const l=document.getElementById('snapsList'); if(!l) return;
  if(!S.snaps.length){ l.innerHTML=`<div class="snap-e">No pinned snapshots yet. Click <span class="material-symbols-rounded" style="font-size:14px;vertical-align:middle">bookmark_add</span> on any card (or use <kbd>⌘K</kbd> → Pin) to capture.</div>`; return; }
  l.innerHTML=S.snaps.map(s=>{
    const hd=s.hrD!==null?` Δ${s.hrD>=0?'+':''}${s.hrD.toFixed(1)}`:'';
    const rd=s.rrD!==null?` Δ${s.rrD>=0?'+':''}${s.rrD.toFixed(1)}`:'';
    return `<div class="snap">
      <span class="material-symbols-rounded" style="color:var(--md-sys-color-primary)">bookmark</span>
      <div>
        <div class="snap-t">${esc(s.time)} · ${s.targets} target(s) · ${fmt(s.fps,1)} Hz</div>
        <div class="snap-v"><span class="hr">HR ${fmt(s.hr,1)}${hd}</span><span class="rr">RR ${fmt(s.rr,1)}${rd}</span><span class="n">BLE ${fmt(s.bleHr,0)}/${fmt(s.bleRr,0)}</span></div>
      </div>
      <button class="ca-btn" onclick="rmSnap(${s.id})" title="Remove"><span class="material-symbols-rounded">close</span></button>
    </div>`;
  }).join('');
}

const CMDS=[
  {id:'o',label:'Go to Overview',icon:'dashboard',run:()=>switchTab('tab-overview')},
  {id:'w',label:'Go to Waves',icon:'monitor_heart',run:()=>switchTab('tab-waves')},
  {id:'h',label:'Go to HR Analysis',icon:'ecg_heart',run:()=>switchTab('tab-hr')},
  {id:'r',label:'Go to RR Analysis',icon:'air',run:()=>switchTab('tab-rr')},
  {id:'s',label:'Go to Snapshots',icon:'bookmark',run:()=>switchTab('tab-snaps')},
  {id:'a',label:'Go to Audit & Logs',icon:'fact_check',run:()=>switchTab('tab-audit')},
  {id:'pin',label:'Pin current snapshot',icon:'bookmark_add',sub:'⌘S',run:snapshotNow},
  {id:'al',label:'Open alerts drawer',icon:'notifications',run:openDrawer},
  {id:'st',label:'Open settings',icon:'tune',run:openSettings},
  {id:'dm',label:'Toggle demo mode',icon:'science',run:toggleDemoMode},
  {id:'th',label:'Toggle theme (light/dark)',icon:'dark_mode',run:toggleTheme},
  {id:'pa',label:'Toggle live polling',icon:'pause',run:togglePause},
  {id:'ex',label:'Export JSON payload',icon:'download',run:exportData},
  {id:'dc',label:'Density: comfortable',icon:'density_medium',run:()=>setDensity('comfortable')},
  {id:'ds',label:'Density: compact',icon:'density_small',run:()=>setDensity('compact')},
  {id:'cs',label:'Clear all snapshots',icon:'delete_sweep',run:clearSnaps},
];
let pSel=0, pFilt=CMDS.slice();
function openPalette(){ document.getElementById('pOv').classList.add('open'); document.getElementById('pIn').value=''; pFilt=CMDS.slice(); pSel=0; renderPalette(); setTimeout(()=>document.getElementById('pIn').focus(),60); }
function closePalette(){ document.getElementById('pOv').classList.remove('open'); }
function renderPalette(){ const l=document.getElementById('pL'); l.innerHTML=pFilt.map((c,i)=>`<div class="p-i ${i===pSel?'sel':''}" data-idx="${i}"><span class="material-symbols-rounded">${safeIcon(c.icon)}</span><span>${esc(c.label)}</span>${c.sub?`<span class="cmd-sub">${esc(c.sub)}</span>`:''}</div>`).join(''); l.querySelectorAll('.p-i').forEach((el,i)=>el.onclick=()=>runCmd(i)); }
function runCmd(i){ const c=pFilt[i]; if(!c) return; closePalette(); setTimeout(()=>c.run(),60); }
document.addEventListener('keydown',e=>{
  if((e.metaKey||e.ctrlKey)&&e.key.toLowerCase()==='k'){ e.preventDefault(); const o=document.getElementById('pOv').classList.contains('open'); if(o) closePalette(); else openPalette(); return; }
  if((e.metaKey||e.ctrlKey)&&e.key.toLowerCase()==='s'){ e.preventDefault(); snapshotNow(); return; }
  if(e.key==='Escape'){ closePalette(); closeDrawer(); const fs=document.querySelector('.card.fs'); if(fs){ fs.classList.remove('fs'); document.body.classList.remove('fs-open'); Object.values(S.charts).forEach(c=>c&&c.resize()); } }
  const po=document.getElementById('pOv').classList.contains('open');
  if(po){ if(e.key==='ArrowDown'){ e.preventDefault(); pSel=Math.min(pSel+1,pFilt.length-1); renderPalette(); } else if(e.key==='ArrowUp'){ e.preventDefault(); pSel=Math.max(pSel-1,0); renderPalette(); } else if(e.key==='Enter'){ e.preventDefault(); runCmd(pSel); } }
  else if(S.ctl.on && !e.metaKey && !e.ctrlKey && !e.altKey && /^[1-4]$/.test(e.key) && !e.target.closest?.('input,textarea,select,[contenteditable="true"]')){
    e.preventDefault();
    switchView(['home','live','report','help'][Number(e.key)-1]);
  }
});
document.addEventListener('input',e=>{ if(e.target.id==='pIn'){ const q=e.target.value.trim().toLowerCase(); pFilt=!q?CMDS.slice():CMDS.filter(c=>c.label.toLowerCase().includes(q)); pSel=0; renderPalette(); } });

function fsCard(btn){ const c=btn.closest('.card'); if(!c) return; const a=document.querySelector('.card.fs'); if(a&&a!==c) a.classList.remove('fs'); c.classList.toggle('fs'); document.body.classList.toggle('fs-open', !!document.querySelector('.card.fs')); const ic=btn.querySelector('.material-symbols-rounded'); if(ic) ic.textContent=c.classList.contains('fs')?'fullscreen_exit':'fullscreen'; setTimeout(()=>Object.values(S.charts).forEach(x=>x&&x.resize()),40); }

document.addEventListener('click',e=>{ const td=e.target.closest('td:last-child'); if(!td||!td.parentElement) return; if(td!==td.parentElement.lastElementChild) return; if(!td.closest('table')) return; const txt=td.textContent.trim(); if(!txt||txt==='--') return; navigator.clipboard?.writeText(txt).then(()=>{ td.classList.add('copied'); setTimeout(()=>td.classList.remove('copied'),1400); }); });

function updateConn(status,text){ const d=document.getElementById('conn'); d.className=`conn ${status||''}`; document.getElementById('connT').textContent=text; }
function updateModeBadge(mode,label){
  const el=document.getElementById('modeBadge');
  const tx=document.getElementById('modeBadgeT');
  if(!el||!tx) return;
  el.className=`mode-badge ${mode||''}`;
  tx.textContent=label;
  const icon=el.querySelector('.material-symbols-rounded');
  if(icon) icon.textContent = mode==='demo' ? 'science' : mode==='stale' ? 'wifi_off' : mode==='live' ? 'wifi' : 'wifi_tethering_off';
}
function openSettings(){ document.getElementById('settingsOv').classList.add('open'); syncSettingsUi(); }
function closeSettings(){ document.getElementById('settingsOv').classList.remove('open'); }
function setSwitchState(id, isOn){ const el=document.getElementById(id); if(!el) return; el.classList.toggle('on', !!isOn); el.setAttribute('aria-pressed', isOn ? 'true' : 'false'); }
function syncSettingsUi(){ setSwitchState('demoModeSwitch', S.demoMode); setSwitchState('autoDemoSwitch', S.autoDemoOnDisconnect); setSwitchState('freezeOnStaleSwitch', S.freezeOnStale); }
function persistDashboardPrefs(){ localStorage.setItem('rvt-dashboard-prefs', JSON.stringify({ demoMode:!!S.demoMode, autoDemoOnDisconnect:!!S.autoDemoOnDisconnect, freezeOnStale:!!S.freezeOnStale })); syncSettingsUi(); }
function loadDashboardPrefs(){
  try{ const prefs=JSON.parse(localStorage.getItem('rvt-dashboard-prefs')||'{}');
    if(typeof prefs.demoMode==='boolean') S.demoMode=prefs.demoMode;
    if(typeof prefs.autoDemoOnDisconnect==='boolean') S.autoDemoOnDisconnect=prefs.autoDemoOnDisconnect;
    if(typeof prefs.freezeOnStale==='boolean') S.freezeOnStale=prefs.freezeOnStale;
  }catch(_){ }
}
function toggleDemoMode(){
  S.demoMode=!S.demoMode; S.autoDemoActive=false; S.disc=0; persistDashboardPrefs();
  if(S.demoMode){ const p=demoPayload(); p.meta.status='demo'; p.meta.note='Demo mode enabled'; S.lastPayload=p; render(p); }
  else {
    const p=S.freezeOnStale && S.lastLivePayload ? makeStalePayload(S.lastLivePayload,'Demo mode disabled. Waiting for live data...') : makeWaitingPayload('Demo mode disabled. Waiting for live_dashboard.json...');
    S.lastPayload=p; render(p); poll();
  }
}
function toggleAutoDemo(){ S.autoDemoOnDisconnect=!S.autoDemoOnDisconnect; persistDashboardPrefs(); }
function toggleFreezeOnStale(){ S.freezeOnStale=!S.freezeOnStale; persistDashboardPrefs(); if(!S.freezeOnStale && !S.demoMode && !S.lastLivePayload) { const p=makeWaitingPayload('Freeze disabled. Waiting for live_dashboard.json...'); S.lastPayload=p; render(p); } }
function clonePayload(v){ return JSON.parse(JSON.stringify(v||{})); }
function makeWaitingPayload(msg='Waiting for live_dashboard.json...'){
  return normalize({
    meta:{status:'waiting',elapsed_s:0,remaining_s:null,session_dir:'--',note:msg},
    radar:{ rows:0, logged_hr_valid:false, logged_rr_valid:false, human:false, human_detected:0, num_targets:0, motion:'idle', last_seen_age_s:S.disc },
    ble:{ rows:0, raw_packets:0 }, thresholds:{ hr_delta_warn:3, hr_delta_crit:8, rr_delta_warn:2, rr_delta_crit:4 },
    faults:[{severity:'warn', title:'Waiting for live data', copy:msg}], events:[`[WAIT] ${msg}`], series:{ t:[] }
  });
}
function makeStalePayload(base, msg='Live poller unavailable'){
  const p=normalize(clonePayload(base || makeWaitingPayload(msg)));
  p.meta=p.meta||{}; p.radar=p.radar||{};
  p.meta.status='stale'; p.meta.note=msg;
  const age=num(p.radar.last_seen_age_s)||0;
  p.radar.last_seen_age_s=Math.max(S.hardStaleS, age + Math.max(1,S.disc));
  p.events=[`[STALE] ${msg}`].concat(Array.isArray(p.events)?p.events:[]).slice(0,80);
  return p;
}

function renderLiveCoach(p,stale,dist){
  const el=document.getElementById('liveCoach'); if(!el||!S.ctl.on) return;
  el.style.display='grid';
  const r=p.radar||{};
  let rangeCard;
  if(stale) rangeCard=flt('bad','Distance coach','Live payload is stale; do not adjust placement from expired range.');
  else if(dist===null) rangeCard=flt('warn','Distance coach','No fresh target range. Confirm the subject is in view.');
  else if(dist<30) rangeCard=flt('warn','Distance coach',`Range ${fmt(dist,0)} cm is close. Move the subject or radar farther apart.`);
  else if(dist>80) rangeCard=flt('warn','Distance coach',`Range ${fmt(dist,0)} cm is far. Move closer toward the 40 cm target.`);
  else rangeCard=flt('good','Distance coach',`Range ${fmt(dist,0)} cm is usable. Hold position.`);
  const moving=boolish(r.doppler_motion)||boolish(r.cluster_anomaly)||boolish(r.multi_target)||String(r.motion||'').toLowerCase().includes('motion');
  const motionCard=stale?flt('bad','Motion coach','Stale payload; motion status is expired.'):moving?flt('warn','Motion coach','Motion or multi-target evidence is active. Ask the subject to sit still and clear nearby reflectors.'):flt('good','Motion coach','No active motion warning. Keep posture and oximeter contact steady.');
  el.innerHTML=rangeCard+motionCard;
}

function render(p){
  const meta=p.meta||{}, radar=p.radar||{}, ble=p.ble||{}, thr=p.thresholds||{}, a=p.analysis||{};
  const pub=a.publish_reason_histogram||{};
  const stl=hardStale(p); const hasRem=meta.remaining_s!==null&&meta.remaining_s!==undefined;
  const isDemo = S.demoMode || S.autoDemoActive || String(meta.status||'').toLowerCase()==='demo';
  if(S.paused) { updateConn('warn','Paused'); updateModeBadge(isDemo?'demo':(stl?'stale':'live'), isDemo?'demo':(stl?'stale':'paused')); }
  else if(isDemo) { updateConn('warn','Demo mode'); updateModeBadge('demo', S.autoDemoActive ? 'auto demo' : 'demo'); }
  else if(stl) { updateConn('bad','Stale'); updateModeBadge('stale','stale'); }
  else if(S.disc>0) { updateConn('warn',`Retry · ${S.disc}`); updateModeBadge('stale','retry'); }
  else if(meta.status==='running') { updateConn('','Live'); updateModeBadge('live','live'); }
  else if(meta.status==='waiting') { updateConn('warn','Waiting'); updateModeBadge('', 'waiting'); }
  else { updateConn('',textOrDash(meta.status)); updateModeBadge('', textOrDash(meta.status)); }

  const ch=[];
  if(meta.status) ch.push(chip(meta.status,meta.status==='running'?'good':(String(meta.status||'').includes('stopped')?'bad':'warn'),meta.status==='running'?'play_circle':'pause_circle'));
  ch.push(chip(`${fmt(meta.elapsed_s,1)}s elapsed`,'','timer'));
  ch.push(chip(hasRem?`${fmt(meta.remaining_s,1)}s remaining`:'manual stop', hasRem&&num(meta.remaining_s)<=5?'warn':'','schedule'));
  if(S.paused) ch.push(chip('UI PAUSED','warn','pause'));
  if(S.demoMode || S.autoDemoActive || String(meta.status||'').toLowerCase()==='demo') ch.push(chip(S.autoDemoActive?'AUTO DEMO':'DEMO MODE','warn','science'));
  if(stl && S.freezeOnStale && !S.demoMode && !S.autoDemoActive) ch.push(chip('STALE · FROZEN','bad','timer_off'));
  if(boolish(radar.use_fast_path)) ch.push(chip('raw fast path','good','bolt'));
  if(num(radar.session_phase)!==null) ch.push(chip(`phase ${phaseName(radar.session_phase)}`,'','schedule'));
  if(boolish(radar.phase_valid_this_frame)) ch.push(chip('live phase valid','good','check_circle'));
  if(!boolish(radar.dsp_ran_this_frame)&&!stl) ch.push(chip('DSP idle this loop','warn','pause_circle'));
  if(boolish(radar.phase_backed_publish_ready)) ch.push(chip('HR phase-ready','good','verified'));
  if(boolish(getR(p,'rr_phase_backed_publish_ready'))) ch.push(chip('RR phase-ready','good','verified'));
  if(boolish(radar.agc_floor_suspect)) ch.push(chip('AGC floor suspect','warn','trending_down'));
  if(boolish(radar.near_field_reflector_suspect)) ch.push(chip('near-field reflector','warn','sensors'));
  if(boolish(radar.hr_anchor_drift_suspect)) ch.push(chip('anchor drift','warn','show_chart'));
  if(boolish(getR(p,'hr_freeze_suspect'))) ch.push(chip('HR stale/frozen','bad','timer_off'));
  if(boolish(radar.fs_fallback_used)) ch.push(chip('fs fallback','warn','tune'));
  if(boolish(getR(p,'rr_source_current_ok'))) ch.push(chip('RR current src','good','radio_button_checked'));
  const fsEff=num(radar.fs_effective); if(fsEff!==null&&fsEff<5.0) ch.push(chip(`low fs ${fmt(fsEff,2)}`,'warn','speed'));
  if(boolish(radar.experimental_profile_enabled)) ch.push(chip('experimental','warn','science'));
  const age=num(radar.last_seen_age_s); if(age!==null&&age>2.5) ch.push(chip(`stale ${fmt(age,1)}s`,stl?'bad':'warn',stl?'error':'warning'));
  if(stl) ch.push(chip('live values expired','bad','timer_off'));
  document.getElementById('chips').innerHTML=ch.join('');
  renderFwBadge(p);

  document.getElementById('physTable').innerHTML=[
    row('pqi_heart',fmt(getR(p,'pqi_heart'),3)),
    row('hr_autocorr_best_conf',fmt(getR(p,'hr_autocorr_best_conf'),3)),
    row('rr_spec_conf',fmt(getR(p,'rr_spec_conf'),3))
  ].join('');
  document.getElementById('polTable').innerHTML=[
    row('top_hr_publish_blocks',textOrDash(first(a.top_hr_publish_blocks,topHist(pub.hr)))),
    row('top_rr_publish_blocks',textOrDash(first(a.top_rr_publish_blocks,topHist(pub.rr)))),
    row('HR publish reason',textOrDash(radar.hr_publish_reason_name||getR(p,'hr_publish_reason'))),
    row('RR publish reason',textOrDash(radar.rr_publish_reason_name||getR(p,'rr_publish_reason')))
  ].join('');

  const radHrL=livePub(p,'reported_hr'); const radRrL=livePub(p,'reported_rr');
  const radHr=stl?null:radHrL; const radRr=stl?null:radRrL;
  const bleHr=ble.hr??lastSeries(p.series,'ble_hr'); const bleRr=ble.rr??lastSeries(p.series,'ble_rr');
  const fps=computeFps(p); const dist=num(getR(p,'distance_cm'));
  renderLiveCoach(p,stl,dist);

  document.getElementById('kpiHr').textContent=fmt(radHr,0);
  document.getElementById('kpiRr').textContent=fmt(radRr,0);
  document.getElementById('kpiFps').textContent=fmt(fps,1);
  document.getElementById('kpiDist').textContent=fmt(dist,0);

  const hrD=(num(radHr)!==null&&num(bleHr)!==null)?num(radHr)-num(bleHr):null;
  const rrD=(num(radRr)!==null&&num(bleRr)!==null)?num(radRr)-num(bleRr):null;
  document.getElementById('kpiHrSub').textContent=(bleHr!=null)?`BLE ${fmt(bleHr,0)} · Δ ${hrD!==null?(hrD>=0?'+':'')+hrD.toFixed(1):'--'}`:'awaiting reference';
  document.getElementById('kpiRrSub').textContent=(bleRr!=null)?`BLE ${fmt(bleRr,0)} · Δ ${rrD!==null?(rrD>=0?'+':'')+rrD.toFixed(1):'--'}`:'awaiting reference';
  document.getElementById('kpiFpsSub').textContent=fps!==null?`DSP loop · ${fmt(fps,1)} Hz median`:'DSP loop';
  document.getElementById('kpiDistSub').textContent=dist!==null?'from radar module':'target distance';

  document.getElementById('radHr').textContent=fmt(radHr,1);
  document.getElementById('radRr').textContent=fmt(radRr,1);
  document.getElementById('bleHr').textContent=fmt(bleHr,1);
  document.getElementById('bleRr').textContent=fmt(bleRr,1);

  const radiusCm=num(getR(p,'position_radius_cm'))??((num(getR(p,'primary_x'))!==null&&num(getR(p,'primary_y'))!==null)?Math.hypot(num(getR(p,'primary_x')),num(getR(p,'primary_y')))*100:null);
  const fwText=(()=>{ const s=textOrDash(radar.sketch_firmware_version||radar.firmware_version); const m=textOrDash(radar.module_firmware_version); if(s!=='--'&&m!=='--') return `${s} | mod ${m}`; return s!=='--'?s:m; })();
  document.getElementById('tgtCount').textContent=`Target: ${textOrDash(getR(p,'num_targets'))}`;
  document.getElementById('fpsV').textContent=fmt(fps,1);
  document.getElementById('brV').textContent=fmt(radRr,1);
  document.getElementById('hrV').textContent=fmt(radHr,1);
  document.getElementById('personV').textContent=boolish(getR(p,'human'))?'Yes':'No';
  document.getElementById('distV').textContent=fmt(getR(p,'distance_cm'),1)==='--'?'--':`${fmt(getR(p,'distance_cm'),1)} cm`;
  document.getElementById('xV').textContent=fmt(getR(p,'primary_x'),3)==='--'?'--':`${fmt(getR(p,'primary_x'),3)} m`;
  document.getElementById('yV').textContent=fmt(getR(p,'primary_y'),3)==='--'?'--':`${fmt(getR(p,'primary_y'),3)} m`;
  document.getElementById('radV').textContent=radiusCm===null?'--':`${radiusCm.toFixed(1)} cm`;
  document.getElementById('fwV').textContent=fwText;
  document.getElementById('srcV').textContent=sourceName(getR(p,'spatial_source'));
  document.getElementById('srcAgeV').textContent=fmt(getR(p,'spatial_age_ms'),0)==='--'?'--':`${fmt(getR(p,'spatial_age_ms'),0)} ms`;
  document.getElementById('dopV').textContent=textOrDash(getR(p,'primary_dop'));
  document.getElementById('dopCmsV').textContent=fmt(getR(p,'primary_dop_speed_cms'),2)==='--'?'--':`${fmt(getR(p,'primary_dop_speed_cms'),2)} cm/s`;

  document.getElementById('posTable').innerHTML=[
    row('Spatial source',sourceName(getR(p,'spatial_source'))),
    row('Target info live',boolish(getR(p,'target_info_ok'))?'Yes':'No'),
    row('Point cloud live',boolish(getR(p,'point_cloud_ok'))?'Yes':'No'),
    row('Multi-target',boolish(getR(p,'multi_target'))?'Yes':'No'),
    row('Cluster',textOrDash(getR(p,'primary_cluster'))),
    row('Spatial age',fmt(getR(p,'spatial_age_ms'),0)==='--'?'--':`${fmt(getR(p,'spatial_age_ms'),0)} ms`),
    row('Phase warmup',boolish(getR(p,'phase_warmup_complete'))?'Yes':'No'),
    row('Clutter alpha',fmt(getR(p,'current_clutter_alpha'),3)),
    row('Rewarm triggered',textOrDash(getR(p,'rewarm_triggered'))),
    row('Rewarm reason',textOrDash(getR(p,'rewarm_reason')))
  ].join('');
  drawRadar(p,stl);

  document.getElementById('radarTable').innerHTML=[
    row('Rows',textOrDash(radar.rows)),
    row('Display source',stl?'expired / stale payload':((boolish(radar.logged_hr_valid)&&num(radar.reported_hr)!==null&&num(radar.reported_hr)>0)?'validated publish':'not published')),
    row('HR publish source',textOrDash(getR(p,'hr_publish_source_class_name')||getR(p,'hr_publish_source_class'))),
    row('RR publish source',textOrDash(getR(p,'rr_publish_source_class_name')||getR(p,'rr_publish_source_class'))),
    row('Session phase',phaseName(getR(p,'session_phase'))),
    row('Harmonic mode',harmonicSum(getR(p,'harmonic_mode'))),
    row('HR triple agree',textOrDash(getR(p,'hr_triple_agree'))),
    row('RR triple agree',textOrDash(getR(p,'rr_triple_agree'))),
    row('Candidate HR',fmt(getR(p,'candidate_hr'),1)),
    row('Candidate RR',fmt(getR(p,'candidate_rr'),1)),
    row('Raw HR',fmt(getR(p,'raw_hr'),1)),
    row('Raw HR (uncorrected)',fmt(getR(p,'raw_hr_uncorrected'),1)),
    row('Raw HR (corrected)',fmt(getR(p,'raw_hr_corrected'),1)),
    row('Raw RR',fmt(getR(p,'raw_rr'),1)),
    row('PQI heart',fmt(getR(p,'pqi_heart'),2)),
    row('PQI breath',fmt(getR(p,'pqi_breath'),2)),
    row('Phase valid',boolish(getR(p,'phase_valid_this_frame'))?'Yes':'No'),
    row('Fs effective',fmt(getR(p,'fs_effective'),2)),
    row('Distance (cm)',fmt(getR(p,'distance_cm')??radar.distance_cm,1)),
    row('Motion',textOrDash(radar.motion)),
    row('Human',textOrDash(radar.human)),
    row('HR valid',textOrDash(radar.logged_hr_valid)),
    row('RR valid',textOrDash(radar.logged_rr_valid))
  ].join('');

  document.getElementById('numT').textContent=textOrDash(radar.num_targets);
  document.getElementById('maxDop').textContent=fmt(radar.max_dop_abs,1);
  document.getElementById('maxDopSp').textContent=fmt(radar.max_dop_speed_cms??getR(p,'max_dop_speed_cms'),1);
  document.getElementById('spTable').innerHTML=[
    row('Point cloud ok',textOrDash(radar.point_cloud_ok)),
    row('Target info ok',textOrDash(radar.target_info_ok)),
    row('Doppler motion',textOrDash(radar.doppler_motion)),
    row('Cluster anomaly',textOrDash(radar.cluster_anomaly)),
    row('Multi-target',textOrDash(radar.multi_target)),
    row('Primary X',fmt(radar.primary_x,2)),
    row('Primary Y',fmt(radar.primary_y,2)),
    row('Primary dop',textOrDash(radar.primary_dop)),
    row('Fast path',textOrDash(radar.use_fast_path)),
    row('Clutter alpha',fmt(radar.current_clutter_alpha,3)),
    row('HR path source',hrPathName(radar.hr_path_source,radar.hr_path_source_name))
  ].join('');

  document.getElementById('bleTable').innerHTML=[
    row('Rows',textOrDash(ble.rows)),
    row('Raw packets',textOrDash(ble.raw_packets)),
    row('SpO₂',ble.spo2===undefined?'--':`${fmt(ble.spo2,0)}%`),
    row('PI',fmt(ble.pi,1)),
    row('Address',textOrDash(ble.address)),
    row('Profile',textOrDash(ble.profile))
  ].join('');

  const hW=num(thr.hr_delta_warn)??3, hC=num(thr.hr_delta_crit)??8;
  const rW=num(thr.rr_delta_warn)??2, rC=num(thr.rr_delta_crit)??4;
  const cards=[], alerts=[];
  const add=(s,t,c)=>{ cards.push(flt(s,t,c)); alerts.push({severity:s,title:t,copy:c}); };
  if(hrD!==null){ const s=Math.abs(hrD)>hC?'bad':Math.abs(hrD)>hW?'warn':'good'; add(s,'Heart-rate mismatch',`Radar ${fmt(radHr,1)} vs BLE ${fmt(bleHr,1)} → Δ ${fmt(hrD,1)} bpm.`); }
  if(rrD!==null){ const s=Math.abs(rrD)>rC?'bad':Math.abs(rrD)>rW?'warn':'good'; add(s,'Respiration mismatch',`Radar ${fmt(radRr,1)} vs BLE ${fmt(bleRr,1)} → Δ ${fmt(rrD,1)} br/min.`); }
  if(boolish(radar.multi_target)) add('bad','Multiple targets','More than one target in the point-cloud. Host DSP may be contaminated.');
  if(boolish(radar.cluster_anomaly)) add('warn','Cluster anomaly','Point-cloud clusters disagree with the single-target assumption.');
  if(boolish(radar.doppler_motion)) add('warn','Doppler motion','Point-cloud Doppler indicates body motion.');
  if(boolish(getR(p,'hr_raw_high_bias_suspect'))) add('warn','High HR anchor bias suspected','Raw/latch HR is staying above the trusted phase anchor while lower phase-backed evidence is present.');
  if(boolish(getR(p,'hr_freeze_suspect'))) add('bad','HR stale/frozen anchor',`Anchor ${fmt(getR(p,'hr_trusted_anchor_value'),1)} bpm, trust age ${textOrDash(getR(p,'hr_trust_age_ms'))} ms.`);
  if(!boolish(getR(p,'module_fw_valid'))) add('bad','Module firmware unknown','Module firmware identity not validated. Treat cross-session comparisons cautiously.');
  if(boolish(getR(p,'buffer_zero_injected'))) add('warn','Buffer zero injection','skipDSP inserted zeros into physiological buffers this loop.');
  if(boolish(getR(p,'rewarm_triggered'))) add('warn','Clutter re-warm triggered', `Reason: ${textOrDash(getR(p,'rewarm_reason'))}`);
  const faults=Array.isArray(p.faults)?p.faults:[];
  cards.push(...faults.slice(0,2).map(f=>flt(f.severity||'warn',f.title||'Fault',f.copy||'')));
  document.getElementById('faults').innerHTML=cards.length?cards.join(''):flt('good','No active fault','No operator-facing fault in the current payload.');
  renderAlerts(alerts);

  document.getElementById('hrPills').innerHTML=[
    pill('Arbiter corrected',boolish(getR(p,'hr_arbiter_corrected'))?1:0),
    pill('Reject-phase rejected',boolish(getR(p,'hr_rejectphase_rejected'))?1:0),
    pill('Coherence rejected',boolish(getR(p,'hr_coherence_rejected'))?1:0),
    pill('Raw source',textOrDash(getR(p,'hr_raw_source'))),
    pill('Raw agree',boolish(getR(p,'hr_raw_agree'))?1:0),
    pill('Bypass active',boolish(getR(p,'hr_bypass_active'))?1:0),
    pill('Grace active',boolish(getR(p,'hr_grace_active'))?1:0),
    pill('Triple agree',boolish(getR(p,'hr_triple_agree'))?1:0),
    pill('Trust fresh',boolish(getR(p,'trusted_hr_fresh'))?1:0),
    pill('High-bias',boolish(getR(p,'hr_raw_high_bias_suspect'))?1:0)
  ].join('');
  document.getElementById('hrFunTable').innerHTML=[
    row('Agreement error (bpm)',fmt(getR(p,'hr_agree_err_bpm'),2)),
    row('HR ZC bpm/conf',`${fmt(getR(p,'hr_zc_bpm'),2)} / ${fmt(getR(p,'hr_zc_conf'),3)}`),
    row('HR spec bpm/mag',`${fmt(getR(p,'hr_spec_bpm'),2)} / ${fmt(getR(p,'hr_spec_mag'),5)}`),
    row('Reject-phase anchor',`${textOrDash(getR(p,'hr_rejectphase_anchor_used'))} · ${fmt(getR(p,'hr_rejectphase_anchor_value'),2)}`),
    row('Arbiter anchor',`${textOrDash(getR(p,'hr_arbiter_anchor_used'))} · ${fmt(getR(p,'hr_arbiter_anchor_value'),2)}`),
    row('Trusted anchor',fmt(getR(p,'hr_trusted_anchor_value'),2)),
    row('Trusted phase anchor',fmt(getR(p,'hr_trusted_phase_anchor'),2)),
    row('Anchor source',textOrDash(getR(p,'hr_anchor_source_name')||hrAnchorName(getR(p,'hr_anchor_source')))),
    row('Anchor error (bpm)',fmt(getR(p,'hr_anchor_err_bpm'),2)),
    row('Raw-anchor Δ (bpm)',fmt(getR(p,'hr_raw_minus_anchor_bpm'),2)),
    row('Raw-phase Δ (bpm)',fmt(getR(p,'hr_raw_minus_phase_bpm'),2)),
    row('HR publish source',textOrDash(getR(p,'hr_publish_source_class_name')||getR(p,'hr_publish_source_class'))),
    row('HR block stage',textOrDash(getR(p,'hr_publish_block_stage_name')||getR(p,'hr_publish_block_stage'))),
    row('Raw high-bias suspect',textOrDash(getR(p,'hr_raw_high_bias_suspect')))
  ].join('');
  document.getElementById('hrStageTable').innerHTML=[
    row('Pre reject-phase',fmt(getR(p,'hr_pre_rejectphase'),2)),
    row('Post reject-phase',fmt(getR(p,'hr_post_rejectphase'),2)),
    row('Post blend',fmt(getR(p,'hr_post_blend'),2)),
    row('Post coherence',fmt(getR(p,'hr_post_coherence'),2)),
    row('Final publish',fmt(getR(p,'hr_final_publish_candidate'),2)),
    row('Trusted phase anchor',fmt(getR(p,'hr_trusted_phase_anchor'),2)),
    row('Anchor source',textOrDash(getR(p,'hr_anchor_source_name')||hrAnchorName(getR(p,'hr_anchor_source')))),
    row('Anchor error',fmt(getR(p,'hr_anchor_err_bpm'),2)),
    row('HR ZC bpm',fmt(getR(p,'hr_zc_bpm'),2)),
    row('HR spec bpm',fmt(getR(p,'hr_spec_bpm'),2)),
    row('Logged HR valid',textOrDash(radar.logged_hr_valid)),
    row('HR publish reason',textOrDash(radar.hr_publish_reason_name))
  ].join('');

  document.getElementById('rrPills').innerHTML=[
    pill('Anchor fresh',boolish(getR(p,'rr_anchor_fresh'))?1:0),
    pill('Raw agree',boolish(getR(p,'rr_raw_agree_ok'))?1:0),
    pill('Outlier persist',textOrDash(getR(p,'rr_outlier_persist'))),
    pill('Recovery',boolish(getR(p,'rr_fundamental_recovery_triggered'))?'triggered':'no'),
    pill('Seed from raw',boolish(getR(p,'rr_seed_from_raw_used'))?1:0),
    pill('Re-anchor ok',boolish(getR(p,'rr_midsession_raw_reanchor_allowed'))?1:0),
    pill('Re-anchor blocked',boolish(getR(p,'rr_midsession_raw_reanchor_blocked'))?1:0),
    pill('Current src',boolish(getR(p,'rr_source_current_ok'))?1:0),
    pill('Latched src',boolish(getR(p,'rr_source_latched_ok'))?1:0),
    pill('Trust fresh',boolish(getR(p,'trusted_rr_fresh'))?1:0)
  ].join('');
  document.getElementById('rrFunTable').innerHTML=[
    row('Gate reason',textOrDash(radar.rr_gate_reason_name)),
    row('Publish reason',textOrDash(radar.rr_publish_reason_name)),
    row('RR ZC bpm/conf',`${fmt(getR(p,'rr_zc_bpm'),2)} / ${fmt(getR(p,'rr_zc_conf'),3)}`),
    row('RR spec bpm/conf',`${fmt(getR(p,'rr_spec_bpm'),2)} / ${fmt(getR(p,'rr_spec_conf'),3)}`),
    row('Candidate conf',fmt(getR(p,'candidate_rr_conf'),3)),
    row('Raw anchor err',fmt(getR(p,'rr_raw_anchor_err_bpm'),2)),
    row('Recovery count',textOrDash(getR(p,'rr_fundamental_recovery_count'))),
    row('Seed consistency',textOrDash(getR(p,'rr_raw_seed_consistent_count'))),
    row('Publish source',textOrDash(getR(p,'rr_publish_source_class_name'))),
    row('Block stage',textOrDash(getR(p,'rr_publish_block_stage_name')))
  ].join('');
  document.getElementById('rrStageTable').innerHTML=[
    row('Pre accept-phase',fmt(stageOr(p,'rr_pre_acceptphase',['candidate_rr']),2)),
    row('Post accept-phase',fmt(stageOr(p,'rr_post_acceptphase',['candidate_rr']),2)),
    row('Post blend',fmt(stageOr(p,'rr_post_blend',['reported_rr']),2)),
    row('Post bias correction',fmt(stageOr(p,'rr_post_bias_correction',['rr_post_blend','reported_rr']),2)),
    row('Post Kalman',rrStage(p,'rr_post_kalman',['reported_rr'])===null?'--':fmt(rrStage(p,'rr_post_kalman',['reported_rr']),2)),
    row('Final publish',fmt(stageOr(p,'rr_final_publish_candidate',['reported_rr']),2)),
    row('Anchor value',fmt(getR(p,'rr_anchor_value'),2)),
    row('Anchor age (ms)',textOrDash(getR(p,'rr_anchor_age_ms'))),
    row('Anchor confidence',fmt(getR(p,'rr_anchor_confidence'),2)),
    row('RR trust age (ms)',textOrDash(getR(p,'rr_trust_age_ms')))
  ].join('');

  const rrF=[];
  if(!boolish(getR(p,'rr_anchor_fresh'))) rrF.push(flt('warn','RR anchor stale','The RR anchor is not fresh. Recovery and publish may rely on stale state.'));
  const rAge=num(getR(p,'candidate_rr_age_ms')); if(rAge!==null&&rAge>10000) rrF.push(flt('warn','RR candidate aging','RR candidate older than 10 s.'));
  if(boolish(getR(p,'rr_midsession_raw_reanchor_blocked'))) rrF.push(flt('warn','RR re-anchor blocked',`Reason: ${textOrDash(getR(p,'rr_midsession_raw_reanchor_reason'))}`));
  if(boolish(getR(p,'rr_fundamental_recovery_triggered'))) rrF.push(flt('good','RR recovery active','Fundamental recovery triggered.'));
  if(stl) rrF.push(flt('bad','Dashboard payload expired','Live values blanked; telemetry age exceeded stale timeout.'));
  document.getElementById('rrFaults').innerHTML=rrF.length?rrF.join(''):flt('good','No RR warning','No RR-specific warning in the current payload.');

  const cg=a.ml_gate_combined||a.ml_gate||{}; const lg=a.ml_gate_locked||{}; const sg=a.ml_gate_settling||{};
  const pg=a.ml_gate||{}; const pStat=pg.status||(pg.passed?'PASS':'FAIL');
  const gaud=a.gate_audit||{}; const fn=gaud.funnel||{}; const fwt=a.fw_truthfulness||{}; const agc=a.agc_anomaly_flags||{};
  const xa=a.experimental_audit||{}; const xb=xa.combined_without_early_or_suspect_half_rate||{};
  const bq=a.ble_ref_quality||ble.quality||{};
  document.getElementById('audTable').innerHTML=[
    row('Sketch truthfulness',textOrDash(fwt.version)),
    row('Module truthfulness',textOrDash(fwt.module_version)),
    row('Module fw valid',textOrDash(fwt.module_version_valid)),
    row('Primary gate',`${textOrDash(pStat)} · r=${fmt(pg.r,3)} · rmse=${fmt(pg.rmse,2)} · n=${textOrDash(pg.n)}`),
    row('Combined gate',`${textOrDash(cg.passed?'PASS':'FAIL')} · r=${fmt(cg.r,3)} · rmse=${fmt(cg.rmse,2)} · n=${textOrDash(cg.n)}`),
    row('Locked gate',`${textOrDash(lg.passed?'PASS':'FAIL')} · r=${fmt(lg.r,3)} · rmse=${fmt(lg.rmse,2)} · n=${textOrDash(lg.n)}`),
    row('Settling gate',`${textOrDash(sg.passed?'PASS':'FAIL')} · r=${fmt(sg.r,3)} · rmse=${fmt(sg.rmse,2)} · n=${textOrDash(sg.n)}`),
    row('Coverage split',`locked ${fmt(a.coverage_locked,1)}% · settling ${fmt(a.coverage_settling,1)}%`),
    row('Funnel',`frames ${textOrDash(fn.radar_frames_total)} · human ${textOrDash(fn.human_frames)} · dsp ${textOrDash(fn.dsp_frames)} · phase ${textOrDash(fn.phase_valid_frames)}`),
    row('Eval bins',`HR ${textOrDash(fn.hr_eval_bins)} · RR ${textOrDash(fn.rr_eval_bins)}`)
  ].join('');
  document.getElementById('histTable').innerHTML=[
    row('Top HR publish blocks',topHist(pub.hr)),
    row('Top RR publish blocks',topHist(pub.rr)),
    row('Top HR gate reasons',topHist(a.hr_gate_reason_histogram)),
    row('Top RR gate reasons',topHist(a.rr_gate_reason_histogram)),
    row('AGC anomaly',`floor ${fmt(agc.gain_floor_pct,1)}% · near ${fmt(agc.near_field_pct,1)}% · skipdsp ${fmt(agc.skipdsp_pct,1)}%`),
    row('Experimental best',`r=${fmt(xb.r,3)} · rmse=${fmt(xb.rmse,2)} · n=${textOrDash(xb.n)}`)
  ].join('');
  document.getElementById('bleQTable').innerHTML=[
    row('Status',textOrDash(bq.status)),
    row('Raw packets',textOrDash(bq.raw_packets??ble.raw_packets)),
    row('Parsed rows',textOrDash(bq.parsed_rows??ble.rows)),
    row('Distilled rows',`${fmt(bq.distilled_rows_pct_of_raw,1)}% of raw`),
    row('Packet loss',`${fmt(bq.packet_loss_pct,1)}%`),
    row('Decode error',`${fmt(bq.decode_error_pct,1)}%`),
    row('PI below threshold',`${fmt(bq.pi_below_threshold_pct,1)}% @ ${fmt(bq.pi_threshold,1)}`)
  ].join('');

  document.getElementById('evLog').textContent=(p.events||[]).length?p.events.join('\n'):'Waiting for events...';
  document.getElementById('foot').innerHTML=`<span class="material-symbols-rounded">folder_open</span>Session folder: ${esc(meta.session_dir||'--')}`;

  updateCharts(p);
}

function demoPayload(){
  S.demoT+=1; const t=S.demoT; const N=120; const ts=Array.from({length:N},(_,i)=>i);
  const hrBase=72+4*Math.sin(t/12); const rrBase=15+1.5*Math.sin(t/18+1);
  const rep_hr=ts.map((_,i)=>hrBase+2*Math.sin(i/3+t/5)+(Math.random()-.5));
  const can_hr=rep_hr.map(v=>v+(Math.random()-.5)*1.2);
  const raw_hr=rep_hr.map(v=>v+(Math.random()-.5)*3.5+1.2);
  const raw_hr_corrected=raw_hr.map(v=>v-(Math.max(0, Math.min(25, 0.4*(v-55)))));
  const ble_hr=rep_hr.map(v=>v+(Math.random()-.5)*0.7-0.3);
  const rep_rr=ts.map((_,i)=>rrBase+0.4*Math.sin(i/4+t/6)+(Math.random()-.5)*.15);
  const can_rr=rep_rr.map(v=>v+(Math.random()-.5)*.3);
  const ble_rr=rep_rr.map(v=>v+(Math.random()-.5)*.2);
  const breath=ts.map((_,i)=>Math.sin((i+t)*0.12)+0.1*Math.sin((i+t)*0.5));
  const heart=ts.map((_,i)=>Math.sin((i+t)*0.9)*0.6+0.15*Math.sin((i+t)*3.5));
  const x=0.35+0.18*Math.sin(t/14); const y=1.85+0.22*Math.cos(t/16); const distCm=Math.hypot(x,y)*100;
  return normalize({
    meta:{status:'running',elapsed_s:t,remaining_s:Math.max(0,320-t),session_dir:'/sessions/demo_2026_04_18'},
    radar:{
      rows:1450+t, reported_hr:rep_hr[rep_hr.length-1], reported_rr:rep_rr[rep_rr.length-1],
      candidate_hr:can_hr[can_hr.length-1], candidate_rr:can_rr[can_rr.length-1],
      raw_hr:raw_hr[raw_hr.length-1], raw_hr_uncorrected:raw_hr[raw_hr.length-1], raw_hr_corrected:raw_hr_corrected[raw_hr_corrected.length-1], raw_rr:rep_rr[rep_rr.length-1]+0.5,
      logged_hr_valid:true, logged_rr_valid:true, num_targets:1,
      primary_x:x, primary_y:y, distance_cm:distCm, primary_dop:3, primary_dop_speed_cms:1.47,
      fps_hz:20, fs_effective:19.8,
      sketch_firmware_version:'v15.1.0', module_firmware_version:'2.1.0', module_fw_valid:true,
      spatial_source:1, spatial_age_ms:40, target_info_ok:true, point_cloud_ok:true, multi_target:false,
      session_phase:3, phase_valid_this_frame:true, phase_backed_publish_ready:true, rr_phase_backed_publish_ready:true,
      human:true, motion:'still', max_dop_abs:3, max_dop_speed_cms:1.5,
      use_fast_path:false, phase_warmup_complete:true, current_clutter_alpha:0.085, dsp_ran_this_frame:true,
      pqi_heart:0.82+0.05*Math.sin(t/10), pqi_breath:0.88, hr_autocorr_best_conf:0.74, rr_spec_conf:0.81,
      hr_triple_agree:true, rr_triple_agree:true,
      hr_publish_reason_name:'published', rr_publish_reason_name:'published',
      hr_publish_source_class_name:'phase_anchor', rr_publish_source_class_name:'accepted',
      hr_trusted_phase_anchor:hrBase, hr_trusted_anchor_value:hrBase,
      hr_anchor_source_name:'trusted_phase', hr_anchor_err_bpm:0.4, hr_raw_minus_anchor_bpm:1.2, hr_raw_minus_phase_bpm:0.9,
      hr_zc_bpm:hrBase, hr_zc_conf:0.7, hr_spec_bpm:hrBase+0.3, hr_spec_mag:0.0032,
      trusted_hr_fresh:true, rr_anchor_fresh:true, rr_source_current_ok:true, rr_source_latched_ok:true, trusted_rr_fresh:true,
      rr_anchor_value:rrBase, rr_anchor_confidence:0.83, candidate_rr_conf:0.82,
      rr_fundamental_recovery_count:0, rr_raw_seed_consistent_count:5,
      hr_raw_source:'combined', last_seen_age_s:0.1, harmonic_mode:2, hr_path_source_name:'auto publish'
    },
    ble:{rows:320,raw_packets:340,spo2:98,pi:3.2,address:'BA:DC:0F:FE:EE:00',profile:'demo',hr:ble_hr[ble_hr.length-1],rr:ble_rr[ble_rr.length-1]},
    thresholds:{hr_delta_warn:3,hr_delta_crit:8,rr_delta_warn:2,rr_delta_crit:4},
    ml_readiness_verdict:{verdict:'ready',readiness_kind:'ready',headline:'Demo signal is ready',limitation_kind:'none',next_action:'review',passed:['Corrected raw HR is flowing','RR remains stable'],failed:[],categories:[{id:'schema',status:'pass',label:'Schema',detail:'Demo payload carries the v11/v15-ready summary fields.'}]},
    faults:[],
    events:[`[${t}] sync ok`,`[${t-1}] publish_hr=${rep_hr[rep_hr.length-1].toFixed(1)}`,`[${t-2}] publish_rr=${rep_rr[rep_rr.length-1].toFixed(1)}`,`[${t-3}] anchor=trusted_phase`],
    series:{
      t:ts, reported_hr:rep_hr, candidate_hr:can_hr, raw_hr, raw_hr_uncorrected:raw_hr, raw_hr_corrected, ble_hr,
      reported_rr:rep_rr, candidate_rr:can_rr, ble_rr,
      rr_final_publish_candidate:rep_rr, rr_anchor_value:ts.map(()=>rrBase),
      hr_trusted_phase_anchor:ts.map(()=>hrBase), breath_phase:breath, heart_phase:heart,
      candidate_rr_conf:ts.map((_,i)=>0.7+0.15*Math.sin(i/5+t/10)),
      rr_fundamental_recovery_count:ts.map((_,i)=>i>60?1:0),
      rr_raw_seed_consistent_count:ts.map((_,i)=>Math.min(5,Math.floor(i/20))),
      radar_hr_valid:ts.map(()=>true), radar_rr_valid:ts.map(()=>true)
    },
    analysis:{
      publish_reason_histogram:{hr:{'published':320,'STALE_FROZEN':4},rr:{'published':318,'LATCH_STALE':3}},
      hr_gate_reason_histogram:{'pass':310,'low_pqi':8}, rr_gate_reason_histogram:{'pass':305,'outlier':12},
      ml_gate:{passed:true,r:0.962,rmse:2.14,n:310,status:'PASS'},
      ml_gate_combined:{passed:true,r:0.958,rmse:2.22,n:308},
      ml_gate_locked:{passed:true,r:0.973,rmse:1.86,n:240},
      ml_gate_settling:{passed:true,r:0.91,rmse:2.82,n:68},
      coverage_locked:78, coverage_settling:22,
      fw_truthfulness:{version:'v15.1.0',module_version:'2.1.0',module_version_valid:true},
      raw_hr_correction_coverage_pct:95.1,
      raw_hr_corrected_bias_bpm:0.6,
      raw_hr_uncorrected_bias_bpm:2.4,
      hr_rescue_publish_count:22,
      hr_rescue_publish_coverage_pct:8.1,
      oracle_audit:{
        hr_oracle_candidate_mae_bpm:1.8,
        hr_oracle_candidate_coverage_pct:82.4,
        rr_oracle_candidate_mae_bpm:0.7,
        rr_oracle_candidate_coverage_pct:88.1,
        hr_best_candidate_source_histogram:{reported_hr:180,candidate_hr:90,raw_hr_corrected:120,hr_spec:45,hr_zc:25},
        rr_best_candidate_source_histogram:{reported_rr:210,candidate_rr:60,raw_rr_effective:40,rr_spec:20,rr_zc:10}
      },
      coverage_loss_ledger:{
        hr_coverage_loss_by_stage:{NO_PHASE:84,RAW_PHASE_STALE_NO_COMPARISON:61,LOW_PQI:12,RAW_DISAGREE_TRUE:8},
        rr_coverage_loss_by_stage:{LOW_PQI:18,STALE_TRACK:9,PUBLISH_POLICY:4},
        hr_salvageable_frames_pct:68.2,
        rr_salvageable_frames_pct:74.9
      },
      agc_anomaly_flags:{gain_floor_pct:1.2,near_field_pct:0.4,skipdsp_pct:0.9},
      gate_audit:{funnel:{radar_frames_total:3400,human_frames:3100,dsp_frames:3080,phase_valid_frames:2900,hr_eval_bins:280,rr_eval_bins:275}},
      experimental_audit:{combined_without_early_or_suspect_half_rate:{r:0.95,rmse:2.3,n:295}},
      ble_ref_quality:{status:'good',raw_packets:340,parsed_rows:320,distilled_rows_pct_of_raw:94.1,packet_loss_pct:2.3,decode_error_pct:0.2,pi_below_threshold_pct:1.8,pi_threshold:0.5},
      raw_hr_bias_audit:{ rate_buckets:[
        {bucket:'50-59',ref_hr_min:50,ref_hr_max:59,bias_bpm:0.8,n:24},
        {bucket:'60-69',ref_hr_min:60,ref_hr_max:69,bias_bpm:1.2,n:58},
        {bucket:'70-79',ref_hr_min:70,ref_hr_max:79,bias_bpm:1.6,n:92},
        {bucket:'80-89',ref_hr_min:80,ref_hr_max:89,bias_bpm:2.1,n:71},
        {bucket:'90-99',ref_hr_min:90,ref_hr_max:99,bias_bpm:1.4,n:35}
      ]}
    }
  });
}

async function poll(){
  if(S.paused) return;
  if(S.demoMode){
    S.autoDemoActive=false;
    const p=demoPayload(); p.meta.status='demo'; p.meta.note='Explicit demo mode'; S.lastPayload=p; S.disc=0; render(p); return;
  }
  try{
    const res=await fetch(`${S.dataUrl}?t=${Date.now()}`,{cache:'no-store'});
    if(!res.ok) throw new Error('HTTP '+res.status);
    const p=normalize(await res.json());
    S.lastPayload=p; S.lastLivePayload=clonePayload(p); S.disc=0; S.autoDemoActive=false; render(p);
  }catch(e){
    S.disc+=1;
    if(S.autoDemoOnDisconnect && S.disc>=2){
      S.autoDemoActive=true;
      const p=demoPayload(); p.meta.status='demo'; p.meta.note='Auto demo on disconnect'; p.meta.auto_demo=true; S.lastPayload=p; render(p);
      return;
    }
    if(S.freezeOnStale && S.lastLivePayload){
      const p=makeStalePayload(S.lastLivePayload, 'No fresh data from trainer/live poller'); S.lastPayload=p; render(p);
      return;
    }
    const p=makeWaitingPayload('No live data from trainer/live poller'); S.lastPayload=p; render(p);
  }
}

async function init(){
  const th=localStorage.getItem('rvt-theme'); if(th==='dark'){ document.documentElement.setAttribute('data-theme','dark'); document.getElementById('themeIcon').textContent='light_mode'; }
  const dn=localStorage.getItem('rvt-density'); if(dn) setDensity(dn);
  loadDashboardPrefs();
  if(await detectControlMode()) setupControlMode();
  buildCharts();
  const seed = S.demoMode ? demoPayload() : makeWaitingPayload('Waiting for live_dashboard.json...');
  if(S.demoMode) seed.meta.status='demo';
  S.lastPayload=seed; render(seed); S.disc=0; syncSettingsUi();
  poll();
  setInterval(()=>{ poll(); if(S.ctl.on) refreshSessionHeader(); }, S.refreshMs);
  setInterval(()=>{ if(S.lastPayload && (S.demoMode || !hardStale(S.lastPayload))) drawRadar(S.lastPayload,hardStale(S.lastPayload)); }, 80);
  renderSnaps();
}
window.addEventListener('DOMContentLoaded', init);
</script>
</body>
</html>
'''
