(function () {
  'use strict';

  var css = `
@media (max-width: 760px) {
  body::before{content:"";position:fixed;inset:0 0 auto;height:36px;background:#172033;z-index:719;pointer-events:none}
  body .main{padding-top:36px!important}
  #mobileRailToggle{position:fixed!important;top:48px!important;left:12px!important;z-index:730!important;width:46px!important;height:46px!important;min-width:46px!important;min-height:46px!important;margin:0!important;transform:none!important}
  .topbar,body[data-view="home"] .topbar,body[data-view="live"] .topbar,body[data-view="report"] .topbar,body[data-view="help"] .topbar,body[data-view="settings"] .topbar{position:sticky!important;top:36px!important;z-index:720!important;display:grid!important;grid-template-columns:minmax(0,1fr) 100px!important;grid-template-areas:"crumbs actions"!important;align-items:start!important;gap:10px!important;width:auto!important;min-height:84px!important;height:auto!important;margin:0 -12px 14px!important;padding:10px 12px 12px 70px!important;overflow:visible!important}
  .topbar .crumbs,body[data-view="home"] .topbar .crumbs,body[data-view="live"] .topbar .crumbs,body[data-view="report"] .topbar .crumbs,body[data-view="help"] .topbar .crumbs,body[data-view="settings"] .topbar .crumbs{grid-area:crumbs!important;display:grid!important;grid-template-columns:minmax(0,1fr)!important;gap:3px!important;min-width:0!important;max-width:100%!important;padding:0!important;margin:0!important;overflow:hidden!important;text-align:left!important;justify-self:stretch!important;justify-content:stretch!important}
  .crumb-trail,#crumbTrail,.uiux-home-crumb,body[data-view="home"] .crumb-trail,body[data-view="home"] #crumbTrail{display:none!important}
  .crumb-eyebrow,#crumbEyebrow,body[data-view="home"] .crumb-eyebrow{display:block!important;margin:0!important;font-size:11px!important;line-height:1.1!important;letter-spacing:.08em!important;white-space:nowrap!important;overflow:hidden!important;text-overflow:ellipsis!important;text-align:left!important}
  .crumb-eyebrow::after,body[data-view="home"] .crumb-eyebrow::after{content:""!important;margin:0!important}
  .crumb-title,.topbar h1,#crumbTitle,body[data-view="home"] .crumb-title{display:-webkit-box!important;-webkit-box-orient:vertical!important;-webkit-line-clamp:2!important;max-width:100%!important;margin:0!important;white-space:normal!important;overflow:hidden!important;text-overflow:clip!important;font-size:clamp(23px,6vw,30px)!important;line-height:1.03!important;letter-spacing:0!important;text-align:left!important}
  .topbar-status,body[data-view="home"] .topbar-status{display:none!important}
  .topbar-actions,body[data-view="home"] .topbar-actions{grid-area:actions!important;align-self:start!important;justify-self:end!important;display:grid!important;grid-template-columns:46px 46px!important;gap:8px!important;width:100px!important;min-width:100px!important;max-width:100px!important;height:46px!important;min-height:46px!important;padding:0!important;margin:0!important;border:0!important;background:transparent!important;box-shadow:none!important;overflow:visible!important}
  .topbar-actions>:not(.tb-overflow):not(.tb-export){display:none!important}
  .topbar-actions>.tb-overflow,.topbar-actions>.tb-export,body[data-view="home"] .topbar-actions>.tb-overflow,body[data-view="home"] .topbar-actions>.tb-export{display:grid!important;place-items:center!important;width:46px!important;min-width:46px!important;height:46px!important;min-height:46px!important;padding:0!important;margin:0!important;border-radius:16px!important;overflow:hidden!important;font-size:0!important;line-height:0!important}
  .topbar-actions>.tb-export .material-symbols-rounded,.topbar-actions>.tb-overflow .material-symbols-rounded{margin:0!important;width:24px!important;height:24px!important;font-size:24px!important;line-height:1!important}
  body[data-view="home"] #setupCard{overflow:hidden!important}
  body[data-view="home"] #setupCard>.ch{display:grid!important;grid-template-columns:minmax(0,1fr) auto!important;align-items:center!important;min-height:82px!important;height:auto!important;margin:0!important;padding:16px!important;gap:12px!important;overflow:hidden!important}
  body[data-view="home"] #setupCard>.ch .tw{min-width:0!important;width:100%!important;overflow:hidden!important}
  body[data-view="home"] #setupCard>.ch .ct,body[data-view="home"] #setupCard>.ch .cs{max-width:100%!important;overflow:hidden!important;text-overflow:ellipsis!important}
  body[data-view="home"] #setupCard>.ch .ca{width:auto!important;min-width:0!important;justify-self:end!important}
  body[data-view="home"] #setupCard .setup-reset-btn{width:44px!important;min-width:44px!important;max-width:44px!important;height:44px!important;min-height:44px!important;padding:0!important;display:grid!important;place-items:center!important;font-size:0!important;line-height:0!important;overflow:hidden!important}
  body[data-view="home"] #setupCard .setup-reset-btn .material-symbols-rounded{width:22px!important;height:22px!important;font-size:22px!important;line-height:1!important;margin:0!important}
  body[data-view="home"] #setupCard .rvt-mobile-disclosure-btn{display:flex!important;align-items:center!important;justify-content:space-between!important;width:calc(100% - 32px)!important;min-width:0!important;height:48px!important;min-height:48px!important;margin:14px 16px 16px!important;padding:0 16px!important;border-radius:14px!important;font-size:14px!important;line-height:1!important;font-weight:900!important;white-space:nowrap!important;overflow:hidden!important;text-overflow:ellipsis!important}
  body[data-view="home"] #setupCard .rvt-mobile-disclosure-btn::after{content:"expand_more";font-family:"Material Symbols Rounded";font-size:22px;font-weight:400;line-height:1;margin-left:10px;flex:0 0 auto}
  body[data-view="home"] #setupCard.rvt-setup-expanded .rvt-mobile-disclosure-btn::after{content:"expand_less"}
}`;

  var iconMap = {
    source: 'sensors',
    palette: 'palette',
    notifications: 'notifications',
    archive: 'inventory_2',
    portability: 'ios_share',
    profile: 'account_circle',
    info: 'info',
    speed: 'speed',
    haptics: 'vibration'
  };

  function setImportant(el, name, value) {
    if (!el || !el.style) return;
    if (el.style.getPropertyValue(name) === value && el.style.getPropertyPriority(name) === 'important') return;
    el.style.setProperty(name, value, 'important');
  }

  function install() {
    var old = document.getElementById('rvt-native-shell-runtime-css');
    if (old) old.remove();
    var style = document.createElement('style');
    style.id = 'rvt-native-shell-runtime-css';
    style.textContent = css;
    document.head.appendChild(style);

    var rail = document.getElementById('mobileRailToggle');
    setImportant(rail, 'position', 'fixed');
    setImportant(rail, 'top', '48px');
    setImportant(rail, 'left', '12px');
    setImportant(rail, 'width', '46px');
    setImportant(rail, 'height', '46px');
    setImportant(rail, 'min-width', '46px');
    setImportant(rail, 'min-height', '46px');
    setImportant(rail, 'z-index', '730');
    setImportant(rail, 'transform', 'none');
    setImportant(rail, 'padding', '0');

    var topbar = document.querySelector('.topbar');
    setImportant(topbar, 'position', 'sticky');
    setImportant(topbar, 'top', '36px');
    setImportant(topbar, 'height', 'auto');
    setImportant(topbar, 'min-height', '84px');
    setImportant(topbar, 'padding', '10px 12px 12px 70px');

    var setupHeader = document.querySelector('body[data-view="home"] #setupCard > .ch');
    setImportant(setupHeader, 'grid-template-columns', 'minmax(0, 1fr) auto');
    setImportant(setupHeader, 'min-height', '82px');
    setImportant(setupHeader, 'margin', '0');
    setImportant(setupHeader, 'width', '100%');
    setImportant(setupHeader, 'top', '0');

    var setupDisclosure = document.querySelector('body[data-view="home"] #setupCard .rvt-mobile-disclosure-btn');
    setImportant(setupDisclosure, 'width', 'calc(100% - 32px)');
    setImportant(setupDisclosure, 'height', '48px');
    setImportant(setupDisclosure, 'margin', '14px 16px 16px');
    setImportant(setupDisclosure, 'font-size', '14px');

    installAndroidSettings();
    syncAndroidSettings();
  }

  function call(name, args) {
    try {
      if (typeof window[name] === 'function') return window[name].apply(window, args || []);
    } catch (_) {}
    return null;
  }

  function isOn(id) {
    var el = document.getElementById(id);
    return !!(el && (el.getAttribute('aria-pressed') === 'true' || el.classList.contains('on')));
  }

  function currentTheme() {
    try {
      return document.documentElement.getAttribute('data-theme') || localStorage.getItem('rvt_theme_mode') || 'light';
    } catch (_) {
      return document.documentElement.getAttribute('data-theme') || 'light';
    }
  }

  function currentDensity() {
    return document.documentElement.getAttribute('data-density') || 'comfortable';
  }

  function rowHtml(key, title, sub, action) {
    var right = action || '<span class="material-symbols-rounded rvt-a16-chevron" aria-hidden="true">chevron_right</span>';
    var tagOpen = action ? '<div class="rvt-a16-row" data-a16-row="' + key + '">' : '<button type="button" class="rvt-a16-row" data-a16-row="' + key + '">';
    var tagClose = action ? '</div>' : '</button>';
    return tagOpen +
      '<span class="rvt-a16-icon"><span class="material-symbols-rounded" aria-hidden="true">' + (iconMap[key] || 'settings') + '</span></span>' +
      '<span class="rvt-a16-copy"><strong>' + title + '</strong><span>' + sub + '</span></span>' +
      right +
      tagClose;
  }

  function switchHtml(id, pressed) {
    return '<button type="button" class="rvt-a16-switch" id="' + id + '" aria-pressed="' + (pressed ? 'true' : 'false') + '"></button>';
  }

  function installAndroidSettings() {
    var view = document.getElementById('view-settings');
    if (!view || document.getElementById('rvtA16Settings')) return;
    view.classList.add('rvt-a16-mounted');
    var shell = document.createElement('section');
    shell.id = 'rvtA16Settings';
    shell.className = 'rvt-a16-settings';
    shell.innerHTML =
      '<div class="rvt-a16-top">' +
        '<div><h2 class="rvt-a16-title">Settings</h2><div class="rvt-a16-subtitle">Radar Vital console preferences</div></div>' +
        '<label class="rvt-a16-search"><span class="material-symbols-rounded" aria-hidden="true">search</span><input id="rvtA16Search" type="search" placeholder="Search Settings" autocomplete="off"></label>' +
      '</div>' +
      '<div class="rvt-a16-layout">' +
        '<div class="rvt-a16-stack">' +
          '<section class="rvt-a16-card rvt-a16-profile"><span class="rvt-a16-avatar">RV</span><span class="rvt-a16-copy"><strong>Radar Vital Trainer</strong><span>Local station preferences and session behavior</span></span><span class="material-symbols-rounded rvt-a16-chevron" aria-hidden="true">chevron_right</span></section>' +
          '<div class="rvt-a16-section-label">Connection and live data</div>' +
          '<section class="rvt-a16-card">' +
            rowHtml('source', 'Demo mode', 'Use generated sample telemetry for demos only.', switchHtml('rvtA16DemoSwitch', false)) +
            rowHtml('speed', 'Auto demo on disconnect', 'Switch to demo after repeated live polling failures.', switchHtml('rvtA16AutoDemoSwitch', false)) +
            rowHtml('archive', 'Freeze stale live data', 'Keep the last real payload visible and mark it stale.', switchHtml('rvtA16FreezeSwitch', true)) +
          '</section>' +
          '<div class="rvt-a16-section-label">Appearance</div>' +
          '<section class="rvt-a16-card">' +
            rowHtml('palette', 'Theme', 'System, light, dark, night red, or high contrast.') +
            '<div class="rvt-a16-segment" data-a16-theme><button class="rvt-a16-chip" data-theme-mode="system">System</button><button class="rvt-a16-chip" data-theme-mode="light">Light</button><button class="rvt-a16-chip" data-theme-mode="dark">Dark</button><button class="rvt-a16-chip" data-theme-mode="night">Night red</button><button class="rvt-a16-chip" data-theme-mode="hc">High contrast</button></div>' +
            rowHtml('speed', 'Density', 'Choose comfortable spacing or compact telemetry density.') +
            '<div class="rvt-a16-segment" data-a16-density><button class="rvt-a16-chip" data-density="comfortable">Comfortable</button><button class="rvt-a16-chip" data-density="compact">Compact</button></div>' +
          '</section>' +
          '<div class="rvt-a16-section-label">Alerts and feedback</div>' +
          '<section class="rvt-a16-card">' +
            rowHtml('notifications', 'Audio alerts', 'Play a short alert tone for threshold and connection events.', switchHtml('rvtA16AudioSwitch', false)) +
            rowHtml('profile', 'Voice announcements', 'Speak eyes-busy alert callouts when supported.', switchHtml('rvtA16VoiceSwitch', false)) +
            rowHtml('haptics', 'Haptic feedback', 'Vibrate on session start, export, copy, and confirmations.') +
          '</section>' +
          '<div class="rvt-a16-section-label">Storage and maintenance</div>' +
          '<section class="rvt-a16-card">' +
            rowHtml('archive', 'Session archive and purge', 'Review local retention, archived rows, and purge actions.') +
            rowHtml('portability', 'Import or export settings', 'Move this console configuration between stations.') +
            rowHtml('profile', 'Settings profiles', 'Save and load named settings snapshots.') +
            rowHtml('info', 'About', 'Dashboard, trainer, firmware, and storage schema versions.') +
          '</section>' +
        '</div>' +
        '<aside class="rvt-a16-card rvt-a16-panel" id="rvtA16Panel">' +
          '<div class="rvt-a16-panel-head"><span class="rvt-a16-icon"><span class="material-symbols-rounded" aria-hidden="true">settings</span></span><div><h3>Settings hierarchy</h3><p>Select a category on the left, or use search to narrow preferences.</p></div></div>' +
          '<div class="rvt-a16-grid">' +
            '<button type="button" class="rvt-a16-action primary" data-a16-action="export"><span class="material-symbols-rounded" aria-hidden="true">download</span>Export settings</button>' +
            '<button type="button" class="rvt-a16-action" data-a16-action="import"><span class="material-symbols-rounded" aria-hidden="true">upload</span>Import settings</button>' +
            '<button type="button" class="rvt-a16-action" data-a16-action="test-sound"><span class="material-symbols-rounded" aria-hidden="true">volume_up</span>Test sound</button>' +
            '<button type="button" class="rvt-a16-action danger" data-a16-action="reset"><span class="material-symbols-rounded" aria-hidden="true">restart_alt</span>Reset defaults</button>' +
          '</div>' +
          '<div class="rvt-a16-about"><div class="rvt-a16-kv"><span>Dashboard</span><strong id="rvtA16DashboardVersion">11.0.0</strong></div><div class="rvt-a16-kv"><span>Trainer</span><strong id="rvtA16TrainerVersion">11.0.0</strong></div><div class="rvt-a16-kv"><span>Firmware</span><strong id="rvtA16FirmwareVersion">v15.0.0</strong></div><div class="rvt-a16-kv"><span>Storage schema</span><strong>Local</strong></div></div>' +
          '<input id="rvtA16ImportFile" type="file" accept=".json,application/json" hidden>' +
        '</aside>' +
      '</div>';
    view.insertBefore(shell, view.firstChild);

    shell.addEventListener('click', function (e) {
      var target = e.target;
      var demo = target.closest('#rvtA16DemoSwitch');
      var autoDemo = target.closest('#rvtA16AutoDemoSwitch');
      var freeze = target.closest('#rvtA16FreezeSwitch');
      var audio = target.closest('#rvtA16AudioSwitch');
      var voice = target.closest('#rvtA16VoiceSwitch');
      var theme = target.closest('[data-theme-mode]');
      var density = target.closest('[data-density]');
      var action = target.closest('[data-a16-action]');
      if (demo) call('toggleDemoMode');
      else if (autoDemo) call('toggleAutoDemo');
      else if (freeze) call('toggleFreezeOnStale');
      else if (audio) call('setAudioAlerts', [!isOn('audioAlertSwitch')]);
      else if (voice) call('setVoiceAlerts', [!isOn('voiceAlertSwitch')]);
      else if (theme) call('setThemeMode', [theme.dataset.themeMode]);
      else if (density) call('setDensity', [density.dataset.density]);
      else if (action) {
        var name = action.dataset.a16Action;
        if (name === 'export') {
          if (window.rvtSettingsIO && typeof window.rvtSettingsIO.export === 'function') window.rvtSettingsIO.export();
          else {
            var btn = document.getElementById('rvtSettingsExportBtn');
            if (btn) btn.click(); else call('exportSettings');
          }
        } else if (name === 'import') {
          var file = document.getElementById('rvtA16ImportFile') || document.getElementById('rvtSettingsImportFile');
          if (file) file.click();
        } else if (name === 'test-sound') {
          call('testAudioAlert');
        } else if (name === 'reset') {
          call('confirmResetDashboardDefaults');
        }
      }
      setTimeout(syncAndroidSettings, 80);
      setTimeout(syncAndroidSettings, 500);
    });

    var search = document.getElementById('rvtA16Search');
    if (search) {
      search.addEventListener('input', function () {
        var q = search.value.trim().toLowerCase();
        Array.prototype.forEach.call(shell.querySelectorAll('.rvt-a16-row'), function (row) {
          row.style.display = !q || row.textContent.toLowerCase().indexOf(q) >= 0 ? '' : 'none';
        });
      });
    }
    var importFile = document.getElementById('rvtA16ImportFile');
    if (importFile) {
      importFile.addEventListener('change', function (e) {
        var f = e.target.files && e.target.files[0];
        if (f && window.rvtSettingsIO && typeof window.rvtSettingsIO.import === 'function') window.rvtSettingsIO.import(f);
        e.target.value = '';
      });
    }
  }

  function syncAndroidSettings() {
    var shell = document.getElementById('rvtA16Settings');
    if (!shell) return;
    var pairs = [
      ['rvtA16DemoSwitch', 'demoModeSwitch'],
      ['rvtA16AutoDemoSwitch', 'autoDemoSwitch'],
      ['rvtA16FreezeSwitch', 'freezeOnStaleSwitch'],
      ['rvtA16AudioSwitch', 'audioAlertSwitch'],
      ['rvtA16VoiceSwitch', 'voiceAlertSwitch']
    ];
    pairs.forEach(function (pair) {
      var mine = document.getElementById(pair[0]);
      if (mine) mine.setAttribute('aria-pressed', isOn(pair[1]) ? 'true' : 'false');
    });
    Array.prototype.forEach.call(shell.querySelectorAll('[data-theme-mode]'), function (btn) {
      btn.setAttribute('aria-pressed', btn.dataset.themeMode === currentTheme() ? 'true' : 'false');
    });
    Array.prototype.forEach.call(shell.querySelectorAll('[data-density]'), function (btn) {
      btn.setAttribute('aria-pressed', btn.dataset.density === currentDensity() ? 'true' : 'false');
    });
    var root = window.RVT_VERSION || window.VERSION || {};
    var dash = document.getElementById('rvtA16DashboardVersion');
    var trainer = document.getElementById('rvtA16TrainerVersion');
    var firmware = document.getElementById('rvtA16FirmwareVersion');
    if (dash && (root.dashboard || window.DASHBOARD_VERSION)) dash.textContent = root.dashboard || window.DASHBOARD_VERSION;
    if (trainer && (root.trainer || window.TRAINER_VERSION)) trainer.textContent = root.trainer || window.TRAINER_VERSION;
    if (firmware && (root.firmware || window.FIRMWARE_VERSION)) firmware.textContent = root.firmware || window.FIRMWARE_VERSION;
  }

  install();
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', install, { once: true });
  window.addEventListener('load', install, { once: true });
  setTimeout(install, 350);
  setTimeout(install, 1200);
  setTimeout(install, 2600);
  setTimeout(install, 5000);
  try {
    var scheduled = 0;
    var observer = new MutationObserver(function (records) {
      for (var i = 0; i < records.length; i += 1) {
        if (records[i].type === 'attributes') {
          if (!scheduled) {
            scheduled = requestAnimationFrame(function () {
              scheduled = 0;
              install();
            });
          }
          break;
        }
      }
    });
    if (document.body) {
      observer.observe(document.body, {
        subtree: true,
        attributes: true,
        attributeFilter: ['style', 'class', 'data-view']
      });
      if (window.rvtTrackMutationObserver) window.rvtTrackMutationObserver(observer);
    }
  } catch (_) {}
})();
