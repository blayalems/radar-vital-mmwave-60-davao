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

  function setImportant(el, name, value) {
    if (!el || !el.style) return;
    if (el.style.getPropertyValue(name) === value && el.style.getPropertyPriority(name) === 'important') return;
    el.style.setProperty(name, value, 'important');
  }

  function clearUtilityTopbarStyles(el) {
    if (!el || !el.style || el.dataset.rvtUtilityTopbar !== '1') return;
    delete el.dataset.rvtUtilityTopbar;
    [
      'display', 'grid-template-columns', 'gap', 'width', 'min-width', 'max-width', 'height', 'min-height',
      'padding', 'margin', 'border', 'background', 'box-shadow', 'overflow', 'place-items', 'position',
      'grid-column', 'grid-row', 'grid-area', 'border-radius', 'font-size', 'line-height', 'flex'
    ].forEach(function (name) { el.style.removeProperty(name); });
  }

  function markUtilityTopbar(el) {
    if (el && el.dataset) el.dataset.rvtUtilityTopbar = '1';
  }

  function ensureSettingsRoutePage() {
    var view = document.getElementById('view-settings');
    var modal = document.querySelector('#settingsOv .set-md');
    if (!view || !modal) return;

    var page = document.getElementById('rvtSettingsRoutePage');
    if (!page) {
      page = document.createElement('section');
      page.id = 'rvtSettingsRoutePage';
      page.className = 'settings-page';
      page.innerHTML =
        '<div class="ch settings-page-head">' +
          '<div class="tw">' +
            '<span class="material-symbols-rounded" aria-hidden="true">tune</span>' +
            '<div><div class="ct">Dashboard settings</div>' +
            '<div class="cs">Session source, appearance, alert thresholds, audio, and live rendering.</div></div>' +
          '</div>' +
          '<div class="ca"><button class="chip-btn" type="button" id="rvtSettingsSyncBtn">' +
            '<span class="material-symbols-rounded" aria-hidden="true">sync</span>Sync</button></div>' +
        '</div>' +
        '<div class="settings-page-toolbar">' +
          '<button class="settings-tab is-active" type="button" data-settings-jump="source">Source</button>' +
          '<button class="settings-tab" type="button" data-settings-jump="appearance">Appearance</button>' +
          '<button class="settings-tab" type="button" data-settings-jump="alerts">Alerts</button>' +
          '<button class="settings-tab" type="button" data-settings-jump="rendering">Rendering</button>' +
        '</div>' +
        '<div class="settings-page-grid" id="rvtSettingsRouteGrid"></div>';
      view.insertBefore(page, view.firstChild);
    } else if (view.firstElementChild !== page) {
      view.insertBefore(page, view.firstChild);
    }

    var grid = document.getElementById('rvtSettingsRouteGrid');
    if (!grid) return;
    grid.innerHTML = '';

    Array.prototype.forEach.call(modal.children, function (child) {
      if (!child || child.classList.contains('set-h')) return;
      if (child.classList.contains('set-actions')) {
        var actions = child.cloneNode(true);
        Array.prototype.forEach.call(actions.querySelectorAll('button'), function (button) {
          if (/done/i.test(button.textContent || '')) button.remove();
        });
        if (actions.children.length) grid.appendChild(actions);
        return;
      }
      var clone = child.cloneNode(true);
      clone.dataset.settingsRoute = '1';
      grid.appendChild(clone);
    });

    var sync = document.getElementById('rvtSettingsSyncBtn');
    if (sync && sync.dataset.bound !== '1') {
      sync.dataset.bound = '1';
      sync.addEventListener('click', function () {
        try { window.syncSettingsUi && window.syncSettingsUi(); } catch (_) {}
        try { window.syncThresholdControls && window.syncThresholdControls(); } catch (_) {}
      });
    }

    Array.prototype.forEach.call(page.querySelectorAll('[data-settings-jump]'), function (btn) {
      if (btn.dataset.bound === '1') return;
      btn.dataset.bound = '1';
      btn.addEventListener('click', function () {
        var key = btn.getAttribute('data-settings-jump');
        var target = key === 'source' ? '.set-source' :
          key === 'appearance' ? '.set-cosmetic' :
          key === 'alerts' ? '.set-critical, .set-feedback' :
          '.set-g:last-of-type';
        var section = page.querySelector(target);
        if (section) section.scrollIntoView({ block: 'start', behavior: 'smooth' });
      });
    });

    try { window.syncSettingsUi && window.syncSettingsUi(); } catch (_) {}
    try { window.syncThresholdControls && window.syncThresholdControls(); } catch (_) {}
  }

  function preferSettingsRoute() {
    if (window.__rvtNativeSettingsRouteInstalled) return;
    window.__rvtNativeSettingsRouteInstalled = true;
    var originalOpenSettings = window.openSettings;
    window.openSettings = function (opts) {
      if (opts && opts.modal && typeof originalOpenSettings === 'function') {
        return originalOpenSettings.apply(this, arguments);
      }
      ensureSettingsRoutePage();
      if (typeof window.switchView === 'function') window.switchView('settings');
      setTimeout(ensureSettingsRoutePage, 80);
      setTimeout(ensureSettingsRoutePage, 360);
    };
  }

  function syncNativeTopbar() {
    var desktop = window.innerWidth > 760;
    var topbar = document.querySelector('.topbar');
    var main = document.querySelector('.main');
    var status = document.querySelector('.topbar-status');
    var actions = document.querySelector('.topbar-actions');
    var command = document.querySelector('.command-strip');

    if (command) setImportant(command, 'display', 'none');

    if (!topbar || !actions || !desktop) {
      clearUtilityTopbarStyles(actions);
      if (actions) Array.prototype.forEach.call(actions.children, function (el) {
        clearUtilityTopbarStyles(el);
        Array.prototype.forEach.call(el.querySelectorAll('.material-symbols-rounded'), clearUtilityTopbarStyles);
      });
      return;
    }

    setImportant(main, 'padding-top', '0');
    var mainStyles = getComputedStyle(main);
    var padLeft = parseFloat(mainStyles.paddingLeft) || 0;
    var padRight = parseFloat(mainStyles.paddingRight) || padLeft;

    markUtilityTopbar(topbar);
    setImportant(topbar, 'position', 'sticky');
    setImportant(topbar, 'top', '0');
    setImportant(topbar, 'z-index', '720');
    setImportant(topbar, 'display', 'grid');
    setImportant(topbar, 'grid-template-columns', 'minmax(0, 1fr) auto auto');
    setImportant(topbar, 'grid-template-areas', '"crumbs status actions"');
    setImportant(topbar, 'align-items', 'center');
    setImportant(topbar, 'gap', '14px');
    setImportant(topbar, 'width', 'calc(100% + ' + (padLeft + padRight) + 'px)');
    setImportant(topbar, 'max-width', 'none');
    setImportant(topbar, 'min-height', '92px');
    setImportant(topbar, 'height', '92px');
    setImportant(topbar, 'margin', '0 -' + padRight + 'px 18px -' + padLeft + 'px');
    setImportant(topbar, 'padding', '14px 24px');
    setImportant(topbar, 'overflow', 'visible');
    setImportant(topbar, 'background', 'var(--bg, #f4f7fb)');
    setImportant(topbar, 'border-bottom', '1px solid rgba(202, 213, 226, .95)');
    setImportant(topbar, 'box-shadow', '0 18px 34px rgba(15, 23, 42, .08)');
    setImportant(topbar, 'backdrop-filter', 'none');

    var crumbs = topbar.querySelector('.crumbs');
    markUtilityTopbar(crumbs);
    setImportant(crumbs, 'grid-area', 'crumbs');
    setImportant(crumbs, 'min-width', '0');
    setImportant(crumbs, 'overflow', 'hidden');

    if (status) {
      markUtilityTopbar(status);
      setImportant(status, 'grid-area', 'status');
      setImportant(status, 'display', 'flex');
      setImportant(status, 'align-items', 'center');
      setImportant(status, 'gap', '8px');
      setImportant(status, 'width', 'auto');
      setImportant(status, 'min-width', '0');
      Array.prototype.forEach.call(status.children, function (el) {
        var keep = el.id === 'modeBadge';
        markUtilityTopbar(el);
        setImportant(el, 'display', keep ? 'inline-flex' : 'none');
        if (keep) {
          setImportant(el, 'height', '46px');
          setImportant(el, 'min-height', '46px');
          setImportant(el, 'max-width', '220px');
          setImportant(el, 'padding', '0 12px');
          setImportant(el, 'border-radius', '18px');
        }
      });
    }

    markUtilityTopbar(actions);
    setImportant(actions, 'display', 'grid');
    setImportant(actions, 'grid-area', 'actions');
    setImportant(actions, 'grid-template-columns', '52px 52px 52px 52px');
    setImportant(actions, 'gap', '8px');
    setImportant(actions, 'width', '232px');
    setImportant(actions, 'min-width', '232px');
    setImportant(actions, 'max-width', '232px');
    setImportant(actions, 'height', '52px');
    setImportant(actions, 'min-height', '52px');
    setImportant(actions, 'padding', '0');
    setImportant(actions, 'margin', '0');
    setImportant(actions, 'border', '0');
    setImportant(actions, 'background', 'transparent');
    setImportant(actions, 'box-shadow', 'none');
    setImportant(actions, 'overflow', 'visible');
    Array.prototype.forEach.call(actions.children, function (el) {
      var keep = el.classList.contains('tb-search') ||
        el.classList.contains('tb-alerts') ||
        el.classList.contains('tb-overflow') ||
        el.classList.contains('tb-export') ||
        el.classList.contains('tb-overflow-menu');
      markUtilityTopbar(el);
      if (!keep) {
        setImportant(el, 'display', 'none');
        return;
      }
      if (el.classList.contains('tb-overflow-menu')) return;
      setImportant(el, 'display', 'grid');
      setImportant(el, 'place-items', 'center');
      setImportant(el, 'position', 'static');
      setImportant(el, 'grid-column', 'auto');
      setImportant(el, 'grid-row', 'auto');
      setImportant(el, 'width', '52px');
      setImportant(el, 'min-width', '52px');
      setImportant(el, 'max-width', '52px');
      setImportant(el, 'height', '52px');
      setImportant(el, 'min-height', '52px');
      setImportant(el, 'padding', '0');
      setImportant(el, 'margin', '0');
      setImportant(el, 'flex', '0 0 52px');
      setImportant(el, 'overflow', 'hidden');
      setImportant(el, 'border-radius', '16px');
      setImportant(el, 'font-size', '0');
      setImportant(el, 'line-height', '0');
      setImportant(el, 'gap', '0');
      Array.prototype.forEach.call(el.querySelectorAll('.material-symbols-rounded'), function (icon) {
        markUtilityTopbar(icon);
        setImportant(icon, 'width', '24px');
        setImportant(icon, 'height', '24px');
        setImportant(icon, 'margin', '0');
        setImportant(icon, 'font-size', '24px');
        setImportant(icon, 'line-height', '1');
      });
    });
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

    syncNativeTopbar();
    ensureSettingsRoutePage();
    preferSettingsRoute();
  }

  function scheduleUtilityTopbarSyncs() {
    syncNativeTopbar();
    setTimeout(syncNativeTopbar, 90);
    setTimeout(syncNativeTopbar, 320);
    setTimeout(syncNativeTopbar, 900);
    setTimeout(syncNativeTopbar, 1700);
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
              scheduleUtilityTopbarSyncs();
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
  setInterval(syncNativeTopbar, 1500);
})();
