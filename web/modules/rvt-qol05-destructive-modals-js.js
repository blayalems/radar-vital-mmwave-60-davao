/* QOL-05: Destructive-action Diff Modals */
(function() {
  'use strict';

  var backdrop = null;

  function getBackdrop() {
    if (backdrop) return backdrop;
    var el = document.getElementById('rvt-diff-modal-backdrop');
    if (!el) {
      el = document.createElement('div');
      el.id = 'rvt-diff-modal-backdrop';
      el.setAttribute('role', 'dialog');
      el.setAttribute('aria-modal', 'true');
      el.setAttribute('aria-labelledby', 'rvtDiffTitle');
      el.innerHTML =
        '<div id="rvt-diff-modal">' +
          '<h3 class="rvt-diff-title" id="rvtDiffTitle"></h3>' +
          '<div class="rvt-diff-items" id="rvtDiffItems"></div>' +
          '<div class="rvt-diff-check-row" id="rvtDiffCheckRow" style="display:none">' +
            '<input type="checkbox" id="rvtDiffChk" />' +
            '<label for="rvtDiffChk" id="rvtDiffChkLabel"></label>' +
          '</div>' +
          '<div class="rvt-diff-phrase-row" id="rvtDiffPhraseRow" style="display:none">' +
            '<label id="rvtDiffPhraseLabel"></label>' +
            '<input type="text" id="rvtDiffPhraseInput" autocomplete="off" />' +
          '</div>' +
          '<div class="rvt-diff-btn-row">' +
            '<button class="rvt-diff-cancel-btn" id="rvtDiffCancelBtn">Cancel</button>' +
            '<button class="rvt-diff-confirm-btn" id="rvtDiffConfirmBtn" disabled>Confirm</button>' +
          '</div>' +
        '</div>';
      document.body.appendChild(el);
    }
    backdrop = el;
    return el;
  }

  /**
   * rvtConfirmDestructive(opts) — show a diff modal before a destructive action
   * opts.title        {string}   Modal heading
   * opts.items        {string[]} What will be lost (list items)
   * opts.impact       {'low'|'high'} — low: checkbox only; high: typed phrase
   * opts.phrase       {string}   Required phrase for high-impact (default 'confirm')
   * opts.onConfirm    {function} Called if user confirms
   * opts.onCancel     {function} Called if user cancels (optional)
   */
  function rvtConfirmDestructive(opts) {
    opts = opts || {};
    var bd = getBackdrop();
    var modal       = bd.querySelector('#rvt-diff-modal');
    var titleEl     = bd.querySelector('#rvtDiffTitle');
    var itemsEl     = bd.querySelector('#rvtDiffItems');
    var checkRow    = bd.querySelector('#rvtDiffCheckRow');
    var chk         = bd.querySelector('#rvtDiffChk');
    var chkLabel    = bd.querySelector('#rvtDiffChkLabel');
    var phraseRow   = bd.querySelector('#rvtDiffPhraseRow');
    var phraseLabel = bd.querySelector('#rvtDiffPhraseLabel');
    var phraseInput = bd.querySelector('#rvtDiffPhraseInput');
    var cancelBtn   = bd.querySelector('#rvtDiffCancelBtn');
    var confirmBtn  = bd.querySelector('#rvtDiffConfirmBtn');

    var items  = opts.items || [];
    var impact = opts.impact || 'low';
    var phrase = (opts.phrase || 'confirm').toLowerCase();

    /* Populate */
    titleEl.textContent = opts.title || 'Confirm action';
    var itemsHtml = '<p>This will permanently remove:</p><ul>';
    items.forEach(function(it){ itemsHtml += '<li>' + it + '</li>'; });
    if (!items.length) itemsHtml += '<li>Selected data</li>';
    itemsHtml += '</ul>';
    itemsEl.innerHTML = itemsHtml;

    /* Reset controls */
    chk.checked = false;
    phraseInput.value = '';
    confirmBtn.disabled = true;

    if (impact === 'high') {
      checkRow.style.display   = 'none';
      phraseRow.style.display  = '';
      phraseLabel.textContent  = 'Type "' + phrase + '" to confirm:';
      confirmBtn.textContent   = 'Permanently delete';
    } else {
      checkRow.style.display   = '';
      phraseRow.style.display  = 'none';
      chkLabel.textContent     = 'I understand this cannot be undone.';
      confirmBtn.textContent   = 'Confirm';
    }

    function updateConfirmState() {
      if (impact === 'high') {
        confirmBtn.disabled = phraseInput.value.trim().toLowerCase() !== phrase;
      } else {
        confirmBtn.disabled = !chk.checked;
      }
    }

    chk.onchange = updateConfirmState;
    phraseInput.oninput = updateConfirmState;

    function close(confirmed) {
      bd.classList.remove('rvt-diff-visible');
      setTimeout(function(){ bd.style.display='none'; }, 180);
      chk.onchange = null; phraseInput.oninput = null;
      cancelBtn.onclick = null; confirmBtn.onclick = null;
      if (confirmed && typeof opts.onConfirm === 'function') opts.onConfirm();
      else if (!confirmed && typeof opts.onCancel === 'function') opts.onCancel();
    }

    cancelBtn.onclick  = function(){ close(false); };
    confirmBtn.onclick = function(){ if (!confirmBtn.disabled) close(true); };

    bd.onclick = function(e){ if (e.target === bd) close(false); };

    bd.style.display = 'flex';
    bd.classList.add('rvt-diff-visible');
    cancelBtn.focus();
  }

  /* Wrap clearSnaps — the primary destructive action in the dashboard */
  function wrapClearSnaps() {
    if (typeof window.clearSnaps !== 'function') return false;
    if (window.clearSnaps.__rvtQol05Wrapped) return true;
    var orig = window.clearSnaps;
    window.clearSnaps = function() {
      var snaps = (window.S && Array.isArray(window.S.snaps)) ? window.S.snaps.length : 0;
      var notes = {};
      try { notes = JSON.parse(localStorage.getItem('rvt-snap-notes') || '{}') || {}; } catch(_) {}
      var noteCount = Object.keys(notes).length;
      var pins  = [];
      try { pins  = JSON.parse(localStorage.getItem('rvt-alert-pins') || '[]') || []; } catch(_) {}
      var items = [];
      if (snaps)     items.push(snaps + ' snapshot' + (snaps===1?'':'s'));
      if (noteCount) items.push(noteCount + ' annotation' + (noteCount===1?'':'s'));
      if (pins.length) items.push(pins.length + ' pinned alert' + (pins.length===1?'':'s'));
      rvtConfirmDestructive({
        title:     'Clear all snapshots',
        items:     items,
        impact:    snaps > 10 ? 'high' : 'low',
        phrase:    'delete',
        onConfirm: function(){ orig.call(window); }
      });
    };
    window.clearSnaps.__rvtQol05Wrapped = true;
    return true;
  }

  /* Wrap rvtBulkAcknowledgeAlerts */
  function wrapBulkAck() {
    if (typeof window.rvtBulkAcknowledgeAlerts !== 'function') return false;
    if (window.rvtBulkAcknowledgeAlerts.__rvtQol05Wrapped) return true;
    var orig = window.rvtBulkAcknowledgeAlerts;
    window.rvtBulkAcknowledgeAlerts = function() {
      var alertEls = document.querySelectorAll('.alert-item:not([data-acked="true"])');
      var count = alertEls.length;
      rvtConfirmDestructive({
        title:     'Acknowledge all alerts',
        items:     [count + ' unacknowledged alert' + (count===1?'':'s')],
        impact:    'low',
        onConfirm: function(){ orig.apply(window, arguments); }
      });
    };
    window.rvtBulkAcknowledgeAlerts.__rvtQol05Wrapped = true;
    return true;
  }

  /* Wrap rvtPurgeArchivedSessions */
  function wrapPurgeArchived() {
    if (typeof window.rvtPurgeArchivedSessions !== 'function') return false;
    if (window.rvtPurgeArchivedSessions.__rvtQol05Wrapped) return true;
    var orig = window.rvtPurgeArchivedSessions;
    window.rvtPurgeArchivedSessions = function() {
      rvtConfirmDestructive({
        title:     'Purge archived sessions',
        items:     ['All archived session records from local storage'],
        impact:    'high',
        phrase:    'purge',
        onConfirm: function(){ orig.apply(window, arguments); }
      });
    };
    window.rvtPurgeArchivedSessions.__rvtQol05Wrapped = true;
    return true;
  }

  function installWraps() {
    var done = wrapClearSnaps() && wrapBulkAck() && wrapPurgeArchived();
    if (!done) setTimeout(installWraps, 300);
  }

  function boot() {
    getBackdrop(); /* build DOM now */
    installWraps();
    window.rvtConfirmDestructive = rvtConfirmDestructive;
  }

  if (document.readyState==='loading') {
    document.addEventListener('DOMContentLoaded', function(){ setTimeout(boot,700); }, {once:true});
  } else { setTimeout(boot,700); }
})();
