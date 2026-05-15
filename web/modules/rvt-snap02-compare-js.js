(function() {
  var selectedSnaps = [];
  var compareBtn = null;
  var compareModal = null;
  var multiSelectMode = false;

  function getSnapId(el) {
    return el.getAttribute('data-snap-id') || el.getAttribute('id') || '';
  }

  function getSnapLabel(el) {
    var lbl = el.querySelector('.rvt-snap-card-label');
    return lbl ? lbl.textContent.trim() : getSnapId(el);
  }

  function enableMultiSelect() {
    if (multiSelectMode) return;
    multiSelectMode = true;
    var snapsList = document.querySelector('[data-view="snapshots"] .snaps-list, .snaps-grid-container, [data-snap-list]');
    if (snapsList) snapsList.classList.add('rvt-snap-multi-select');

    compareBtn = document.createElement('button');
    compareBtn.className = 'rvt-snap-compare-btn';
    compareBtn.textContent = 'Compare Selected (0)';
    compareBtn.disabled = true;
    compareBtn.onclick = openCompareModal;
    compareBtn.setAttribute('aria-label', 'Compare selected snapshots');

    var header = document.querySelector('[data-view="snapshots"] .view-header, .snapshots-header');
    if (header) header.appendChild(compareBtn);
  }

  function disableMultiSelect() {
    if (!multiSelectMode) return;
    multiSelectMode = false;
    selectedSnaps = [];
    var snapsList = document.querySelector('[data-view="snapshots"] .snaps-list, .snaps-grid-container, [data-snap-list]');
    if (snapsList) snapsList.classList.remove('rvt-snap-multi-select');

    document.querySelectorAll('[data-selected="true"]').forEach(function(el) {
      el.removeAttribute('data-selected');
    });

    if (compareBtn) compareBtn.remove();
    compareBtn = null;
  }

  function toggleSnapSelection(snapEl) {
    if (!multiSelectMode) return;
    var snapId = getSnapId(snapEl);
    if (!snapId) return;

    var isSelected = snapEl.getAttribute('data-selected') === 'true';
    if (isSelected) {
      snapEl.removeAttribute('data-selected');
      selectedSnaps = selectedSnaps.filter(function(id) { return id !== snapId; });
    } else {
      snapEl.setAttribute('data-selected', 'true');
      selectedSnaps.push(snapId);
    }

    updateCompareButton();
  }

  function updateCompareButton() {
    if (!compareBtn) return;
    var count = selectedSnaps.length;
    compareBtn.textContent = 'Compare Selected (' + count + ')';
    compareBtn.disabled = count !== 2;
  }

  function openCompareModal() {
    if (selectedSnaps.length !== 2) return;

    var snaps = window.S && window.S.snaps;
    if (!snaps || !Array.isArray(snaps)) return;

    var snap1 = snaps.find(function(s) { return s && (s.id === selectedSnaps[0] || s.label === selectedSnaps[0]); });
    var snap2 = snaps.find(function(s) { return s && (s.id === selectedSnaps[1] || s.label === selectedSnaps[1]); });

    if (!snap1 || !snap2) return;

    buildCompareModal(snap1, snap2);
  }

  function buildCompareModal(snap1, snap2) {
    if (compareModal) compareModal.remove();

    compareModal = document.createElement('div');
    compareModal.id = 'rvt-compare-modal';
    compareModal.setAttribute('role', 'dialog');
    compareModal.setAttribute('aria-modal', 'true');
    compareModal.setAttribute('aria-label', 'Snapshot comparison');

    var container = document.createElement('div');
    container.className = 'rvt-compare-container';

    var header = document.createElement('div');
    header.className = 'rvt-compare-header';
    header.innerHTML = '<h2 class="rvt-compare-title">Snapshot Comparison</h2>';
    var closeBtn = document.createElement('button');
    closeBtn.className = 'rvt-compare-close';
    closeBtn.textContent = '✕';
    closeBtn.onclick = function() { compareModal.classList.remove('rvt-compare-visible'); };
    header.appendChild(closeBtn);
    container.appendChild(header);

    /* KPI comparison */
    var kpiGrid = document.createElement('div');
    kpiGrid.className = 'rvt-compare-grid';

    [snap1, snap2].forEach(function(snap, idx) {
      var side = document.createElement('div');
      side.className = 'rvt-compare-side';
      side.innerHTML = '<p class="rvt-compare-side-label">' + (snap.label || 'Snapshot ' + (idx + 1)) + ' (' + (snap.ts || 'N/A') + ')</p>';

      var table = document.createElement('table');
      table.className = 'rvt-compare-kpi-table';
      table.innerHTML = '<thead><tr><th>Metric</th><th>Value</th></tr></thead><tbody>';

      /* Extract KPI values */
      var kpis = snap.kpis || snap.data || {};
      Object.keys(kpis).slice(0, 5).forEach(function(key) {
        var val = kpis[key];
        table.innerHTML += '<tr><td>' + key + '</td><td>' + (typeof val === 'number' ? val.toFixed(2) : val) + '</td></tr>';
      });

      table.innerHTML += '</tbody>';
      side.appendChild(table);
      kpiGrid.appendChild(side);
    });

    container.appendChild(kpiGrid);

    /* Delta column (if data available) */
    if ((snap1.kpis || snap1.data) && (snap2.kpis || snap2.data)) {
      var deltaTable = document.createElement('table');
      deltaTable.className = 'rvt-compare-kpi-table';
      deltaTable.innerHTML = '<thead><tr><th>Metric</th><th>Δ (Snap2 - Snap1)</th></tr></thead><tbody>';

      var kpis1 = snap1.kpis || snap1.data || {};
      var kpis2 = snap2.kpis || snap2.data || {};
      Object.keys(kpis1).slice(0, 5).forEach(function(key) {
        var v1 = parseFloat(kpis1[key]) || 0;
        var v2 = parseFloat(kpis2[key]) || 0;
        var delta = (v2 - v1).toFixed(2);
        var deltaClass = delta > 0 ? 'increase' : (delta < 0 ? 'decrease' : '');
        deltaTable.innerHTML += '<tr><td>' + key + '</td><td class="rvt-compare-delta ' + deltaClass + '">' + delta + '</td></tr>';
      });

      deltaTable.innerHTML += '</tbody>';
      container.appendChild(deltaTable);
    }

    /* Actions */
    var actions = document.createElement('div');
    actions.className = 'rvt-compare-actions';
    var exportBtn = document.createElement('button');
    exportBtn.className = 'rvt-compare-export-btn';
    exportBtn.textContent = 'Export Comparison';
    exportBtn.onclick = function() { exportComparison(snap1, snap2); };
    actions.appendChild(exportBtn);
    container.appendChild(actions);

    compareModal.appendChild(container);
    document.body.appendChild(compareModal);
    compareModal.classList.add('rvt-compare-visible');

    compareModal.onclick = function(e) {
      if (e.target === compareModal) compareModal.classList.remove('rvt-compare-visible');
    };
  }

  function exportComparison(snap1, snap2) {
    /* Generate CSV/text export */
    var content = 'Snapshot Comparison\n\n';
    content += 'Snapshot 1: ' + (snap1.label || 'N/A') + ' (' + (snap1.ts || 'N/A') + ')\n';
    content += 'Snapshot 2: ' + (snap2.label || 'N/A') + ' (' + (snap2.ts || 'N/A') + ')\n\n';

    content += 'Metric,Snap1,Snap2,Delta\n';
    var kpis1 = snap1.kpis || snap1.data || {};
    var kpis2 = snap2.kpis || snap2.data || {};
    Object.keys(kpis1).forEach(function(key) {
      var v1 = kpis1[key];
      var v2 = kpis2[key] || 'N/A';
      var delta = (typeof v1 === 'number' && typeof v2 === 'number') ? (v2 - v1).toFixed(2) : 'N/A';
      content += key + ',' + v1 + ',' + v2 + ',' + delta + '\n';
    });

    var blob = new Blob([content], { type: 'text/csv' });
    var url = URL.createObjectURL(blob);
    var a = document.createElement('a');
    a.href = url;
    a.download = 'comparison_' + Date.now() + '.csv';
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  }

  function setupMultiSelectToggle() {
    var snapsList = document.querySelector('[data-view="snapshots"] .snaps-list, .snaps-grid-container, [data-snap-list]');
    if (!snapsList) return;

    /* Add shortcut: Ctrl+Shift+M to toggle multi-select */
    document.addEventListener('keydown', function(e) {
      if (e.ctrlKey && e.shiftKey && (e.key === 'M' || e.key === 'm')) {
        e.preventDefault();
        multiSelectMode ? disableMultiSelect() : enableMultiSelect();
      }
    });

    /* Click on card to select in multi-select mode */
    snapsList.addEventListener('click', function(e) {
      if (!multiSelectMode) return;
      var card = e.target.closest('[data-snap-index]');
      if (card) toggleSnapSelection(card);
    });
  }

  function boot() {
    setupMultiSelectToggle();

    window.rvtSnapCompare = {
      enableMultiSelect: enableMultiSelect,
      disableMultiSelect: disableMultiSelect,
      getSelected: function() { return selectedSnaps.slice(); },
      openCompare: openCompareModal
    };

    window.rvtSnapCompareReady = true;
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', function(){ setTimeout(boot, 1200); }, {once:true});
  } else {
    setTimeout(boot, 1200);
  }
})();
