/* A11Y-03: Data-table Alternative for Canvases */
(function() {
  'use strict';

  var TABLE_CLASS = 'rvt-a11y-chart-table';
  var SHOW_CLASS  = 'rvt-a11y-show-tables';
  var tablesVisible = false;

  function formatVal(v) {
    if (v == null) return '—';
    if (typeof v === 'number') return isNaN(v) ? '—' : v.toFixed(1);
    if (typeof v === 'object' && v !== null && 'y' in v) return formatVal(v.y);
    return String(v);
  }

  function buildOrUpdateTable(chartKey, chart) {
    if (!chart || !chart.canvas) return;
    var canvas = chart.canvas;
    var tableId = 'rvt-a11y-tbl-' + chartKey;
    var table = document.getElementById(tableId);

    if (!table) {
      table = document.createElement('table');
      table.id = tableId;
      table.className = TABLE_CLASS;
      /* insert immediately after canvas wrapper or canvas itself */
      var anchor = canvas.parentElement || canvas;
      anchor.insertAdjacentElement('afterend', table);
    }

    var data = chart.data || {};
    var datasets = data.datasets || [];
    var labels   = data.labels || [];
    var label    = canvas.getAttribute('aria-label') || chartKey;

    /* Only show last 10 labels to keep table manageable */
    var startIdx = Math.max(0, labels.length - 10);
    var slicedLabels = labels.slice(startIdx);

    var html = '<caption>' + label + '</caption>';
    if (datasets.length) {
      html += '<thead><tr><th scope="col">Time</th>';
      datasets.forEach(function(ds){ html += '<th scope="col">' + (ds.label || 'Value') + '</th>'; });
      html += '</tr></thead><tbody>';
      slicedLabels.forEach(function(lbl, i) {
        var absIdx = startIdx + i;
        html += '<tr><th scope="row">' + (lbl || absIdx) + '</th>';
        datasets.forEach(function(ds){
          html += '<td>' + formatVal(ds.data && ds.data[absIdx]) + '</td>';
        });
        html += '</tr>';
      });
      html += '</tbody>';
    } else {
      html += '<tbody><tr><td>No data</td></tr></tbody>';
    }

    table.innerHTML = html;
    table.setAttribute('aria-label', label + ' data table');
  }

  function updateAllTables() {
    var charts = window.S && window.S.charts;
    if (!charts) return;
    Object.keys(charts).forEach(function(k){
      var ch = charts[k];
      if (ch && !ch.isDestroyed) buildOrUpdateTable(k, ch);
    });
  }

  function toggleTableVisibility(force) {
    tablesVisible = (force !== undefined) ? !!force : !tablesVisible;
    if (tablesVisible) {
      document.body.classList.add(SHOW_CLASS);
    } else {
      document.body.classList.remove(SHOW_CLASS);
    }
    return tablesVisible;
  }

  function boot() {
    /* Update tables on a 1-second cadence during live mode */
    setInterval(updateAllTables, 1000);

    window.rvtA11yCanvasTables = {
      update: updateAllTables,
      show:   function(){ return toggleTableVisibility(true); },
      hide:   function(){ return toggleTableVisibility(false); },
      toggle: function(){ return toggleTableVisibility(); },
      isVisible: function(){ return tablesVisible; }
    };
  }

  if (document.readyState==='loading') {
    document.addEventListener('DOMContentLoaded', function(){ setTimeout(boot,600); }, {once:true});
  } else { setTimeout(boot,600); }
})();
