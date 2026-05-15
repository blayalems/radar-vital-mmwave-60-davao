(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function cards(){return document.querySelectorAll('.diagnostic-card,[data-diagnostic-card],.report-diagnostic-card');}
  function applyTints(){cards().forEach(function(c){var v=(c.dataset.verdict||c.dataset.status||c.textContent||'').toLowerCase();c.classList.add('rvt-diagnostic-card-tinted');c.style.backgroundColor=v.indexOf('fail')>=0?'color-mix(in srgb,var(--bad,#b83030) 6%,var(--surface-1,#fff))':v.indexOf('warn')>=0?'color-mix(in srgb,var(--warn,#b7791f) 6%,var(--surface-1,#fff))':'color-mix(in srgb,var(--ok,#1f8f5f) 6%,var(--surface-1,#fff))';});}
  function removeTints(){cards().forEach(function(c){c.classList.remove('rvt-diagnostic-card-tinted');c.style.backgroundColor='';});}
  window.rvtRpt07={applyTints:applyTints,removeTints:removeTints};window.rvtTintedCards=window.rvtRpt07;R.boot(applyTints);
})();
