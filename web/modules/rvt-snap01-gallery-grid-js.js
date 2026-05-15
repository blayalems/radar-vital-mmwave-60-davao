(function() {
  var VIEW_MODE_KEY = 'rvt-snap-view-mode';
  var currentViewMode = 'grid';
  var snapsList = null;
  var toggleBtn = null;
  var observer = null;

  function getSavedViewMode() {
    try {
      var saved = localStorage.getItem(VIEW_MODE_KEY);
      return (saved === 'list' || saved === 'grid') ? saved : 'grid';
    } catch(e) { return 'grid'; }
  }

  function saveViewMode(mode) {
    try { localStorage.setItem(VIEW_MODE_KEY, mode); } catch(e) {}
  }

  function setViewMode(mode) {
    if (!snapsList || (mode !== 'list' && mode !== 'grid')) return;
    currentViewMode = mode;

    snapsList.classList.remove('rvt-snap-gallery-grid', 'rvt-snap-gallery-list');
    snapsList.classList.add(mode === 'grid' ? 'rvt-snap-gallery-grid' : 'rvt-snap-gallery-list');

    if (toggleBtn) {
      toggleBtn.classList.remove('rvt-snap-grid-mode', 'rvt-snap-list-mode');
      toggleBtn.classList.add(mode === 'grid' ? 'rvt-snap-grid-mode' : 'rvt-snap-list-mode');
      toggleBtn.setAttribute('aria-label', mode === 'grid' ? 'Switch to list view' : 'Switch to grid view');
      toggleBtn.setAttribute('title', mode === 'grid' ? 'Switch to list view' : 'Switch to grid view');
    }

    saveViewMode(mode);
  }

  function updateGridColumns() {
    if (!snapsList) return;
    var w = window.innerWidth;
    if (currentViewMode === 'grid') {
      if (w >= 1280) {
        snapsList.style.gridTemplateColumns = 'repeat(4, 1fr)';
      } else if (w >= 768) {
        snapsList.style.gridTemplateColumns = 'repeat(2, 1fr)';
      } else {
        snapsList.style.gridTemplateColumns = '1fr';
      }
    }
  }

  function ensureToggleButton() {
    if (toggleBtn) return;

    var header = document.querySelector('[data-view="snapshots"] .view-header, .snapshots-header, [data-snap-list-header]');
    if (!header) {
      /* fallback: find snapshots container and prepend button */
      var cont = document.querySelector('[data-view="snapshots"]');
      if (!cont) return;
      header = cont;
    }

    toggleBtn = document.createElement('button');
    toggleBtn.id = 'rvt-snap-view-toggle';
    toggleBtn.type = 'button';
    toggleBtn.className = currentViewMode === 'grid' ? 'rvt-snap-grid-mode' : 'rvt-snap-list-mode';
    toggleBtn.setAttribute('aria-label', currentViewMode === 'grid' ? 'Switch to list view' : 'Switch to grid view');
    toggleBtn.setAttribute('title', currentViewMode === 'grid' ? 'Switch to list view' : 'Switch to grid view');

    toggleBtn.onclick = function(e) {
      e.preventDefault();
      setViewMode(currentViewMode === 'grid' ? 'list' : 'grid');
    };

    header.appendChild(toggleBtn);
  }

  function installKeyboardNav() {
    if (!snapsList) return;

    snapsList.addEventListener('keydown', function(e) {
      if (e.key === 'l' || e.key === 'L') {
        e.preventDefault();
        setViewMode(currentViewMode === 'grid' ? 'list' : 'grid');
        return;
      }

      if (currentViewMode !== 'grid') return;

      var target = e.target.closest('[data-snap-index]');
      if (!target) return;

      var idx = parseInt(target.getAttribute('data-snap-index'), 10) || 0;
      var items = Array.from(snapsList.querySelectorAll('[data-snap-index]'));
      var pos = items.indexOf(target);
      if (pos < 0) return;

      var cols = getComputedStyle(snapsList).getPropertyValue('grid-template-columns').split(' ').length;
      var next = null;

      switch(e.key) {
        case 'ArrowRight': next = items[pos+1]; break;
        case 'ArrowLeft':  next = items[pos-1]; break;
        case 'ArrowDown':  next = items[Math.min(pos + cols, items.length-1)]; break;
        case 'ArrowUp':    next = items[Math.max(pos - cols, 0)]; break;
        case 'Home':       next = items[0]; break;
        case 'End':        next = items[items.length-1]; break;
      }

      if (next) {
        e.preventDefault();
        next.focus();
      }
    });
  }

  function setupSnapshotIndexing() {
    if (!snapsList) return;
    var items = snapsList.querySelectorAll('[role="button"], .snap-item, [data-snap-id]');
    items.forEach(function(el, idx) {
      el.setAttribute('data-snap-index', idx);
      if (!el.hasAttribute('tabindex')) el.setAttribute('tabindex', '0');
    });
  }

  function onSnapshotsElementReady() {
    snapsList = document.querySelector('[data-view="snapshots"] .snaps-list, .snaps-grid-container, [data-snap-list]');
    if (!snapsList) return false;

    setupSnapshotIndexing();
    ensureToggleButton();
    setViewMode(getSavedViewMode());
    installKeyboardNav();

    /* hook MutationObserver for dynamic snapshot additions */
    if (!observer) {
      var cb = function(muts) {
        var needsReindex = false;
        for (var i = 0; i < muts.length; i++) {
          if (muts[i].type === 'childList') { needsReindex = true; break; }
        }
        if (needsReindex) setupSnapshotIndexing();
      };
      observer = new MutationObserver(cb);
      if (window.rvtTrackMutationObserver) {
        window.rvtTrackMutationObserver(observer);
      }
      observer.observe(snapsList, { childList: true });
    }

    return true;
  }

  /* Export API immediately, initialization happens on demand */
  window.rvtSnapGallery = {
    setMode: function(m) {
      if (!snapsList) onSnapshotsElementReady();
      setViewMode(m);
    },
    getMode: function() { return currentViewMode; },
    updateColumns: function() {
      if (!snapsList) onSnapshotsElementReady();
      updateGridColumns();
    }
  };

  function boot() {
    if (!onSnapshotsElementReady()) {
      setTimeout(boot, 300);
      return;
    }
    window.addEventListener('resize', updateGridColumns);
    window.rvtSnapGalleryReady = true;
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', function(){ setTimeout(boot, 800); }, {once:true});
  } else {
    setTimeout(boot, 800);
  }
})();
