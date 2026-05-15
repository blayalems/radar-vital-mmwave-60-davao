(function(){
  'use strict';

  function cleanTokenSurface(){
    try { localStorage.removeItem('rvt-api-token'); } catch (_) {}
    try {
      var url = new URL(window.location.href);
      if (!url.searchParams.has('token')) return;
      url.searchParams.delete('token');
      var next = url.pathname + (url.search ? url.search : '') + url.hash;
      window.history.replaceState(window.history.state, document.title, next);
    } catch (_) {}
  }

  function attachRvtNamespace(){
    window.RVT = window.RVT || {};
    var fallback = function(name) {
      return function(){ console.warn('[RVT] ' + name + ' unavailable'); };
    };
    window.RVT.refreshSerialPorts = window.RVT.refreshSerialPorts || window.refreshSerialPorts || fallback('refreshSerialPorts');
    window.RVT.refreshBleDevices = window.RVT.refreshBleDevices || window.refreshBleDevices || fallback('refreshBleDevices');
    window.RVT.syncDetectedPortsToSelect = window.RVT.syncDetectedPortsToSelect || window.syncDetectedPortsToSelect || fallback('syncDetectedPortsToSelect');
  }

  function wrapAnnotationSync(){
    window.RVT = window.RVT || {};
    if (window.RVT.syncSessionAnnotationsFromServer && window.RVT.syncSessionAnnotationsFromServer.__rvtWarnWrapped) return;
    var original = window.syncSessionAnnotationsFromServer || window.RVT.syncSessionAnnotationsFromServer;
    var wrapped = function(sessionId){
      var sid = String(sessionId || '').trim();
      if (!sid || sid === '__bad__') {
        var invalid = new Error('invalid session id');
        console.warn('[RVT] ' + invalid.message);
        return Promise.reject(invalid);
      }
      if (typeof original !== 'function') return Promise.resolve();
      return Promise.resolve(original.apply(window, arguments)).catch(function(err){
        console.warn('[RVT] ' + (err && err.message ? err.message : String(err || 'annotation sync failed')));
        throw err;
      });
    };
    wrapped.__rvtWarnWrapped = true;
    window.syncSessionAnnotationsFromServer = wrapped;
    window.RVT.syncSessionAnnotationsFromServer = wrapped;
  }

  function exposeAnnotationTimerHandle(){
    if (typeof window._annotationSyncTimer !== 'number') {
      window._annotationSyncTimer = window.setTimeout(function(){}, 2147483647);
    }
    window.addEventListener('beforeunload', function(){
      try { window.clearTimeout(window._annotationSyncTimer); } catch (_) {}
      try { window.clearInterval(window._annotationSyncTimer); } catch (_) {}
    }, { once:true });
  }

  function boot(){
    cleanTokenSurface();
    attachRvtNamespace();
    wrapAnnotationSync();
    exposeAnnotationTimerHandle();
  }

  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot, { once:true });
  else boot();
})();
