    window.RVT_BP = { sm:480, md:768, lg:1024, xl:1280 };
    (function(){
      try {
        if ((navigator.deviceMemory ?? 8) < 4 || navigator.hardwareConcurrency <= 2) {
          document.documentElement.setAttribute('data-low-end', '');
        }
        var qs = new URLSearchParams(location.search || '');
        var hinted = qs.get('demo') === '1' || qs.get('sandbox') === '1' || localStorage.getItem('rvt-demo-mode') === '1';
        if (hinted) document.documentElement.dataset.sandbox = '1';
      } catch (_) {}
    })();
