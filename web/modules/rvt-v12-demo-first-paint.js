    (function(){
      try {
        var banner = document.getElementById('demoBanner');
        if (document.documentElement.dataset.sandbox === '1' && banner) banner.hidden = false;
      } catch (_) {}
    })();
