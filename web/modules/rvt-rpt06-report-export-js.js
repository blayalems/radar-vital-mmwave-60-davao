(function(){
  'use strict';
  var U=window.rvtGammaBatch2a;if(!U)return;
  var B=window.rvtGammaBatch2b;
  function t(k,v){return B?B.t(k,v):U.t(k,v);}
  function renderExportButton(){
    var report=document.getElementById('view-report')||document.querySelector('[data-view="report"]');if(!report||document.getElementById('rvtReportExportWrap'))return;
    var wrap=document.createElement('div');wrap.className='rvt-report-export-wrap';wrap.id='rvtReportExportWrap';
    wrap.innerHTML='<button type="button" class="rvt-report-export-btn" id="rvtReportExportBtn" aria-label="'+U.escapeHtml(t('gamma.rpt06.export'))+'"><span class="material-symbols-rounded" aria-hidden="true">print</span><span>'+U.escapeHtml(t('gamma.rpt06.export'))+'</span></button>';
    var hero=report.querySelector('.verdict-hero');var target=hero?hero.nextElementSibling:report.firstChild;
    report.insertBefore(wrap,target);
    document.getElementById('rvtReportExportBtn').addEventListener('click',function(){window.rvtReportExport.print();});
  }
  window.rvtReportExport={print:function(){
    U.toast(t('gamma.rpt06.printing'));
    setTimeout(function(){window.print();},150);
  }};
  U.bootSoon(renderExportButton);
})();
