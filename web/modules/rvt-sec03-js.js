(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function textBytes(s){return new TextEncoder().encode(s);}
  function hex(buf){return Array.from(new Uint8Array(buf)).map(function(b){return b.toString(16).padStart(2,'0');}).join('');}
  function fallbackHash(s){var h=0;for(var i=0;i<s.length;i++)h=((h<<5)-h+s.charCodeAt(i))|0;return ('00000000'+(h>>>0).toString(16)).slice(-8);}
  function hashEntry(entry,prevHash){var s=JSON.stringify(Object.assign({},entry,{prevHash:prevHash||''}));if(window.crypto&&crypto.subtle&&crypto.subtle.digest)return crypto.subtle.digest('SHA-256',textBytes(s)).then(hex);return Promise.resolve(fallbackHash(s));}
  function entries(){return (window.S&&window.S.auditLog)||R.parseJson('rvt-audit-log',[],Array.isArray);}
  async function verify(){var prev='',list=entries();for(var i=0;i<list.length;i++){var e=list[i],expected=await hashEntry(Object.assign({},e,{hash:undefined,prevHash:undefined}),prev);if(e.prevHash&&e.prevHash!==prev)return {ok:false,breakAt:e.timestamp||i};if(e.hash&&e.hash!==expected)return {ok:false,breakAt:e.timestamp||i};prev=e.hash||expected;}return {ok:true,breakAt:null,count:list.length};}
  function getChainHead(){var a=entries();return a.length?(a[a.length-1].hash||a[a.length-1].prevHash||''):'';}
  function render(){var host=document.querySelector('#settings-data-privacy,[data-settings-section="privacy"],#view-settings')||document.body;var b=R.ensure('rvt-sec03-verify','button',host);b.className='rvt-gd-button';b.textContent=R.t('gd.auditVerify');b.onclick=function(){verify().then(function(r){R.toast(r.ok?R.t('gd.auditPass',{count:r.count||0}):R.t('gd.auditFail',{timestamp:r.breakAt}));});};}
  window.rvtSec03={hashEntry:hashEntry,verify:verify,getChainHead:getChainHead};window.rvtAuditTamper=window.rvtSec03;R.boot(render);
})();
