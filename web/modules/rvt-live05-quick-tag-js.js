(function(){
  'use strict';
  var R=window.rvtGammaBatch2bRuntime;if(!R)return;
  var map={M:'motion',C:'cough',S:'speaking',B:'baseline'};
  function notes(){return document.getElementById('notes')||document.getElementById('sessionNotesText')||document.querySelector('textarea[aria-label*="notes" i], textarea');}
  function active(){var a=document.activeElement;return!(a&&(a.tagName==='TEXTAREA'||a.tagName==='INPUT'||a.isContentEditable));}
  function stamp(){return new Date().toTimeString().slice(0,8);}
  function tag(type){var n=notes();if(!n)return false;var time=stamp();var line='['+time+'] '+type;var prefix=n.value&&n.value.slice(-1)!=='\n'?'\n':'';n.value+=prefix+line;n.dispatchEvent(new Event('input',{bubbles:true}));R.toast(R.t('gamma2b.live.tagged',{type:type,time:time}));return line;}
  document.addEventListener('keydown',function(e){if(e.ctrlKey||e.metaKey||e.altKey||!active())return;var key=String(e.key||'').toUpperCase();if(map[key]){e.preventDefault();e.stopImmediatePropagation();tag(map[key]);}},true);
  window.rvtQuickTag={tag:tag,getShortcuts:function(){return Object.assign({},map);},isActive:active};
})();
