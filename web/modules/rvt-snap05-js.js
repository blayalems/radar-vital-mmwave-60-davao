(function(){'use strict';var R=window.rvtGammaDelta;if(!R)return;
  function htmlEsc(s){return String(s||'').replace(/[&<>"']/g,function(c){return {'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c];});}
  function escape(source){return htmlEsc(source).replace(/`/g,'&#96;').replace(/#/g,'&#35;');}
  function render(src){var s=escape(src);var lines=s.split(/\r?\n/),out=[],inList=false;lines.forEach(function(l){var m=l.match(/^\s*-\s+(.+)/);if(m){if(!inList){out.push('<ul>');inList=true;}out.push('<li>'+m[1].replace(/\*\*([^*]+)\*\*/g,'<strong>$1</strong>').replace(/\*([^*]+)\*/g,'<em>$1</em>')+'</li>');}else{if(inList){out.push('</ul>');inList=false;}out.push(l.replace(/\*\*([^*]+)\*\*/g,'<strong>$1</strong>').replace(/\*([^*]+)\*/g,'<em>$1</em>'));}});if(inList)out.push('</ul>');return out.join('\n');}
  window.rvtSnapMarkdown={render:render,escape:escape,isMarkdown:function(s){return /(\*\*[^*]+\*\*|\*[^*]+\*|^\s*-\s+)/m.test(String(s||''));}};
})();
