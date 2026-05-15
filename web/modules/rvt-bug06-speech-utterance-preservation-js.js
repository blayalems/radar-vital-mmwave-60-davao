/* Repair: BUG-06 speech debounce wrapper destroys utterance settings.
   The earlier wrapper extracted only `utterance.text`, then constructed a brand-new
   utterance inside `emit()`. This dropped voice/lang/pitch/rate/volume that callers
   had configured (e.g., voiceReadout's rate=1.1 / pitch=1.0 / volume=0.8).
   Fix: queue the original utterance reference and pass it through verbatim.
   Also re-expose `getLastSpoken` / `getQueueLength` so they reflect the real state.
*/
(function(){
  'use strict';
  var synth = window.speechSynthesis;
  if (!synth || synth.__rvtSpeechPreserved) return;
  var current = synth.speak;
  if (typeof current !== 'function') return;
  var lastSpoken = { text: '', timestamp: 0 };
  var lastEndAt = 0;
  var queued = null;
  var queueTimer = 0;
  var nativeCancel = synth.cancel ? synth.cancel.bind(synth) : function(){};
  var prior = current.bind(synth);
  function shouldSuppress(text, priority, now) {
    var ms = priority === 'critical' ? 1000 : 3000;
    return lastSpoken.text === text && now - lastSpoken.timestamp < ms;
  }
  function emit(utterance, priority) {
    queued = null;
    if (queueTimer) { clearTimeout(queueTimer); queueTimer = 0; }
    nativeCancel();
    utterance.__rvtDebounceBypass = true;
    utterance.__rvtPriority = priority || 'normal';
    var prevOnEnd = utterance.onend;
    var prevOnError = utterance.onerror;
    utterance.onend = function(ev) {
      lastEndAt = Date.now();
      if (typeof prevOnEnd === 'function') try { prevOnEnd.call(this, ev); } catch (_) {}
    };
    utterance.onerror = function(ev) {
      lastEndAt = Date.now();
      if (typeof prevOnError === 'function') try { prevOnError.call(this, ev); } catch (_) {}
    };
    lastSpoken = { text: utterance.text || '', timestamp: Date.now() };
    prior(utterance);
  }
  synth.speak = function(utterance) {
    if (!utterance) return;
    if (utterance.__rvtDebounceBypass) return prior(utterance);
    var text = (typeof utterance.text === 'string' ? utterance.text : String(utterance || '')).trim();
    var priority = utterance.__rvtPriority === 'critical' ? 'critical' : 'normal';
    if (!text) return;
    var now = Date.now();
    if (shouldSuppress(text, priority, now)) return;
    if (priority === 'critical') { emit(utterance, priority); return; }
    var wait = Math.max(0, 1000 - (now - lastEndAt));
    if (wait > 0) {
      if (queueTimer) clearTimeout(queueTimer);
      queued = utterance;
      queueTimer = setTimeout(function(){ var q = queued; if (q) emit(q, priority); }, wait);
      return;
    }
    emit(utterance, priority);
  };
  if (window.SpeechSynthesisUtterance) {
    var api = window.rvtSpeechDebounce || (window.rvtSpeechDebounce = {});
    api.speak = function(text, priority) {
      try {
        var utt = new window.SpeechSynthesisUtterance(String(text == null ? '' : text));
        utt.__rvtPriority = priority === 'critical' ? 'critical' : 'normal';
        synth.speak(utt);
      } catch (e) {
        if (typeof window.rvtSoftError === 'function') window.rvtSoftError('speech debounce speak failed', e);
      }
    };
    api.cancel = function() {
      queued = null;
      if (queueTimer) { clearTimeout(queueTimer); queueTimer = 0; }
      nativeCancel();
    };
    api.getLastSpoken = function() { return { text: lastSpoken.text, timestamp: lastSpoken.timestamp }; };
    api.getQueueLength = function() { return queued ? 1 : 0; };
  }
  synth.__rvtSpeechPreserved = true;
})();
