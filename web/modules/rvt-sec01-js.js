(function(){
  'use strict';
  var CHECKPOINT_STORE = 'settings-backup';
  var SALT_KEY = 'rvt-key-salt';
  var WARN_KEY = 'rvt-enc-warned';
  var STRINGS = {
    'sec01.reencrypt':'Re-encrypt all sessions',
    'sec01.reencryptDone':'Session data re-encrypted',
    'sec01.unavailable':'Encryption unavailable - data stored without encryption. Use HTTPS for full security.',
    'sec01.authFailed':'Authentication failed - could not derive encryption key',
    'sec01.decryptFailed':'Could not decrypt session data - key may have changed',
    'sec01.rotationFailed':'Key rotation failed - operator not switched'
  };
  var keyRef = null;
  var lastCredential = '';
  var degraded = false;
  var lastTiming = null;
  var patchedIdb = false;
  var patchedUnlock = false;
  var patchedOperator = false;
  var origIdb = null;
  var pendingUnlockCredential = '';

  function extendStrings(){
    window.RVT_STRINGS = window.RVT_STRINGS || { en: {} };
    window.RVT_STRINGS.en = window.RVT_STRINGS.en || {};
    Object.keys(STRINGS).forEach(function(k){
      if (!(k in window.RVT_STRINGS.en)) window.RVT_STRINGS.en[k] = STRINGS[k];
    });
  }
  function t(key, vars){
    extendStrings();
    var s = typeof window.rvtT === 'function' ? window.rvtT(key, vars || {}) : (window.RVT_STRINGS.en[key] || key);
    Object.keys(vars || {}).forEach(function(k){ s = String(s).replace(new RegExp('\\{'+k+'\\}','g'), String(vars[k])); });
    return s;
  }
  function toast(msg){
    if (typeof window.toast === 'function') window.toast(msg);
    else if (typeof window.showToast === 'function') window.showToast(msg);
    else console.warn('[RVT-SEC01]', msg);
  }
  function registerManifest(){
    var m = window.rvtStorage && window.rvtStorage.manifest;
    if (!m) return;
    if (!m[SALT_KEY]) m[SALT_KEY] = { kind:'scalar', defaultValue:'', validate:function(v){ return typeof v === 'string'; } };
    if (!m[WARN_KEY]) m[WARN_KEY] = { kind:'scalar', defaultValue:'false', validate:function(v){ return v === 'true' || v === 'false'; } };
  }
  function hasCrypto(){
    if (window.__rvtSec01ForceNoCrypto) return false;
    return !!(window.crypto && window.crypto.subtle && window.crypto.getRandomValues &&
      crypto.subtle.importKey && crypto.subtle.deriveKey && crypto.subtle.encrypt && crypto.subtle.decrypt);
  }
  function warnUnavailableOnce(){
    degraded = true;
    try {
      if (localStorage.getItem(WARN_KEY) !== 'true') {
        toast(t('sec01.unavailable'));
        localStorage.setItem(WARN_KEY, 'true');
      }
    } catch (_) {}
  }
  function bytesToBase64(bytes){
    var s = '';
    new Uint8Array(bytes).forEach(function(b){ s += String.fromCharCode(b); });
    return btoa(s);
  }
  function base64ToBytes(s){
    var raw = atob(String(s || ''));
    var out = new Uint8Array(raw.length);
    for (var i = 0; i < raw.length; i++) out[i] = raw.charCodeAt(i);
    return out;
  }
  function concatIvCipher(iv, cipher){
    var a = new Uint8Array(iv);
    var b = new Uint8Array(cipher);
    var out = new Uint8Array(a.length + b.length);
    out.set(a, 0);
    out.set(b, a.length);
    return out.buffer;
  }
  function getSalt(){
    var salt = '';
    try { salt = localStorage.getItem(SALT_KEY) || ''; } catch (_) {}
    if (!salt) {
      var bytes = new Uint8Array(16);
      if (window.crypto && window.crypto.getRandomValues) crypto.getRandomValues(bytes);
      else for (var i = 0; i < bytes.length; i++) bytes[i] = Math.floor(Math.random() * 256);
      salt = bytesToBase64(bytes);
      try { localStorage.setItem(SALT_KEY, salt); } catch (_) {}
    }
    return salt;
  }
  async function derive(credential, saltB64){
    if (!hasCrypto()) { warnUnavailableOnce(); return null; }
    var source = String(credential || '');
    if (!source) throw new Error('missing credential');
    var material = await crypto.subtle.importKey('raw', new TextEncoder().encode(source), 'PBKDF2', false, ['deriveKey']);
    return crypto.subtle.deriveKey(
      { name:'PBKDF2', salt:base64ToBytes(saltB64 || getSalt()), iterations:100000, hash:'SHA-256' },
      material,
      { name:'AES-GCM', length:256 },
      false,
      ['encrypt','decrypt']
    );
  }
  async function loadKey(credential){
    try {
      var k = await derive(credential, getSalt());
      keyRef = k;
      lastCredential = String(credential || '');
      degraded = !k;
      return;
    } catch (err) {
      keyRef = null;
      toast(t('sec01.authFailed'));
      throw err;
    }
  }
  function isKeyLoaded(){ return !!keyRef; }
  function isEncrypted(){ return hasCrypto() && !!keyRef && !degraded; }
  function devMode(){
    try { return new URLSearchParams(location.search).get('debug') === 'sec' || localStorage.getItem('rvt-dev-mode') === 'true'; }
    catch (_) { return false; }
  }
  async function encryptWithKey(obj, k){
    var iv = new Uint8Array(12);
    crypto.getRandomValues(iv);
    var plain = new TextEncoder().encode(JSON.stringify(obj));
    var cipher = await crypto.subtle.encrypt({ name:'AES-GCM', iv:iv }, k, plain);
    return concatIvCipher(iv, cipher);
  }
  async function decryptWithKey(buffer, k){
    var bytes = buffer instanceof ArrayBuffer ? new Uint8Array(buffer) : new Uint8Array(buffer.buffer || buffer);
    var iv = bytes.slice(0, 12);
    var cipher = bytes.slice(12);
    var plain = await crypto.subtle.decrypt({ name:'AES-GCM', iv:iv }, k, cipher);
    return JSON.parse(new TextDecoder().decode(plain));
  }
  async function encrypt(obj){
    if (!isEncrypted()) return obj;
    return encryptWithKey(obj, keyRef);
  }
  async function decrypt(buffer){
    if (!isEncrypted()) return buffer;
    try {
      return await decryptWithKey(buffer, keyRef);
    } catch (err) {
      toast(t('sec01.decryptFailed'));
      throw err;
    }
  }
  function encryptedEnvelope(record, payload){
    return { id: record && record.id ? record.id : 'current-draft', __rvtSec01Encrypted:true, payload:payload, ts:Date.now() };
  }
  function isEnvelope(v){ return !!(v && v.__rvtSec01Encrypted && v.payload); }
  async function decryptRecord(record, k){
    if (isEnvelope(record)) {
      var obj = await decryptWithKey(record.payload, k || keyRef);
      if (obj && typeof obj === 'object' && !obj.id && record.id) obj.id = record.id;
      return obj;
    }
    if (record instanceof ArrayBuffer) return decryptWithKey(record, k || keyRef);
    return record;
  }
  async function encryptRecord(record, k){
    if (!k) return record;
    if (isEnvelope(record)) return record;
    var payload = await encryptWithKey(record, k);
    return encryptedEnvelope(record, payload);
  }
  function patchIdbApi(){
    if (patchedIdb || !window.rvtIDB) return !!patchedIdb;
    origIdb = {
      put: window.rvtIDB.put,
      get: window.rvtIDB.get,
      getAll: window.rvtIDB.getAll,
      open: window.rvtIDB.open
    };
    if (typeof origIdb.put !== 'function' || typeof origIdb.get !== 'function') return false;
    window.rvtIDB.put = async function(storeName, record){
      var started = performance.now();
      var out = record;
      if (storeName === CHECKPOINT_STORE && isEncrypted()) out = await encryptRecord(record, keyRef);
      var result = await origIdb.put.call(this, storeName, out);
      if (storeName === CHECKPOINT_STORE && isEncrypted()) {
        lastTiming = performance.now() - started;
        if (devMode()) console.info('[RVT-SEC01] encrypt+store '+lastTiming.toFixed(2)+' ms');
      }
      return result;
    };
    window.rvtIDB.get = async function(storeName, id){
      var result = await origIdb.get.call(this, storeName, id);
      if (storeName === CHECKPOINT_STORE && isEncrypted()) {
        try { return await decryptRecord(result, keyRef); }
        catch (_) { return null; }
      }
      return result;
    };
    window.rvtIDB.getAll = async function(storeName){
      var rows = await origIdb.getAll.call(this, storeName);
      if (storeName === CHECKPOINT_STORE && isEncrypted()) {
        var out = [];
        for (var i = 0; i < rows.length; i++) {
          try { out.push(await decryptRecord(rows[i], keyRef)); }
          catch (_) { out.push(null); }
        }
        return out;
      }
      return rows;
    };
    patchedIdb = true;
    return true;
  }
  function patchPrototypeNotice(){
    if (!window.IDBObjectStore || IDBObjectStore.prototype.__rvtSec01Patched) return;
    var origPut = IDBObjectStore.prototype.put;
    var origGet = IDBObjectStore.prototype.get;
    IDBObjectStore.prototype.put = function(value, key){
      if (this.name === CHECKPOINT_STORE && isEncrypted() && !(value && value.__rvtSec01Encrypted)) {
        console.warn('[RVT-SEC01] Direct checkpoint put bypassed async encryption; use rvtIDB.put for SEC-01 wrapping.');
      }
      return arguments.length > 1 ? origPut.call(this, value, key) : origPut.call(this, value);
    };
    IDBObjectStore.prototype.get = function(key){
      return origGet.call(this, key);
    };
    IDBObjectStore.prototype.__rvtSec01Patched = true;
  }
  async function rawRows(){
    patchIdbApi();
    if (!origIdb || !origIdb.getAll) return [];
    try { return await origIdb.getAll(CHECKPOINT_STORE); }
    catch (err) { console.warn('[RVT-SEC01] checkpoint store unavailable:', err); degraded = true; return []; }
  }
  async function writeRaw(record){
    patchIdbApi();
    if (!origIdb || !origIdb.put) return;
    return origIdb.put(CHECKPOINT_STORE, record);
  }
  async function rotateKey(newCredential){
    if (!hasCrypto()) { warnUnavailableOnce(); return { rotated:0, failed:0 }; }
    if (!keyRef) { await loadKey(newCredential); return { rotated:0, failed:0 }; }
    var oldKey = keyRef;
    var newKey = await derive(newCredential, getSalt());
    var rows = await rawRows();
    var rotated = 0, failed = 0;
    for (var i = 0; i < rows.length; i++) {
      try {
        var plain = await decryptRecord(rows[i], oldKey);
        var enc = await encryptRecord(plain, newKey);
        await writeRaw(enc);
        rotated++;
      } catch (err) {
        failed++;
        console.warn('[RVT-SEC01] rotate failed for record', rows[i] && rows[i].id, err);
      }
    }
    keyRef = newKey;
    lastCredential = String(newCredential || '');
    oldKey = null;
    return { rotated:rotated, failed:failed };
  }
  async function reencryptAll(){
    if (!lastCredential) throw new Error('credential required');
    if (!hasCrypto()) { warnUnavailableOnce(); return { reencrypted:0, failed:0 }; }
    var oldKey = keyRef;
    var rows = await rawRows();
    var bytes = new Uint8Array(16);
    crypto.getRandomValues(bytes);
    var newSalt = bytesToBase64(bytes);
    var newKey = await derive(lastCredential, newSalt);
    var reencrypted = 0, failed = 0;
    for (var i = 0; i < rows.length; i++) {
      try {
        var plain = oldKey ? await decryptRecord(rows[i], oldKey) : rows[i];
        var enc = await encryptRecord(plain, newKey);
        await writeRaw(enc);
        reencrypted++;
      } catch (err) {
        failed++;
        console.warn('[RVT-SEC01] reencrypt failed for record', rows[i] && rows[i].id, err);
      }
    }
    try { localStorage.setItem(SALT_KEY, newSalt); } catch (_) {}
    keyRef = newKey;
    oldKey = null;
    toast(t('sec01.reencryptDone'));
    return { reencrypted:reencrypted, failed:failed };
  }
  function clearKey(){ keyRef = null; }
  function patchOperatorSwitch(){
    if (patchedOperator || !window.rvtOperatorSwitch || typeof window.rvtOperatorSwitch.switchTo !== 'function') return false;
    var original = window.rvtOperatorSwitch.switchTo;
    window.rvtOperatorSwitch.switchTo = function(operatorName, credential){
      if (!isEncrypted()) return original.apply(this, arguments);
      var self = this, args = arguments;
      return rotateKey(credential || operatorName).then(function(res){
        if (res.failed) throw new Error('rotation failed');
        return original.apply(self, args);
      }).catch(function(err){
        toast(t('sec01.rotationFailed'));
        console.warn('[RVT-SEC01] operator switch aborted:', err);
        return window.rvtOperatorSwitch.getCurrent ? window.rvtOperatorSwitch.getCurrent() : null;
      });
    };
    patchedOperator = true;
    return true;
  }
  function patchUnlock(){
    if (patchedUnlock || !window.rvtAutoLock || typeof window.rvtAutoLock.unlock !== 'function') return false;
    var original = window.rvtAutoLock.unlock;
    window.rvtAutoLock.unlock = function(credential){
      var result = original.apply(this, arguments);
      var cred = credential || pendingUnlockCredential || (document.getElementById('rvtLockPinInput') || {}).value || '';
      if (cred && (!window.rvtAutoLock.isLocked || !window.rvtAutoLock.isLocked())) {
        Promise.resolve(loadKey(cred)).catch(function(){});
      }
      return result;
    };
    patchedUnlock = true;
    return true;
  }
  function hookUnlockDom(){
    document.addEventListener('click', function(e){
      if (!e.target || e.target.id !== 'rvtLockUnlockBtn') return;
      var input = document.getElementById('rvtLockPinInput');
      pendingUnlockCredential = input ? input.value : '';
      setTimeout(function(){
        if (pendingUnlockCredential && window.rvtAutoLock && !window.rvtAutoLock.isLocked()) {
          loadKey(pendingUnlockCredential).catch(function(){});
          pendingUnlockCredential = '';
        }
      }, 80);
    }, true);
    document.addEventListener('keydown', function(e){
      if (e.key !== 'Enter' || !e.target || e.target.id !== 'rvtLockPinInput') return;
      pendingUnlockCredential = e.target.value || '';
      setTimeout(function(){
        if (pendingUnlockCredential && window.rvtAutoLock && !window.rvtAutoLock.isLocked()) {
          loadKey(pendingUnlockCredential).catch(function(){});
          pendingUnlockCredential = '';
        }
      }, 80);
    }, true);
  }
  function renderUtility(){
    var host = document.querySelector('#settings-data-privacy,[data-settings-section="data-privacy"],[data-settings-section="privacy"],#view-settings') || document.body;
    var wrap = document.getElementById('rvt-sec01-actions');
    if (!wrap) {
      wrap = document.createElement('div');
      wrap.id = 'rvt-sec01-actions';
      wrap.className = 'rvt-sec01-actions';
      wrap.innerHTML = '<button id="rvt-sec01-reencrypt" class="rvt-sec01-btn" type="button"></button><span id="rvt-sec01-status" class="rvt-sec01-status" aria-live="polite"></span>';
      host.appendChild(wrap);
    }
    var btn = document.getElementById('rvt-sec01-reencrypt');
    var status = document.getElementById('rvt-sec01-status');
    if (btn) {
      btn.textContent = t('sec01.reencrypt');
      btn.onclick = function(){
        btn.disabled = true;
        reencryptAll().then(function(r){ if(status) status.textContent = r.reencrypted + ' re-encrypted, ' + r.failed + ' failed'; })
          .catch(function(err){ if(status) status.textContent = err.message; })
          .finally(function(){ btn.disabled = false; });
      };
    }
  }
  function boot(){
    extendStrings();
    registerManifest();
    getSalt();
    if (!hasCrypto()) warnUnavailableOnce();
    patchIdbApi();
    patchPrototypeNotice();
    patchOperatorSwitch();
    patchUnlock();
    hookUnlockDom();
    renderUtility();
    var tries = 0;
    var wait = setInterval(function(){
      tries++;
      patchIdbApi();
      patchOperatorSwitch();
      patchUnlock();
      if ((patchedIdb && patchedUnlock) || tries > 50) clearInterval(wait);
    }, 100);
  }
  window.rvtSec01 = {
    loadKey:loadKey,
    encrypt:encrypt,
    decrypt:decrypt,
    isKeyLoaded:isKeyLoaded,
    isEncrypted:isEncrypted,
    rotateKey:rotateKey,
    clearKey:clearKey,
    getSalt:getSalt,
    reencryptAll:reencryptAll,
    getLastTiming:function(){ return lastTiming; },
    getCheckpointStore:function(){ return CHECKPOINT_STORE; }
  };
  window.addEventListener('beforeunload', clearKey);
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot, { once:true });
  else setTimeout(boot, 0);
})();
