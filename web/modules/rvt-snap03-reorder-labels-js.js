(function() {
  var ORDER_KEY = 'rvt-snap-order';
  var NOTES_KEY = 'rvt-snap-notes';
  var MAX_LABEL = 60;
  var draggedItem = null;
  var dropZone = null;
  var editingItem = null;

  function loadOrder() {
    try { return JSON.parse(localStorage.getItem(ORDER_KEY) || '[]') || []; } catch(e) { return []; }
  }

  function saveOrder(order) {
    try { localStorage.setItem(ORDER_KEY, JSON.stringify(order)); } catch(e) {}
  }

  function loadNotes() {
    try { return JSON.parse(localStorage.getItem(NOTES_KEY) || '{}') || {}; } catch(e) { return {}; }
  }

  function saveNotes(notes) {
    try { localStorage.setItem(NOTES_KEY, JSON.stringify(notes)); } catch(e) {}
  }

  function getSnapId(el) {
    return el.getAttribute('data-snap-id') || el.getAttribute('id') || '';
  }

  function startEdit(labelEl) {
    if (editingItem) cancelEdit();

    var text = labelEl.textContent.trim();
    var input = document.createElement('input');
    input.className = 'rvt-snap-label-edit';
    input.type = 'text';
    input.value = text;
    input.maxLength = MAX_LABEL;
    input.setAttribute('aria-label', 'Edit snapshot label');

    var finished = function() {
      var newText = input.value.trim().slice(0, MAX_LABEL) || text;
      labelEl.textContent = newText;
      input.parentElement.replaceChild(labelEl, input);

      var snapId = getSnapId(labelEl.closest('[data-snap-id], [data-snap-index]'));
      if (snapId) {
        var notes = loadNotes();
        notes[snapId] = newText;
        saveNotes(notes);
        announceChange('Label updated to: ' + newText);
      }
      editingItem = null;
    };

    input.onblur = finished;
    input.onkeydown = function(e) {
      if (e.key === 'Enter') { e.preventDefault(); finished(); }
      else if (e.key === 'Escape') { cancelEdit(); }
    };

    labelEl.parentElement.replaceChild(input, labelEl);
    editingItem = input;
    input.focus();
    input.select();
  }

  function cancelEdit() {
    if (editingItem) {
      var orig = document.createElement('p');
      orig.className = 'rvt-snap-card-label';
      orig.textContent = editingItem.value;
      editingItem.parentElement.replaceChild(orig, editingItem);
      editingItem = null;
    }
  }

  function announceChange(msg) {
    var hint = document.createElement('div');
    hint.className = 'rvt-snap-reorder-hint';
    hint.setAttribute('aria-live', 'polite');
    hint.setAttribute('aria-atomic', 'true');
    hint.textContent = msg;
    document.body.appendChild(hint);
    setTimeout(function() { hint.remove(); }, 2000);
  }

  function setupDragDrop() {
    var snapsList = document.querySelector('[data-view="snapshots"] .snaps-list, .snaps-grid-container, [data-snap-list]');
    if (!snapsList) return;

    var onDragStart = function(e) {
      draggedItem = this.closest('[data-snap-index]');
      if (!draggedItem) return;
      draggedItem.classList.add('rvt-dragging');
      e.dataTransfer.effectAllowed = 'move';
      e.dataTransfer.setData('text/plain', getSnapId(draggedItem));
    };

    var onDragOver = function(e) {
      e.preventDefault();
      e.dataTransfer.dropEffect = 'move';
      var target = e.target.closest('[data-snap-index]');
      if (target && target !== draggedItem) {
        target.classList.add('rvt-drag-over');
      }
    };

    var onDragLeave = function(e) {
      var target = e.target.closest('[data-snap-index]');
      if (target) target.classList.remove('rvt-drag-over');
    };

    var onDrop = function(e) {
      e.preventDefault();
      if (!draggedItem) return;

      var dropTarget = e.target.closest('[data-snap-index]');
      if (!dropTarget || dropTarget === draggedItem) {
        draggedItem.classList.remove('rvt-dragging');
        document.querySelectorAll('.rvt-drag-over').forEach(function(el) { el.classList.remove('rvt-drag-over'); });
        return;
      }

      /* Reorder in DOM */
      var parent = snapsList;
      var items = Array.from(parent.querySelectorAll('[data-snap-index]'));
      var fromIdx = items.indexOf(draggedItem);
      var toIdx = items.indexOf(dropTarget);

      if (fromIdx >= 0 && toIdx >= 0 && fromIdx !== toIdx) {
        if (fromIdx < toIdx) {
          dropTarget.parentElement.insertBefore(draggedItem, dropTarget.nextElementSibling);
        } else {
          dropTarget.parentElement.insertBefore(draggedItem, dropTarget);
        }

        /* Save reordered IDs */
        items = Array.from(parent.querySelectorAll('[data-snap-index]'));
        var order = items.map(function(el) { return getSnapId(el); });
        saveOrder(order);
        announceChange('Snapshot moved. New position: ' + (items.indexOf(draggedItem) + 1) + ' of ' + items.length);
      }

      draggedItem.classList.remove('rvt-dragging');
      document.querySelectorAll('.rvt-drag-over').forEach(function(el) { el.classList.remove('rvt-drag-over'); });
      draggedItem = null;
    };

    var onDragEnd = function(e) {
      if (draggedItem) draggedItem.classList.remove('rvt-dragging');
      document.querySelectorAll('.rvt-drag-over').forEach(function(el) { el.classList.remove('rvt-drag-over'); });
      draggedItem = null;
    };

    snapsList.addEventListener('dragstart', onDragStart);
    snapsList.addEventListener('dragover', onDragOver);
    snapsList.addEventListener('dragleave', onDragLeave);
    snapsList.addEventListener('drop', onDrop);
    snapsList.addEventListener('dragend', onDragEnd);
  }

  function setupLabelEditing() {
    var snapsList = document.querySelector('[data-view="snapshots"] .snaps-list, .snaps-grid-container, [data-snap-list]');
    if (!snapsList) return;

    snapsList.addEventListener('dblclick', function(e) {
      var labelEl = e.target.closest('.rvt-snap-card-label');
      if (labelEl && !editingItem) startEdit(labelEl);
    });

    snapsList.addEventListener('keydown', function(e) {
      if (e.key === 'F2' || e.key === 'f2') {
        e.preventDefault();
        var card = e.target.closest('[data-snap-index]');
        if (card) {
          var labelEl = card.querySelector('.rvt-snap-card-label');
          if (labelEl && !editingItem) startEdit(labelEl);
        }
      }
    });
  }

  function setupKeyboardReorder() {
    var snapsList = document.querySelector('[data-view="snapshots"] .snaps-list, .snaps-grid-container, [data-snap-list]');
    if (!snapsList) return;

    snapsList.addEventListener('keydown', function(e) {
      if (!e.ctrlKey && !e.metaKey) return;
      if (e.key !== 'ArrowUp' && e.key !== 'ArrowDown') return;

      e.preventDefault();
      var card = e.target.closest('[data-snap-index]');
      if (!card) return;

      var items = Array.from(snapsList.querySelectorAll('[data-snap-index]'));
      var idx = items.indexOf(card);
      if (idx < 0) return;

      var swapIdx = e.key === 'ArrowUp' ? idx - 1 : idx + 1;
      if (swapIdx < 0 || swapIdx >= items.length) return;

      var swapItem = items[swapIdx];
      if (e.key === 'ArrowUp') {
        card.parentElement.insertBefore(card, swapItem);
      } else {
        swapItem.parentElement.insertBefore(swapItem, card);
      }

      items = Array.from(snapsList.querySelectorAll('[data-snap-index]'));
      var order = items.map(function(el) { return getSnapId(el); });
      saveOrder(order);
      announceChange('Snapshot moved. New position: ' + (items.indexOf(card) + 1) + ' of ' + items.length);
      card.focus();
    });
  }

  function boot() {
    setupDragDrop();
    setupLabelEditing();
    setupKeyboardReorder();

    window.rvtSnapReorder = {
      getOrder: loadOrder,
      saveOrder: saveOrder,
      getNotes: loadNotes,
      saveNotes: saveNotes
    };

    window.rvtSnapReorderReady = true;
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', function(){ setTimeout(boot, 1000); }, {once:true});
  } else {
    setTimeout(boot, 1000);
  }
})();
