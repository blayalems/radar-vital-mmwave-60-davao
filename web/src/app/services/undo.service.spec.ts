import { TestBed } from '@angular/core/testing';
import { UndoService } from './undo.service';

describe('UndoService', () => {
  let service: UndoService;

  beforeEach(() => {
    TestBed.configureTestingModule({
      providers: [UndoService]
    });
    service = TestBed.inject(UndoService);
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });

  it('should initialize with empty undo/redo stacks', () => {
    expect(service.canUndo()).toBe(false);
    expect(service.canRedo()).toBe(false);
    expect(service.lastActionLabel()).toBe('');
    expect(service.nextActionLabel()).toBe('');
  });

  it('should push actions and allow undoing them', () => {
    let actionDone = false;
    let actionUndone = false;

    service.push({
      label: 'Test Action',
      undo: () => { actionUndone = true; },
      redo: () => { actionDone = true; }
    });

    expect(service.canUndo()).toBe(true);
    expect(service.canRedo()).toBe(false);
    expect(service.lastActionLabel()).toBe('Test Action');

    service.undo();
    expect(actionUndone).toBe(true);
    expect(service.canUndo()).toBe(false);
    expect(service.canRedo()).toBe(true);
    expect(service.nextActionLabel()).toBe('Test Action');
  });

  it('should allow redoing an action after undoing it', () => {
    let undoCount = 0;
    let redoCount = 0;

    service.push({
      label: 'Reversible Action',
      undo: () => { undoCount++; },
      redo: () => { redoCount++; }
    });

    service.undo();
    expect(undoCount).toBe(1);
    expect(redoCount).toBe(0);

    service.redo();
    expect(undoCount).toBe(1);
    expect(redoCount).toBe(1);
    expect(service.canUndo()).toBe(true);
    expect(service.canRedo()).toBe(false);
  });

  it('should clear forward history when a new action is pushed', () => {
    service.push({ label: 'A', undo: () => {}, redo: () => {} });
    service.push({ label: 'B', undo: () => {}, redo: () => {} });
    
    service.undo(); // back to A
    expect(service.canRedo()).toBe(true);
    expect(service.nextActionLabel()).toBe('B');

    service.push({ label: 'C', undo: () => {}, redo: () => {} });
    expect(service.canRedo()).toBe(false);
    expect(service.lastActionLabel()).toBe('C');
  });

  it('should enforce the maximum limit of 50 actions', () => {
    let undoCount = 0;
    for (let i = 0; i < 55; i++) {
      service.push({
        label: `Action ${i}`,
        undo: () => { undoCount++; }
      });
    }

    // Stack should have max 50 items. Oldest 5 (0-4) should be pruned.
    // So we can undo exactly 50 times.
    expect(service.canUndo()).toBe(true);
    expect(service.lastActionLabel()).toBe('Action 54');

    let steps = 0;
    while (service.canUndo()) {
      service.undo();
      steps++;
    }
    expect(steps).toBe(50);
    expect(undoCount).toBe(50);
  });

  it('should clear everything on clear()', () => {
    service.push({ label: 'Action', undo: () => {} });
    service.clear();
    expect(service.canUndo()).toBe(false);
    expect(service.canRedo()).toBe(false);
    expect(service.lastActionLabel()).toBe('');
  });
});
