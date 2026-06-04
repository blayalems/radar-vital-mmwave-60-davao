import { Injectable, computed, signal } from '@angular/core';

export interface UndoAction {
  label: string;
  undo: () => void;
  redo?: () => void;
}

@Injectable({
  providedIn: 'root'
})
export class UndoService {
  private readonly undoStack = signal<UndoAction[]>([]);
  private readonly redoStack = signal<UndoAction[]>([]);
  private readonly limit = 50;

  readonly canUndo = computed(() => this.undoStack().length > 0);
  readonly canRedo = computed(() => this.redoStack().length > 0);
  readonly lastActionLabel = computed(() => this.undoStack().at(-1)?.label ?? null);
  readonly nextActionLabel = computed(() => this.redoStack().at(-1)?.label ?? null);

  push(action: UndoAction): void {
    const label = action.label.trim();
    if (!label) throw new Error('Undo action label is required.');
    this.undoStack.update(items => [...items, { ...action, label }].slice(-this.limit));
    this.redoStack.set([]);
  }

  undo(): string | null {
    const action = this.undoStack().at(-1);
    if (!action) return null;
    action.undo();
    this.undoStack.update(items => items.slice(0, -1));
    if (action.redo) this.redoStack.update(items => [...items, action].slice(-this.limit));
    return action.label;
  }

  redo(): string | null {
    const action = this.redoStack().at(-1);
    if (!action?.redo) return null;
    action.redo();
    this.redoStack.update(items => items.slice(0, -1));
    this.undoStack.update(items => [...items, action].slice(-this.limit));
    return action.label;
  }

  clear(): void {
    this.undoStack.set([]);
    this.redoStack.set([]);
  }
}
