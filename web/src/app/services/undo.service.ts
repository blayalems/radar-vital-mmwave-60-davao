import { Injectable, computed, signal } from '@angular/core';

/**
 * A single undoable action. Each entry carries a human-readable label (shown in
 * the snackbar / command palette) and an idempotent `undo` callback that
 * restores the exact prior state captured at push time.
 */
export interface UndoableAction {
  id: string;
  label: string;
  /** Epoch ms when the action was recorded. */
  ts: number;
  /** Restores the captured prior state. Must be safe to call exactly once. */
  undo: () => void;
}

const MAX_UNDO_DEPTH = 25;

/**
 * Universal undo stack for the modern dashboard.
 *
 * v11 exposed a global Ctrl+Z that reversed the most recent destructive UI
 * mutation (snapshot delete, clear-all, alert dismissal, note edits). The
 * Angular v12 dashboard regained that contract through this service: feature
 * components capture the prior state and register an `undo` closure before
 * mutating signals, then the shell triggers the most recent one on Ctrl+Z.
 *
 * The stack is intentionally bounded ({@link MAX_UNDO_DEPTH}) and is volatile —
 * undo history never persists across reloads, mirroring the v11 behaviour.
 */
@Injectable({ providedIn: 'root' })
export class UndoService {
  private readonly stack = signal<UndoableAction[]>([]);

  /** Whether there is at least one reversible action available. */
  readonly canUndo = computed(() => this.stack().length > 0);

  /** Label of the action that Ctrl+Z would reverse next, if any. */
  readonly nextLabel = computed(() => this.stack().at(-1)?.label ?? null);

  /** Current undo depth (for diagnostics / command palette). */
  readonly depth = computed(() => this.stack().length);

  /**
   * Register a reversible action. Call this *before* mutating state so the
   * closure can capture and later restore the prior value.
   */
  push(label: string, undo: () => void): void {
    const action: UndoableAction = {
      id: `undo_${Date.now()}_${Math.random().toString(36).slice(2, 8)}`,
      label,
      ts: Date.now(),
      undo
    };
    this.stack.update(items => {
      const next = [...items, action];
      // Keep the stack bounded; drop the oldest entries past the cap.
      return next.length > MAX_UNDO_DEPTH ? next.slice(next.length - MAX_UNDO_DEPTH) : next;
    });
  }

  /**
   * Reverse the most recent action. Returns its label when something was undone
   * so the caller can surface a confirmation, or `null` when the stack is empty.
   */
  undo(): string | null {
    const items = this.stack();
    if (items.length === 0) return null;
    const action = items[items.length - 1];
    this.stack.set(items.slice(0, -1));
    try {
      action.undo();
    } catch (err) {
      console.warn('[UndoService] undo callback failed', err);
    }
    return action.label;
  }

  /** Drop all pending undo actions (e.g. when switching storage scope). */
  clear(): void {
    this.stack.set([]);
  }
}
