import { TestBed } from '@angular/core/testing';
import { UndoService } from './undo.service';

describe('UndoService', () => {
  let service: UndoService;

  beforeEach(() => {
    TestBed.configureTestingModule({ providers: [UndoService] });
    service = TestBed.inject(UndoService);
  });

  it('starts empty', () => {
    expect(service.canUndo()).toBe(false);
    expect(service.nextLabel()).toBeNull();
    expect(service.undo()).toBeNull();
  });

  it('records and reverses an action', () => {
    let value = 'original';
    service.push('Change value', () => { value = 'original'; });
    value = 'mutated';
    expect(service.canUndo()).toBe(true);
    expect(service.nextLabel()).toBe('Change value');

    const label = service.undo();
    expect(label).toBe('Change value');
    expect(value).toBe('original');
    expect(service.canUndo()).toBe(false);
  });

  it('reverses actions in last-in-first-out order', () => {
    const order: string[] = [];
    service.push('first', () => order.push('first'));
    service.push('second', () => order.push('second'));

    service.undo();
    service.undo();
    expect(order).toEqual(['second', 'first']);
  });

  it('bounds the stack depth and drops oldest entries', () => {
    for (let i = 0; i < 30; i++) {
      service.push(`action ${i}`, () => undefined);
    }
    // Capped at 25 entries; the most recent should be retained.
    expect(service.depth()).toBe(25);
    expect(service.nextLabel()).toBe('action 29');
  });

  it('clears the stack', () => {
    service.push('action', () => undefined);
    service.clear();
    expect(service.canUndo()).toBe(false);
  });

  it('does not throw if an undo callback fails', () => {
    service.push('bad', () => { throw new Error('boom'); });
    expect(() => service.undo()).not.toThrow();
    expect(service.canUndo()).toBe(false);
  });
});
