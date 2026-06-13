import { normalizePreflightStatus } from './rvt.models';

describe('normalizePreflightStatus', () => {
  it('normalizes present statuses', () => {
    expect(normalizePreflightStatus({ status: 'GOOD' })).toBe('good');
  });

  it('treats missing or partial check payloads as empty status', () => {
    expect(normalizePreflightStatus({})).toBe('');
    expect(normalizePreflightStatus(null)).toBe('');
    expect(normalizePreflightStatus(undefined)).toBe('');
  });
});
