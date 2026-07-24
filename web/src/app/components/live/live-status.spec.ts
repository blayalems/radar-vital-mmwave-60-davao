import { describe, expect, it } from 'vitest';
import { describeLiveStatus, LiveStatusInput } from './live-status';

const healthy: LiveStatusInput = {
  stale: false,
  heartRate: 72,
  respirationRate: 14,
  heartRateLow: 40,
  heartRateHigh: 140,
  respirationRateLow: 6,
  respirationRateHigh: 30
};

describe('describeLiveStatus', () => {
  it('keeps one aggregate key while raw in-range telemetry changes', () => {
    const first = describeLiveStatus(healthy);
    const next = describeLiveStatus({ ...healthy, heartRate: 74, respirationRate: 15 });

    expect(first.key).toBe('current|hr:normal|rr:normal');
    expect(next.key).toBe(first.key);
    expect(next.message).toContain('Heart rate is within threshold at 74 beats per minute.');
  });

  it('changes the aggregate key when a metric crosses a threshold', () => {
    const high = describeLiveStatus({ ...healthy, heartRate: 151 });

    expect(high.key).toBe('current|hr:high|rr:normal');
    expect(high.message).toContain('Heart rate is above threshold at 151 beats per minute.');
  });

  it('treats standby zeroes as unavailable instead of low alerts', () => {
    const standby = describeLiveStatus({ ...healthy, heartRate: 0, respirationRate: 0 });

    expect(standby.key).toBe('current|hr:unavailable|rr:unavailable');
    expect(standby.message).toContain('Heart rate is waiting for a valid reading.');
    expect(standby.message).not.toContain('below threshold');
  });

  it('uses one stable stale state regardless of retained values', () => {
    const first = describeLiveStatus({ ...healthy, stale: true });
    const next = describeLiveStatus({
      ...healthy,
      stale: true,
      heartRate: 180,
      respirationRate: null
    });

    expect(first).toEqual(next);
    expect(first.key).toBe('stale');
  });
});
