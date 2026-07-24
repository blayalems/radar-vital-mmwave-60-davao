import { describe, expect, it } from 'vitest';
import { countValidTrendSamples, describeTrendSamples } from './trend-canvas.component';

describe('describeTrendSamples', () => {
  it('states honestly when no samples are available', () => {
    expect(describeTrendSamples(0)).toEqual({
      state: 'empty',
      title: 'No trend samples yet',
      detail: 'Waiting for the first valid published reading.'
    });
  });

  it('reports bounded warm-up progress before the trend is credible', () => {
    expect(describeTrendSamples(4)).toEqual({
      state: 'warming',
      title: 'Trend warming up',
      detail: '4 of 10 valid samples collected.'
    });
  });

  it('reports ready only at the configured minimum', () => {
    expect(describeTrendSamples(9).state).toBe('warming');
    expect(describeTrendSamples(10)).toEqual({
      state: 'ready',
      title: 'Trend ready',
      detail: '10 samples available.'
    });
  });

  it('normalizes invalid counts and unsafe minimums', () => {
    expect(describeTrendSamples(Number.NaN, 1).state).toBe('empty');
    expect(describeTrendSamples(1, 1).detail).toBe('1 of 2 valid samples collected.');
  });

  it('excludes standby zeroes and non-finite values from readiness', () => {
    expect(countValidTrendSamples([0, Number.NaN, -1, 72, 73])).toBe(2);
  });
});
