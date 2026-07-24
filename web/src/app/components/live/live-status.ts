export type LiveMetricBand = 'unavailable' | 'normal' | 'low' | 'high';

export interface LiveStatusInput {
  stale: boolean;
  heartRate: number | null;
  respirationRate: number | null;
  heartRateLow: number;
  heartRateHigh: number;
  respirationRateLow: number;
  respirationRateHigh: number;
}

export interface LiveStatusDescription {
  key: string;
  message: string;
}

function metricBand(value: number | null, low: number, high: number): LiveMetricBand {
  if (value === null || !Number.isFinite(value) || value <= 0) return 'unavailable';
  if (value < low) return 'low';
  if (value > high) return 'high';
  return 'normal';
}

function metricMessage(label: string, value: number | null, band: LiveMetricBand, unit: string): string {
  if (band === 'unavailable' || value === null) return `${label} is waiting for a valid reading.`;
  const relation = band === 'normal'
    ? 'within threshold'
    : `${band === 'low' ? 'below' : 'above'} threshold`;
  return `${label} is ${relation} at ${Math.round(value)} ${unit}.`;
}

export function describeLiveStatus(input: LiveStatusInput): LiveStatusDescription {
  if (input.stale) {
    return {
      key: 'stale',
      message: 'Live telemetry is stale. Do not treat displayed values as current.'
    };
  }

  const heartRateBand = metricBand(input.heartRate, input.heartRateLow, input.heartRateHigh);
  const respirationRateBand = metricBand(
    input.respirationRate,
    input.respirationRateLow,
    input.respirationRateHigh
  );

  return {
    // Raw values are deliberately absent from the key. Assistive technology
    // should hear meaningful band transitions, not the one-second data feed.
    key: `current|hr:${heartRateBand}|rr:${respirationRateBand}`,
    message: [
      'Live telemetry is current.',
      metricMessage('Heart rate', input.heartRate, heartRateBand, 'beats per minute'),
      metricMessage('Respiration', input.respirationRate, respirationRateBand, 'breaths per minute')
    ].join(' ')
  };
}
