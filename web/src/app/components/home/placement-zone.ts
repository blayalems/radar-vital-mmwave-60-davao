export interface PlacementZone {
  id: 'searching' | 'close' | 'optimal' | 'good' | 'acceptable' | 'far';
  label: string;
  hint: string;
  tone: 'ok' | 'warn' | 'muted';
}

/**
 * Placement zones mirror the firmware's distance-confidence bands
 * (optimal 40-100 cm, good <=140 cm, acceptable <=180 cm) and chest-height
 * <=1.5 m guidance for the 60 GHz vital-signs radar.
 */
export function describePlacementZone(distanceCm: unknown): PlacementZone {
  const distance = Number(distanceCm);
  if (!Number.isFinite(distance) || distance <= 0) {
    return {
      id: 'searching',
      label: 'No target',
      hint: 'Seat the subject facing the radar at chest height.',
      tone: 'muted'
    };
  }
  if (distance < 40) {
    return {
      id: 'close',
      label: 'Too close',
      hint: 'Move back to 40–100 cm for the cleanest signal.',
      tone: 'warn'
    };
  }
  if (distance <= 100) {
    return {
      id: 'optimal',
      label: 'Sweet spot',
      hint: 'Hold this position for the session.',
      tone: 'ok'
    };
  }
  if (distance <= 140) {
    return {
      id: 'good',
      label: 'Good',
      hint: 'Slightly closer (40–100 cm) may improve heart-rate quality.',
      tone: 'ok'
    };
  }
  if (distance <= 180) {
    return {
      id: 'acceptable',
      label: 'Acceptable',
      hint: 'Best results within 1.5 m of the radar.',
      tone: 'ok'
    };
  }
  return {
    id: 'far',
    label: 'Out of range',
    hint: 'Bring the subject within 1.5 m of the radar.',
    tone: 'warn'
  };
}
