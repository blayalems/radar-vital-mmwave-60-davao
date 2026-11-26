import { ReferenceObservation, SessionRecord, StudyObjectiveReport } from '../../models/rvt.models';

export interface ObjectiveExportResult {
  ok?: boolean;
  objective_id?: string;
  format?: 'latex' | 'csv' | string;
  filename: string;
  content_type?: string;
  content: string;
  status?: string;
}

export interface BlindedObserverDisplay {
  observation_id: string;
  observer_id: string;
  displayValue: string;
  masked: boolean;
  numericValue: number | null;
  observed_at: string | null;
}

export interface RrObserverSummary {
  lockedCount: number;
  unblinded: boolean;
  entries: BlindedObserverDisplay[];
  discrepancyBpm: number | null;
  exceedsDiscrepancyThreshold: boolean;
  suggestedMeanBpm: number | null;
  adjudicatedFinalValue: number | null;
  adjudicatedRationale: string;
  adjudicatedAt: string | null;
}

const CONFIRMATORY_CONDITIONS = [
  'd060_none',
  'd060_cardboard',
  'd080_none',
  'd080_cardboard',
  'd100_none',
  'd100_cardboard',
  'd200_none',
  'd200_cardboard',
  'd300_none',
  'd300_cardboard'
] as const;

export function computeBreathCountRate(breathCount: number, durationS: number): number | null {
  if (!Number.isFinite(breathCount) || breathCount <= 0) return null;
  if (!Number.isFinite(durationS) || durationS <= 0) return null;
  return Math.round(((breathCount * 60) / durationS) * 100) / 100;
}

export function extractSessionConditionInfo(
  summary: SessionRecord | null,
  session: SessionRecord | null
): { conditionId: string; distanceM: number | null; barrierType: string } {
  const candidates: Array<Record<string, unknown>> = [];
  const visit = (item: unknown) => {
    if (!item || typeof item !== 'object' || Array.isArray(item)) return;
    const rec = item as Record<string, unknown>;
    candidates.push(rec);
    for (const key of ['params', 'study', 'metadata', 'session_manifest']) {
      if (rec[key] && typeof rec[key] === 'object') {
        candidates.push(rec[key] as Record<string, unknown>);
      }
    }
  };
  visit(summary);
  visit(session);

  let conditionId = '';
  let distanceM: number | null = null;
  let barrierType = '';

  for (const rec of candidates) {
    if (!conditionId && typeof rec['condition_id'] === 'string') {
      conditionId = rec['condition_id'].trim();
    }
    if (distanceM === null && rec['distance_m'] !== undefined && rec['distance_m'] !== null && rec['distance_m'] !== '') {
      const parsed = Number(rec['distance_m']);
      if (Number.isFinite(parsed)) distanceM = parsed;
    }
    if (!barrierType && typeof (rec['barrier_type'] ?? rec['barrier']) === 'string') {
      barrierType = String(rec['barrier_type'] ?? rec['barrier']).trim().toLowerCase();
    }
  }

  return { conditionId, distanceM, barrierType };
}

export function isD060NoneCondition(summary: SessionRecord | null, session: SessionRecord | null): boolean {
  const info = extractSessionConditionInfo(summary, session);
  if (info.conditionId === 'd060_none') return true;
  if (info.distanceM !== null && Math.abs(info.distanceM - 0.6) < 0.01 && info.barrierType === 'none') {
    return true;
  }
  return false;
}

export function summarizeRrObservers(
  references: ReferenceObservation[],
  adjudication: Record<string, unknown> | null | undefined
): RrObserverSummary {
  const rrEntries = references.filter(ref => ref.kind === 'rr_observer');
  const lockedEntries = rrEntries.filter(ref => ref.locked !== false);
  const unblinded = lockedEntries.length >= 2;

  const entries: BlindedObserverDisplay[] = rrEntries.map((ref, idx) => {
    const observerLabel = String(ref.observer_id || `Observer ${idx + 1}`).trim() || `Observer ${idx + 1}`;
    const rawVal = ref.value !== null && ref.value !== undefined ? Number(ref.value) : Number.NaN;
    const numericValue = Number.isFinite(rawVal) ? rawVal : null;
    const masked = !unblinded;
    return {
      observation_id: ref.observation_id,
      observer_id: observerLabel,
      displayValue: masked
        ? 'Locked (blinded)'
        : (numericValue !== null ? `${numericValue.toFixed(2)} ${ref.unit || 'br/min'}` : '--'),
      masked,
      numericValue: masked ? null : numericValue,
      observed_at: ref.observed_at ?? null
    };
  });

  const lockedValues = lockedEntries
    .map(ref => Number(ref.value))
    .filter(v => Number.isFinite(v));

  let discrepancyBpm: number | null = null;
  let suggestedMeanBpm: number | null = null;
  if (unblinded && lockedValues.length >= 2) {
    discrepancyBpm = Math.round(Math.abs(lockedValues[0] - lockedValues[1]) * 100) / 100;
    suggestedMeanBpm = Math.round(((lockedValues[0] + lockedValues[1]) / 2) * 100) / 100;
  }

  const adjFinal = adjudication?.['final_value'] !== undefined && adjudication?.['final_value'] !== null
    ? Number(adjudication['final_value'])
    : Number.NaN;

  return {
    lockedCount: lockedEntries.length,
    unblinded,
    entries,
    discrepancyBpm,
    exceedsDiscrepancyThreshold: discrepancyBpm !== null && discrepancyBpm > 1.0,
    suggestedMeanBpm,
    adjudicatedFinalValue: Number.isFinite(adjFinal) ? adjFinal : null,
    adjudicatedRationale: String(adjudication?.['rationale'] ?? ''),
    adjudicatedAt: adjudication?.['adjudicated_at'] ? String(adjudication['adjudicated_at']) : null
  };
}

export function extractObjectiveEvidenceLines(report: StudyObjectiveReport): Array<{ label: string; value: string }> {
  const payload = (report.report && typeof report.report === 'object' ? report.report : {}) as Record<string, unknown>;
  const fmt = (value: unknown, digits = 2, suffix = ''): string => {
    const n = Number(value);
    return Number.isFinite(n) ? `${n.toFixed(digits)}${suffix}` : '--';
  };
  const text = (value: unknown, fallback = '--'): string =>
    value !== undefined && value !== null && String(value).trim() !== '' ? String(value) : fallback;

  if (report.objective_id === 'objective_3_false_alarm') {
    const faCount = payload['false_alarm_count'] ?? payload['false_alarms'];
    const totalTrials = payload['trial_count'] ?? 72;
    const ciLow = payload['ci_low_pct'] ?? payload['clopper_pearson_low_pct'];
    const ciHigh = payload['ci_high_pct'] ?? payload['clopper_pearson_high_pct'];
    return [
      { label: 'False alarms', value: faCount !== undefined ? `${faCount} / ${totalTrials}` : `-- / ${totalTrials}` },
      { label: '95% Clopper-Pearson CI', value: ciLow !== undefined && ciHigh !== undefined ? `[${fmt(ciLow, 2, '%')}, ${fmt(ciHigh, 2, '%')}]` : '--' },
      { label: 'Qualified duration', value: payload['qualified_duration_s'] !== undefined ? `${payload['qualified_duration_s']} s` : '--' }
    ];
  }

  const loaLow = payload['ba_loa_low'] ?? payload['loa_low'];
  const loaHigh = payload['ba_loa_high'] ?? payload['loa_high'];
  const tostLow = payload['tost_ci90_low'] ?? payload['tost_low'];
  const tostHigh = payload['tost_ci90_high'] ?? payload['tost_high'];

  return [
    { label: 'Participants (n)', value: text(payload['n_participants']) },
    { label: 'Valid output', value: fmt(payload['valid_output_rate_pct'] ?? payload['valid_output_rate'], 1, '%') },
    { label: 'Bias / RMSE / MAE', value: `${fmt(payload['bias'])} / ${fmt(payload['rmse'])} / ${fmt(payload['mae'])}` },
    { label: 'BA 95% LoA', value: loaLow !== undefined && loaHigh !== undefined ? `[${fmt(loaLow)}, ${fmt(loaHigh)}]` : '--' },
    {
      label: report.objective_id === 'objective_1_rr' ? '90% TOST CI / Holm' : 'Outcome',
      value: tostLow !== undefined && tostHigh !== undefined
        ? `[${fmt(tostLow)}, ${fmt(tostHigh)}] · ${text(payload['holm_decision'], report.status)}`
        : text(payload['holm_decision'], report.status)
    }
  ];
}

export function buildFallbackObjectiveExport(
  report: StudyObjectiveReport,
  format: 'latex' | 'csv',
  isSandbox: boolean
): ObjectiveExportResult {
  const payload = (report.report && typeof report.report === 'object' ? report.report : {}) as Record<string, unknown>;
  const rawConditions = (payload['conditions'] && typeof payload['conditions'] === 'object'
    ? payload['conditions']
    : {}) as Record<string, Record<string, unknown>>;

  const num = (rec: Record<string, unknown> | undefined, key: string, digits = 2): string => {
    if (!rec || report.status !== 'ready') return '--';
    const val = Number(rec[key]);
    return Number.isFinite(val) ? val.toFixed(digits) : '--';
  };

  if (format === 'csv') {
    const header = 'objective_id,condition_id,status,n_participants,valid_output_pct,bias,rmse,mae,ba_loa_low,ba_loa_high,tost_ci90_low,tost_ci90_high,holm_decision';
    const lines = CONFIRMATORY_CONDITIONS.map(cid => {
      const rec = rawConditions[cid];
      const decision = report.status === 'ready' && rec?.['holm_decision'] ? String(rec['holm_decision']) : 'inconclusive';
      return [
        report.objective_id,
        cid,
        report.status,
        rec?.['n_participants'] ?? '--',
        num(rec, 'valid_output_rate_pct', 1),
        num(rec, 'bias'),
        num(rec, 'rmse'),
        num(rec, 'mae'),
        num(rec, 'ba_loa_low'),
        num(rec, 'ba_loa_high'),
        num(rec, 'tost_ci90_low'),
        num(rec, 'tost_ci90_high'),
        decision
      ].map(v => `"${String(v).replaceAll('"', '""')}"`).join(',');
    });
    const prefix = isSandbox ? '# SANDBOX DEMO DATA — NOT FOR MANUSCRIPT SUBMISSION\n' : '';
    return {
      ok: true,
      objective_id: report.objective_id,
      format: 'csv',
      filename: `${report.objective_id}_conditions.csv`,
      content_type: 'text/csv',
      content: `${prefix}${header}\n${lines.join('\n')}\n`
    };
  }

  const sandboxComment = isSandbox ? '% SANDBOX DEMO DATA — NOT FOR MANUSCRIPT SUBMISSION\n' : '';
  const rowsLatex = CONFIRMATORY_CONDITIONS.map(cid => {
    const rec = rawConditions[cid];
    const escapedCid = cid.replaceAll('_', '\\_');
    const nVal = report.status === 'ready' && rec?.['n_participants'] !== undefined ? String(rec['n_participants']) : '--';
    const validPct = num(rec, 'valid_output_rate_pct', 1);
    const bias = num(rec, 'bias');
    const rmse = num(rec, 'rmse');
    const mae = num(rec, 'mae');
    const loa = report.status === 'ready' && rec?.['ba_loa_low'] !== undefined && rec?.['ba_loa_high'] !== undefined
      ? `[${num(rec, 'ba_loa_low')}, ${num(rec, 'ba_loa_high')}]`
      : '--';
    const tost = report.status === 'ready' && rec?.['tost_ci90_low'] !== undefined && rec?.['tost_ci90_high'] !== undefined
      ? `[${num(rec, 'tost_ci90_low')}, ${num(rec, 'tost_ci90_high')}]`
      : '--';
    const decision = report.status === 'ready' && rec?.['holm_decision'] ? String(rec['holm_decision']) : 'inconclusive';
    return `${escapedCid} & ${nVal} & ${validPct} & ${bias} & ${rmse} & ${mae} & ${loa} & ${tost} & ${decision} \\\\`;
  }).join('\n');

  const tex = [
    sandboxComment + `\\begin{table}[!t]`,
    `\\centering`,
    `\\caption{Condition-Stratified Performance Summary for ${report.objective_id.replaceAll('_', '\\_')} (Status: ${report.status})}`,
    `\\label{tab:${report.objective_id}}`,
    `\\begin{tabular}{lcccccccc}`,
    `\\hline`,
    `Condition & $n$ & Valid (\\%) & Bias & RMSE & MAE & BA LoA & 90\\% TOST CI & Holm Decision \\\\`,
    `\\hline`,
    rowsLatex,
    `\\hline`,
    `\\end{tabular}`,
    `\\end{table}`
  ].join('\n');

  return {
    ok: true,
    objective_id: report.objective_id,
    format: 'latex',
    filename: `${report.objective_id}_ieee_table.tex`,
    content_type: 'application/x-tex',
    content: `${tex}\n`
  };
}
