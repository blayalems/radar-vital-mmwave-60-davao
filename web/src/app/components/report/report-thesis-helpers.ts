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
  'd080_none',
  'd100_none',
  'd200_none',
  'd300_none',
  'd060_cardboard',
  'd080_cardboard',
  'd100_cardboard',
  'd200_cardboard',
  'd300_cardboard'
] as const;

function asRecord(value: unknown): Record<string, unknown> | null {
  return value && typeof value === 'object' && !Array.isArray(value)
    ? (value as Record<string, unknown>)
    : null;
}

function finiteNum(value: unknown): number | null {
  if (value === null || value === undefined || value === '') return null;
  const n = Number(value);
  return Number.isFinite(n) ? n : null;
}

function ratioOrPct(rawPct: unknown, rawRatio: unknown): number | null {
  const pct = finiteNum(rawPct);
  if (pct !== null) return pct;
  const ratio = finiteNum(rawRatio);
  return ratio !== null ? ratio * 100 : null;
}

function conditionDistanceBarrier(conditionId: string): { distanceM: string; barrierType: string } {
  const match = /^d(\d{3})_(none|cardboard)$/.exec(conditionId);
  if (!match) return { distanceM: '0.0', barrierType: 'unknown' };
  return {
    distanceM: (Number(match[1]) / 100).toFixed(1),
    barrierType: match[2]
  };
}

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
  const payload = asRecord(report.report) ?? {};
  const isReady = report.status === 'ready';
  const fmt = (value: unknown, digits = 2, suffix = ''): string => {
    const n = finiteNum(value);
    return n !== null ? `${n.toFixed(digits)}${suffix}` : '--';
  };
  const text = (value: unknown, fallback = '--'): string =>
    value !== undefined && value !== null && String(value).trim() !== '' ? String(value) : fallback;

  if (report.objective_id === 'objective_3_false_alarm') {
    const faExact = isReady ? asRecord(payload['false_alarm_exact']) : null;
    const ci95 = isReady ? asRecord(payload['confidence_interval_95']) : null;
    const exactTest = isReady ? asRecord(payload['exact_binomial_test']) : null;
    const faCount = isReady
      ? (payload['false_alarm_trial_count'] ?? payload['false_alarm_count'] ?? payload['false_alarms'] ?? faExact?.['successes'])
      : undefined;
    const totalTrials = isReady
      ? (payload['qualified_trial_count'] ?? payload['planned_trial_count'] ?? payload['trial_count'] ?? faExact?.['trials'] ?? 72)
      : (payload['planned_trial_count'] ?? payload['trial_count'] ?? 72);
    const obsRatePct = isReady
      ? ratioOrPct(payload['observed_false_alarm_rate_pct'], payload['observed_false_alarm_rate'] ?? faExact?.['proportion'])
      : null;
    const ciLowPct = isReady
      ? ratioOrPct(payload['ci_low_pct'] ?? payload['clopper_pearson_low_pct'], ci95?.['low'] ?? faExact?.['ci_low'])
      : null;
    const ciHighPct = isReady
      ? ratioOrPct(payload['ci_high_pct'] ?? payload['clopper_pearson_high_pct'], ci95?.['high'] ?? faExact?.['ci_high'])
      : null;
    const decision = isReady ? text(exactTest?.['decision'] ?? payload['decision'], report.status) : report.status;
    const faDisplay = faCount !== undefined && faCount !== null
      ? (obsRatePct !== null ? `${faCount} / ${totalTrials} (${obsRatePct.toFixed(2)}%)` : `${faCount} / ${totalTrials}`)
      : `-- / ${totalTrials}`;
    const lines: Array<{ label: string; value: string }> = [
      { label: 'False alarms', value: faDisplay },
      { label: '95% Clopper-Pearson CI', value: ciLowPct !== null && ciHighPct !== null ? `[${fmt(ciLowPct, 2, '%')}, ${fmt(ciHighPct, 2, '%')}]` : '--' },
      { label: 'Outcome', value: decision }
    ];
    if (isReady && payload['qualified_duration_s'] !== undefined) {
      lines.push({ label: 'Qualified duration', value: `${payload['qualified_duration_s']} s` });
    }
    return lines;
  }

  const stats = isReady ? (asRecord(payload['statistical_report']) ?? payload) : {};
  const denominators = asRecord(stats['denominators']);
  const coverage = asRecord(stats['coverage']);
  const balanced = asRecord(stats['participant_balanced_metrics']);
  const rawMetrics = asRecord(stats['metrics']);
  const metrics = (balanced && finiteNum(balanced['rmse']) !== null) ? balanced : rawMetrics;
  const agreement = asRecord(stats['agreement']);
  const conditionTost = asRecord(stats['condition_tost']);
  const primaryTost = asRecord(conditionTost?.['primary']);
  const tost = asRecord(stats['tost']) ?? primaryTost;

  const nParticipants = isReady
    ? (denominators?.['n_participants'] ?? primaryTost?.['n_participants'] ?? stats['n_participants'] ?? payload['n_participants'])
    : undefined;
  const validOutputPct = isReady
    ? (ratioOrPct(
        stats['valid_output_rate_pct'] ?? coverage?.['valid_output_rate_pct'] ?? payload['valid_output_rate_pct'],
        coverage?.['coverage']
      ) ?? finiteNum(stats['valid_output_rate'] ?? payload['valid_output_rate']))
    : null;
  const bias = isReady
    ? (metrics?.['bias_estimate_minus_reference']
        ?? metrics?.['bias']
        ?? agreement?.['bias']
        ?? primaryTost?.['mean_difference']
        ?? tost?.['mean_difference']
        ?? stats['bias']
        ?? payload['bias'])
    : undefined;
  const rmse = isReady ? (metrics?.['rmse'] ?? stats['rmse'] ?? payload['rmse']) : undefined;
  const mae = isReady ? (metrics?.['mae'] ?? stats['mae'] ?? payload['mae']) : undefined;
  const loaLow = isReady
    ? (agreement?.['loa_lower'] ?? agreement?.['ba_loa_low'] ?? stats['ba_loa_low'] ?? stats['loa_low'] ?? payload['ba_loa_low'] ?? payload['loa_low'])
    : undefined;
  const loaHigh = isReady
    ? (agreement?.['loa_upper'] ?? agreement?.['ba_loa_high'] ?? stats['ba_loa_high'] ?? stats['loa_high'] ?? payload['ba_loa_high'] ?? payload['loa_high'])
    : undefined;
  const tostLow = isReady
    ? (primaryTost?.['ci_low'] ?? tost?.['ci_low'] ?? stats['tost_ci90_low'] ?? stats['tost_low'] ?? payload['tost_ci90_low'] ?? payload['tost_low'])
    : undefined;
  const tostHigh = isReady
    ? (primaryTost?.['ci_high'] ?? tost?.['ci_high'] ?? stats['tost_ci90_high'] ?? stats['tost_high'] ?? payload['tost_ci90_high'] ?? payload['tost_high'])
    : undefined;
  const tostStatus = String(primaryTost?.['status'] ?? tost?.['status'] ?? '');
  const decision = !isReady
    ? report.status
    : (tostStatus.startsWith('inconclusive')
        ? 'inconclusive'
        : text(
            primaryTost?.['decision_holm']
              ?? primaryTost?.['decision']
              ?? tost?.['decision_holm']
              ?? tost?.['decision']
              ?? stats['holm_decision']
              ?? payload['holm_decision'],
            report.status
          ));

  return [
    { label: 'Participants (n)', value: text(nParticipants) },
    { label: 'Valid output', value: fmt(validOutputPct, 1, '%') },
    { label: 'Bias / RMSE / MAE', value: `${fmt(bias)} / ${fmt(rmse)} / ${fmt(mae)}` },
    { label: 'BA 95% LoA', value: loaLow !== undefined && loaHigh !== undefined ? `[${fmt(loaLow)}, ${fmt(loaHigh)}]` : '--' },
    {
      label: report.objective_id === 'objective_1_rr' ? '90% TOST CI / Holm' : 'Outcome',
      value: tostLow !== undefined && tostHigh !== undefined
        ? `[${fmt(tostLow)}, ${fmt(tostHigh)}] · ${decision}`
        : decision
    }
  ];
}

export function buildFallbackObjectiveExport(
  report: StudyObjectiveReport,
  format: 'latex' | 'csv',
  isSandbox: boolean
): ObjectiveExportResult {
  const payload = asRecord(report.report) ?? {};
  const isReady = report.status === 'ready';
  const csvCell = (v: unknown): string => `"${String(v).replaceAll('"', '""')}"`;

  if (report.objective_id === 'objective_3_false_alarm') {
    const faExact = isReady ? asRecord(payload['false_alarm_exact']) : null;
    const ci95 = isReady ? asRecord(payload['confidence_interval_95']) : null;
    const exactTest = isReady ? asRecord(payload['exact_binomial_test']) : null;
    const planned = isReady && payload['planned_trial_count'] !== undefined ? String(payload['planned_trial_count']) : '--';
    const qualified = isReady && payload['qualified_trial_count'] !== undefined
      ? String(payload['qualified_trial_count'])
      : (isReady && faExact?.['trials'] !== undefined ? String(faExact['trials']) : '--');
    const alarms = isReady && (payload['false_alarm_trial_count'] ?? payload['false_alarm_count'] ?? faExact?.['successes']) !== undefined
      ? String(payload['false_alarm_trial_count'] ?? payload['false_alarm_count'] ?? faExact?.['successes'])
      : '--';
    const ratePct = isReady
      ? ratioOrPct(payload['observed_false_alarm_rate_pct'], payload['observed_false_alarm_rate'] ?? faExact?.['proportion'])
      : null;
    const ciLowPct = isReady
      ? ratioOrPct(payload['ci_low_pct'] ?? payload['clopper_pearson_low_pct'], ci95?.['low'] ?? faExact?.['ci_low'])
      : null;
    const ciHighPct = isReady
      ? ratioOrPct(payload['ci_high_pct'] ?? payload['clopper_pearson_high_pct'], ci95?.['high'] ?? faExact?.['ci_high'])
      : null;
    const pVal = isReady ? finiteNum(exactTest?.['p_value'] ?? faExact?.['p_value']) : null;
    const decision = isReady ? String(exactTest?.['decision'] ?? payload['decision'] ?? 'inconclusive') : 'inconclusive';

    if (format === 'csv') {
      const header = 'objective_id,status,planned_trial_count,qualified_trial_count,false_alarm_trial_count,observed_false_alarm_rate_pct,ci95_low_pct,ci95_high_pct,p_value,decision';
      const row = [
        report.objective_id,
        report.status,
        planned,
        qualified,
        alarms,
        ratePct !== null ? ratePct.toFixed(2) : '--',
        ciLowPct !== null ? ciLowPct.toFixed(2) : '--',
        ciHighPct !== null ? ciHighPct.toFixed(2) : '--',
        pVal !== null ? pVal.toFixed(4) : '--',
        decision
      ].map(csvCell).join(',');
      const prefix = isSandbox ? '# SANDBOX DEMO DATA — NOT FOR MANUSCRIPT SUBMISSION\n' : '';
      return {
        ok: true,
        objective_id: report.objective_id,
        format: 'csv',
        filename: `${report.objective_id}_summary.csv`,
        content_type: 'text/csv',
        content: `${prefix}${header}\n${row}\n`
      };
    }

    const sandboxComment = isSandbox ? '% SANDBOX DEMO DATA — NOT FOR MANUSCRIPT SUBMISSION\n' : '';
    const rateStr = ratePct !== null ? ratePct.toFixed(2) : '--';
    const ciStr = ciLowPct !== null && ciHighPct !== null
      ? `[${ciLowPct.toFixed(2)}, ${ciHighPct.toFixed(2)}]\\%`
      : '--';
    const tex = [
      sandboxComment + `\\begin{table}[!t]`,
      `\\centering`,
      `\\caption{${report.objective_id.replaceAll('_', '\\_')} (${report.status})}`,
      `\\label{tab:${report.objective_id}}`,
      `\\begin{tabular}{lcccc}`,
      `\\hline`,
      `Outcome & Qualified Trials & False Alarms & Rate (\\%) & 95\\% Clopper--Pearson CI \\\\`,
      `\\hline`,
      `No-subject false alarm & ${qualified} & ${alarms} & ${rateStr} & ${ciStr} \\\\`,
      `Decision & \\multicolumn{4}{l}{${decision.replaceAll('_', '\\_')}} \\\\`,
      `\\hline`,
      `\\end{tabular}`,
      `\\end{table}`
    ].join('\n');
    return {
      ok: true,
      objective_id: report.objective_id,
      format: 'latex',
      filename: `${report.objective_id}_summary.tex`,
      content_type: 'application/x-tex',
      content: `${tex}\n`
    };
  }

  const stats = isReady ? (asRecord(payload['statistical_report']) ?? payload) : {};
  const conditionTost = isReady ? asRecord(stats['condition_tost']) : null;
  const tostByCondition = new Map<string, Record<string, unknown>>();
  if (conditionTost) {
    const primary = asRecord(conditionTost['primary']);
    if (primary?.['condition_id']) {
      tostByCondition.set(String(primary['condition_id']), primary);
    }
    if (Array.isArray(conditionTost['secondary'])) {
      for (const item of conditionTost['secondary']) {
        const rec = asRecord(item);
        if (rec?.['condition_id']) {
          tostByCondition.set(String(rec['condition_id']), rec);
        }
      }
    }
  }
  const rawConditions: Record<string, Record<string, unknown>> = {};
  if (isReady) {
    for (const key of ['condition_metrics', 'by_condition', 'conditions']) {
      const candidate = asRecord(stats[key]);
      if (candidate) {
        for (const [cid, val] of Object.entries(candidate)) {
          const rec = asRecord(val);
          if (rec) rawConditions[cid] = rec;
        }
      }
    }
  }

  const formatNum = (val: unknown, digits = 2): string => {
    const n = finiteNum(val);
    return n !== null ? n.toFixed(digits) : '--';
  };

  const rows = CONFIRMATORY_CONDITIONS.map(cid => {
    const { distanceM, barrierType } = conditionDistanceBarrier(cid);
    const role = cid === 'd100_none' ? 'primary' : 'secondary';
    const extra = isReady ? rawConditions[cid] : undefined;
    const tost = isReady ? tostByCondition.get(cid) : undefined;
    const nParticipants = isReady ? (tost?.['n_participants'] ?? extra?.['n_participants']) : undefined;
    const validPct = isReady ? ratioOrPct(extra?.['valid_output_rate_pct'], extra?.['coverage']) : null;
    const biasVal = isReady
      ? (extra?.['bias'] ?? extra?.['bias_estimate_minus_reference'] ?? tost?.['mean_difference'])
      : undefined;
    const rmseVal = isReady ? extra?.['rmse'] : undefined;
    const maeVal = isReady ? extra?.['mae'] : undefined;
    const loaLow = isReady ? (extra?.['ba_loa_low'] ?? extra?.['loa_lower'] ?? extra?.['loa_low']) : undefined;
    const loaHigh = isReady ? (extra?.['ba_loa_high'] ?? extra?.['loa_upper'] ?? extra?.['loa_high']) : undefined;
    const tostLow = isReady ? (tost?.['ci_low'] ?? extra?.['tost_ci90_low'] ?? extra?.['tost_ci_low']) : undefined;
    const tostHigh = isReady ? (tost?.['ci_high'] ?? extra?.['tost_ci90_high'] ?? extra?.['tost_ci_high']) : undefined;
    const pVal = isReady ? tost?.['p_value'] : undefined;
    const pHolm = isReady ? tost?.['p_value_holm'] : undefined;
    const rawTostStatus = String(tost?.['status'] ?? '');
    const decision = isReady
      ? (rawTostStatus.startsWith('inconclusive')
          ? 'inconclusive'
          : String(tost?.['decision_holm'] ?? tost?.['decision'] ?? extra?.['holm_decision'] ?? 'inconclusive'))
      : 'inconclusive';
    return {
      cid,
      distanceM,
      barrierType,
      role,
      nParticipants: nParticipants !== undefined && nParticipants !== null ? String(nParticipants) : '--',
      validPct: formatNum(validPct, 1),
      bias: formatNum(biasVal),
      rmse: formatNum(rmseVal),
      mae: formatNum(maeVal),
      loaLow: formatNum(loaLow),
      loaHigh: formatNum(loaHigh),
      tostLow: formatNum(tostLow),
      tostHigh: formatNum(tostHigh),
      pVal: formatNum(pVal, 4),
      pHolm: formatNum(pHolm, 4),
      decision
    };
  });

  if (format === 'csv') {
    const header = 'objective_id,status,condition_id,distance_m,barrier_type,role,n_participants,valid_output_rate_pct,bias,rmse,mae,ba_loa_low,ba_loa_high,tost_ci_low,tost_ci_high,p_value,p_value_holm,holm_decision';
    const lines = rows.map(row => [
      report.objective_id,
      report.status,
      row.cid,
      row.distanceM,
      row.barrierType,
      row.role,
      row.nParticipants,
      row.validPct,
      row.bias,
      row.rmse,
      row.mae,
      row.loaLow,
      row.loaHigh,
      row.tostLow,
      row.tostHigh,
      row.pVal,
      row.pHolm,
      row.decision
    ].map(csvCell).join(','));
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
  const rowsLatex = rows.map(row => {
    const escapedCid = row.cid.replaceAll('_', '\\_');
    const loa = row.loaLow !== '--' && row.loaHigh !== '--' ? `[${row.loaLow}, ${row.loaHigh}]` : '--';
    const tost = row.tostLow !== '--' && row.tostHigh !== '--' ? `[${row.tostLow}, ${row.tostHigh}]` : '--';
    return `${escapedCid} & ${row.nParticipants} & ${row.validPct} & ${row.bias} & ${row.rmse} & ${row.mae} & ${loa} & ${tost} & ${row.decision} \\\\`;
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
