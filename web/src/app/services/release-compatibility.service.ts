import { Injectable, inject, signal } from '@angular/core';
import { ControlStatus } from '../models/rvt.models';
import { ApiService } from './api.service';
import { PRODUCT_VERSION } from './app-meta';
import {
  clientReleaseHandshake,
  CONTROL_API_SCHEMA,
  SERIAL_PROTOCOL,
  SERIAL_WIDTH_EXPECTED,
  STUDY_SESSION_SCHEMA
} from './release-contract';
import type { ClientReleaseHandshake } from './release-contract';
export {
  clientReleaseHandshake,
  CONTROL_API_SCHEMA,
  SERIAL_PROTOCOL,
  SERIAL_WIDTH_EXPECTED,
  STUDY_SESSION_SCHEMA
} from './release-contract';
export type { ClientReleaseHandshake } from './release-contract';

export type ReleaseCompatibilityState = 'checking' | 'compatible' | 'incompatible' | 'unverified';

export interface ReleaseVersionPayload {
  product_version?: unknown;
  trainer?: unknown;
  trainer_version?: unknown;
  dashboard?: unknown;
  dashboard_version?: unknown;
  firmware_expected?: unknown;
  firmware_observed?: unknown;
  serial_protocol?: unknown;
  serial_width_expected?: unknown;
  serial_width_observed?: unknown;
  schema_versions?: Record<string, unknown>;
  [key: string]: unknown;
}

export interface ReleaseCompatibilityDetail {
  id: string;
  label: string;
  expected: string;
  observed: string;
  state: 'match' | 'mismatch' | 'unknown';
}

export interface ReleaseCompatibilitySummary {
  state: ReleaseCompatibilityState;
  blocksStart: boolean;
  headline: string;
  message: string;
  guidance: string[];
  details: ReleaseCompatibilityDetail[];
  checkedAtMs: number | null;
}

const CHECKING: ReleaseCompatibilitySummary = {
  state: 'checking',
  blocksStart: true,
  headline: 'Checking release compatibility',
  message: 'Comparing this dashboard with the trainer, firmware, serial protocol, and API schemas.',
  guidance: [],
  details: [],
  checkedAtMs: null
};

function text(value: unknown): string {
  return typeof value === 'string' ? value.trim() : '';
}

function nestedRecord(value: unknown): Record<string, unknown> {
  return value && typeof value === 'object' ? value as Record<string, unknown> : {};
}

function firstText(...values: unknown[]): string {
  for (const value of values) {
    const normalized = text(value);
    if (normalized) return normalized;
  }
  return '';
}

function firstFinite(...values: unknown[]): number | null {
  for (const value of values) {
    if (value === '' || value === null || value === undefined) continue;
    const normalized = Number(value);
    if (Number.isFinite(normalized)) return normalized;
  }
  return null;
}

function canonicalVersion(value: unknown): string {
  return text(value).replace(/^v(?=\d)/i, '');
}

function versionDetail(id: string, label: string, expected: string, observed: string): ReleaseCompatibilityDetail {
  if (!observed) return { id, label, expected, observed: 'Not reported', state: 'unknown' };
  return {
    id,
    label,
    expected,
    observed,
    state: canonicalVersion(observed) === canonicalVersion(expected) ? 'match' : 'mismatch'
  };
}

function exactDetail(
  id: string,
  label: string,
  expected: string | number,
  observed: string | number | null
): ReleaseCompatibilityDetail {
  if (observed === null || observed === '') {
    return { id, label, expected: String(expected), observed: 'Not reported', state: 'unknown' };
  }
  return {
    id,
    label,
    expected: String(expected),
    observed: String(observed),
    state: String(observed) === String(expected) ? 'match' : 'mismatch'
  };
}

export function evaluateReleaseCompatibility(
  status: ControlStatus | null | undefined,
  version: ReleaseVersionPayload | null | undefined
): ReleaseCompatibilitySummary {
  const statusRecord = nestedRecord(status);
  const versionRecord = nestedRecord(version);
  const active = nestedRecord(statusRecord['active_session']);
  const versionSchemas = nestedRecord(versionRecord['schema_versions']);
  const statusSchemas = nestedRecord(statusRecord['schema_versions']);

  const product = firstText(versionRecord['product_version'], statusRecord['product_version']);
  const trainer = firstText(
    versionRecord['trainer'],
    versionRecord['trainer_version'],
    statusRecord['trainer_version']
  );
  const dashboard = firstText(
    versionRecord['dashboard'],
    versionRecord['dashboard_version'],
    statusRecord['dashboard_version']
  );
  const firmwareExpected = firstText(versionRecord['firmware_expected'], statusRecord['firmware_expected']);
  const firmwareObserved = firstText(
    versionRecord['firmware_observed'],
    statusRecord['firmware_observed'],
    active['firmware_observed'],
    active['firmware_version']
  );
  const serialProtocol = firstText(
    versionRecord['serial_protocol'],
    statusRecord['serial_protocol'],
    active['serial_protocol']
  );
  const serialWidthExpected = firstFinite(
    versionRecord['serial_width_expected'],
    statusRecord['serial_width_expected'],
    active['serial_width_expected']
  );
  const serialWidthObserved = firstFinite(
    versionRecord['serial_width_observed'],
    statusRecord['serial_width_observed'],
    active['serial_width_observed']
  );
  const controlSchema = firstText(versionSchemas['control_api'], statusSchemas['control_api']);
  const studySchema = firstText(versionSchemas['study_session'], statusSchemas['study_session']);

  const details: ReleaseCompatibilityDetail[] = [
    versionDetail('product', 'Product release', PRODUCT_VERSION, product),
    versionDetail('trainer', 'Trainer release', PRODUCT_VERSION, trainer),
    versionDetail('dashboard', 'Dashboard release', PRODUCT_VERSION, dashboard),
    versionDetail('firmware_expected', 'Expected firmware', `v${PRODUCT_VERSION}`, firmwareExpected),
    exactDetail('serial_protocol', 'Serial protocol', SERIAL_PROTOCOL, serialProtocol),
    exactDetail('serial_width_expected', 'Serial width', SERIAL_WIDTH_EXPECTED, serialWidthExpected),
    exactDetail('control_api', 'Control API schema', CONTROL_API_SCHEMA, controlSchema),
    exactDetail('study_session', 'Study session schema', STUDY_SESSION_SCHEMA, studySchema)
  ];

  // The public version endpoint and authenticated status endpoint are
  // independent release witnesses. Never let a matching value from one hide a
  // known mismatch from the other.
  const statusWitnesses = [
    versionDetail('product', 'Status product release', PRODUCT_VERSION, firstText(statusRecord['product_version'])),
    versionDetail('trainer', 'Status trainer release', PRODUCT_VERSION, firstText(statusRecord['trainer_version'])),
    versionDetail('dashboard', 'Status dashboard release', PRODUCT_VERSION, firstText(statusRecord['dashboard_version'])),
    versionDetail('firmware_expected', 'Status expected firmware', `v${PRODUCT_VERSION}`, firstText(statusRecord['firmware_expected'])),
    exactDetail('serial_protocol', 'Status serial protocol', SERIAL_PROTOCOL, firstText(statusRecord['serial_protocol'])),
    exactDetail(
      'serial_width_expected',
      'Status serial width',
      SERIAL_WIDTH_EXPECTED,
      firstFinite(statusRecord['serial_width_expected'])
    ),
    exactDetail('control_api', 'Status control API schema', CONTROL_API_SCHEMA, firstText(statusSchemas['control_api'])),
    exactDetail('study_session', 'Status study session schema', STUDY_SESSION_SCHEMA, firstText(statusSchemas['study_session']))
  ];
  details.push(...statusWitnesses.filter(detail => detail.state === 'mismatch'));

  if (firmwareObserved) {
    details.push(versionDetail(
      'firmware_observed',
      'Observed firmware',
      firmwareExpected || `v${PRODUCT_VERSION}`,
      firmwareObserved
    ));
  }
  if (serialWidthObserved !== null) {
    details.push(exactDetail('serial_width_observed', 'Observed serial width', SERIAL_WIDTH_EXPECTED, serialWidthObserved));
  }

  const mismatches = details.filter(detail => detail.state === 'mismatch');
  const unknowns = details.filter(detail => detail.state === 'unknown');
  if (mismatches.length) {
    const mismatchIds = new Set(mismatches.map(detail => detail.id));
    const guidance = new Set<string>();
    if (['product', 'trainer', 'control_api', 'study_session'].some(id => mismatchIds.has(id))) {
      guidance.add('Restart the trainer with the matching release.');
    }
    if (['product', 'dashboard', 'control_api', 'study_session'].some(id => mismatchIds.has(id))) {
      guidance.add('Reload the dashboard and clear any stale installed-app cache.');
    }
    if (['firmware_expected', 'firmware_observed', 'serial_protocol', 'serial_width_expected', 'serial_width_observed']
      .some(id => mismatchIds.has(id))) {
      guidance.add('Flash the matching firmware, reconnect USB, and run preflight again.');
    }
    return {
      state: 'incompatible',
      blocksStart: true,
      headline: 'Release mismatch — Start blocked',
      message: mismatches.map(detail => `${detail.label}: expected ${detail.expected}, received ${detail.observed}`).join('; '),
      guidance: [...guidance],
      details,
      checkedAtMs: Date.now()
    };
  }

  if (unknowns.length) {
    return {
      state: 'unverified',
      blocksStart: false,
      headline: 'Legacy handshake — compatibility unverified',
      message:
        `${unknowns.map(detail => detail.label).join(', ')} ${unknowns.length === 1 ? 'was' : 'were'} not reported. ` +
        'Operational capture remains available, but treat the session as unverified and exclude it from confirmatory analysis.',
      guidance: [
        'Restart the trainer and reload the dashboard to obtain a complete release handshake.',
        'Flash current firmware before confirmatory data collection if firmware or serial identity cannot be verified.'
      ],
      details,
      checkedAtMs: Date.now()
    };
  }

  return {
    state: 'compatible',
    blocksStart: false,
    headline: 'Release-compatible',
    message: `Dashboard, trainer, firmware contract, ${SERIAL_PROTOCOL}/${SERIAL_WIDTH_EXPECTED}-column serial data, and API schemas agree.`,
    guidance: [],
    details,
    checkedAtMs: Date.now()
  };
}

@Injectable({ providedIn: 'root' })
export class ReleaseCompatibilityService {
  private readonly api = inject(ApiService);
  private readonly current = signal<ReleaseCompatibilitySummary>(CHECKING);

  readonly summary = this.current.asReadonly();

  handshake(): ClientReleaseHandshake {
    return clientReleaseHandshake();
  }

  async refresh(statusHint?: ControlStatus | null): Promise<ReleaseCompatibilitySummary> {
    this.current.set(CHECKING);
    const [statusResult, versionResult] = await Promise.allSettled([
      statusHint
        ? Promise.resolve(statusHint)
        : this.api.request<ControlStatus>('/api/status', undefined, false, 5000),
      this.api.request<ReleaseVersionPayload>('/api/version', undefined, false, 5000)
    ]);
    const status = statusResult.status === 'fulfilled' ? statusResult.value : statusHint;
    const version = versionResult.status === 'fulfilled' ? versionResult.value : null;
    const evaluated = evaluateReleaseCompatibility(status, version);
    this.current.set(evaluated);
    return evaluated;
  }
}
