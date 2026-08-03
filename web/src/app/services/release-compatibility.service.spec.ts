import { ControlStatus } from '../models/rvt.models';
import { PRODUCT_VERSION } from './app-meta';
import {
  CONTROL_API_SCHEMA,
  SERIAL_PROTOCOL,
  SERIAL_WIDTH_EXPECTED,
  STUDY_SESSION_SCHEMA,
  clientReleaseHandshake,
  evaluateReleaseCompatibility
} from './release-compatibility.service';

function matchingStatus(): ControlStatus {
  return {
    ok: true,
    trainer_version: PRODUCT_VERSION,
    dashboard_version: PRODUCT_VERSION,
    firmware_expected: `v${PRODUCT_VERSION}`,
    serial_protocol: SERIAL_PROTOCOL,
    serial_width_expected: SERIAL_WIDTH_EXPECTED,
    schema_versions: {
      control_api: CONTROL_API_SCHEMA,
      study_session: STUDY_SESSION_SCHEMA
    }
  };
}

function matchingVersion(): Record<string, unknown> {
  return {
    product_version: PRODUCT_VERSION,
    trainer: PRODUCT_VERSION,
    dashboard: PRODUCT_VERSION,
    firmware_expected: `v${PRODUCT_VERSION}`,
    serial_protocol: SERIAL_PROTOCOL,
    serial_width_expected: SERIAL_WIDTH_EXPECTED,
    schema_versions: {
      control_api: CONTROL_API_SCHEMA,
      study_session: STUDY_SESSION_SCHEMA
    }
  };
}

describe('release compatibility evaluator', () => {
  it('sends the nested client handshake agreed with the trainer', () => {
    expect(clientReleaseHandshake()).toEqual({
      product_version: PRODUCT_VERSION,
      dashboard_version: PRODUCT_VERSION,
      serial_protocol: 'v15.2',
      serial_width_expected: 222,
      schema_versions: {
        control_api: 'rvt-control-api-v12.0',
        study_session: 'rvt-study-session-v16.5.1'
      }
    });
  });

  it('verifies the fully matching trainer/dashboard/protocol/schema handshake', () => {
    const summary = evaluateReleaseCompatibility(
      matchingStatus(),
      matchingVersion()
    );

    expect(summary.state).toBe('compatible');
    expect(summary.blocksStart).toBe(false);
    expect(summary.details.every(detail => detail.state === 'match')).toBe(true);
  });

  it.each([
    ['trainer release', { trainer: '16.5.2' }, 'Restart the trainer'],
    ['dashboard release', { dashboard: '16.5.2' }, 'Reload the dashboard'],
    ['serial protocol', { serial_protocol: 'v15.1' }, 'Flash the matching firmware'],
    ['serial width', { serial_width_expected: 219 }, 'Flash the matching firmware']
  ])(
    'blocks Start and gives recovery guidance for a wrong %s',
    (_caseName, patch, expectedGuidance) => {
      const summary = evaluateReleaseCompatibility(
        matchingStatus(),
        { ...matchingVersion(), ...patch }
      );

      expect(summary.state).toBe('incompatible');
      expect(summary.blocksStart).toBe(true);
      expect(summary.guidance.join(' ')).toContain(expectedGuidance);
    }
  );

  it.each([
    ['control_api', 'rvt-control-api-v11.0'],
    ['study_session', 'rvt-study-session-v16.4.0']
  ])('blocks Start for a wrong %s schema', (schema, wrongValue) => {
    const version = matchingVersion();
    version['schema_versions'] = {
      ...(version['schema_versions'] as Record<string, unknown>),
      [schema]: wrongValue
    };

    const summary = evaluateReleaseCompatibility(matchingStatus(), version);

    expect(summary.state).toBe('incompatible');
    expect(summary.blocksStart).toBe(true);
    expect(summary.guidance.join(' ')).toContain('Reload the dashboard');
  });

  it('keeps a legacy unknown handshake operational but not confirmatory', () => {
    const summary = evaluateReleaseCompatibility(
      { ok: true } as ControlStatus,
      {}
    );

    expect(summary.state).toBe('unverified');
    expect(summary.blocksStart).toBe(false);
    expect(summary.message.toLowerCase()).toContain('exclude');
    expect(summary.message.toLowerCase()).toContain('confirmatory');
    expect(summary.guidance.join(' ').toLowerCase()).toContain('reload');
  });
});
