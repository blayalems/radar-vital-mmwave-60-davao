import { PreflightCheck } from '../../models/rvt.models';
import { isStartBlockingPreflightCheck, mergeRadarPortChoices, sessionStartTimestampMs } from './home.component';

describe('HomeComponent start readiness helpers', () => {
  const check = (id: string, status: string): PreflightCheck => ({
    id,
    label: id,
    status,
    description: ''
  });

  it('keeps COM7 and COM10 available while preserving detected ports', () => {
    expect(mergeRadarPortChoices(['COM7'], ['COM10', 'COM7'], ['COM11'])).toEqual(['COM7', 'COM10', 'COM11']);
  });

  it('does not hard-block Start for BLE or probe failures', () => {
    expect(isStartBlockingPreflightCheck(check('ble_adapter', 'fail'))).toBe(false);
    expect(isStartBlockingPreflightCheck(check('ble_device_probe', 'fail'))).toBe(false);
    expect(isStartBlockingPreflightCheck(check('serial_port_probe', 'fail'))).toBe(false);
  });

  it('does not hard-block Start for package/source audit failures', () => {
    expect(isStartBlockingPreflightCheck(check('python_env', 'fail'))).toBe(false);
    expect(isStartBlockingPreflightCheck(check('firmware_file_present', 'fail'))).toBe(false);
  });

  it('hard-blocks Start for required collection storage and schema failures', () => {
    expect(isStartBlockingPreflightCheck(check('serial_port_list', 'fail'))).toBe(true);
    expect(isStartBlockingPreflightCheck(check('session_folder_writable', 'error'))).toBe(true);
    expect(isStartBlockingPreflightCheck(check('schema_hash_consistency', 'bad'))).toBe(true);
  });

  it('does not treat missing session timestamps as Unix epoch', () => {
    expect(sessionStartTimestampMs({})).toBeNull();
    expect(sessionStartTimestampMs({ started_at: '', started_ms: 0 })).toBeNull();
    expect(sessionStartTimestampMs({ started_at: '1970-01-01T00:00:00.000Z' })).toBeNull();
  });
});
