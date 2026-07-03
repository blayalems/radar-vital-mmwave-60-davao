import { PreflightCheck } from '../../models/rvt.models';
import { isStartBlockingPreflightCheck, mergeRadarPortChoices } from './home.component';

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

  it('hard-blocks Start for required local runtime and storage failures', () => {
    expect(isStartBlockingPreflightCheck(check('python_env', 'fail'))).toBe(true);
    expect(isStartBlockingPreflightCheck(check('session_folder_writable', 'error'))).toBe(true);
    expect(isStartBlockingPreflightCheck(check('schema_hash_consistency', 'bad'))).toBe(true);
  });
});
