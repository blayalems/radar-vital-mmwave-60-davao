import { PRODUCT_VERSION } from './app-meta';

export const SERIAL_PROTOCOL = 'v15.2';
export const SERIAL_WIDTH_EXPECTED = 222;
export const CONTROL_API_SCHEMA = 'rvt-control-api-v12.0';
export const STUDY_SESSION_SCHEMA = 'rvt-study-session-v16.5.9';

export interface ClientReleaseHandshake {
  product_version: string;
  dashboard_version: string;
  serial_protocol: string;
  serial_width_expected: number;
  schema_versions: {
    control_api: string;
    study_session: string;
  };
}

export function clientReleaseHandshake(): ClientReleaseHandshake {
  return {
    product_version: PRODUCT_VERSION,
    dashboard_version: PRODUCT_VERSION,
    serial_protocol: SERIAL_PROTOCOL,
    serial_width_expected: SERIAL_WIDTH_EXPECTED,
    schema_versions: {
      control_api: CONTROL_API_SCHEMA,
      study_session: STUDY_SESSION_SCHEMA
    }
  };
}
