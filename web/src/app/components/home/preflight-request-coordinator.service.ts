import { Injectable, inject } from '@angular/core';
import { PreflightCheck, SetupState } from '../../models/rvt.models';
import { ApiService } from '../../services/api.service';

const PREFLIGHT_REQUEST_TIMEOUT_MS = 30000;
export type PreflightSetup = Pick<SetupState, 'radar_port' | 'ble_address'>;

export function preflightSetupFingerprint(setup: PreflightSetup): string {
  return JSON.stringify([
    String(setup.radar_port || '').trim(),
    String(setup.ble_address || '').trim()
  ]);
}

export type PreflightRunResult =
  | { readonly status: 'applied'; readonly fingerprint: string; readonly checks: PreflightCheck[] }
  | { readonly status: 'empty' }
  | { readonly status: 'stale' }
  | { readonly status: 'superseded' }
  | { readonly status: 'error'; readonly message: string };

export type SinglePreflightResult =
  | { readonly status: 'applied'; readonly check: PreflightCheck; readonly preserveSnapshot: boolean }
  | { readonly status: 'stale' }
  | { readonly status: 'superseded' }
  | { readonly status: 'error'; readonly message: string };

@Injectable({ providedIn: 'root' })
export class PreflightRequestCoordinator {
  private readonly api = inject(ApiService);
  private generation = 0;
  private validFingerprint = '';

  get lastValidFingerprint(): string {
    return this.validFingerprint;
  }

  async runAll(
    setup: PreflightSetup,
    currentFingerprint: () => string
  ): Promise<PreflightRunResult> {
    const generation = ++this.generation;
    const fingerprint = preflightSetupFingerprint(setup);
    this.validFingerprint = '';
    const query = new URLSearchParams({
      port: setup.radar_port,
      address: setup.ble_address
    });
    try {
      const response = await this.api.request<{ checks?: PreflightCheck[] }>(
        `/api/preflight?${query.toString()}`,
        undefined,
        false,
        PREFLIGHT_REQUEST_TIMEOUT_MS
      );
      if (generation !== this.generation) return { status: 'superseded' };
      if (fingerprint !== currentFingerprint()) return { status: 'stale' };
      if (!Array.isArray(response?.checks)) return { status: 'empty' };
      this.validFingerprint = fingerprint;
      return { status: 'applied', fingerprint, checks: response.checks };
    } catch (error) {
      if (generation !== this.generation) return { status: 'superseded' };
      if (fingerprint !== currentFingerprint()) return { status: 'stale' };
      const message = error instanceof Error ? error.message : 'Preflight unavailable.';
      return {
        status: 'error',
        message: message === 'Request timeout'
          ? 'Preflight timed out while probing hardware. Re-run the checks; Start only blocks on collection, storage, schema, and clock failures.'
          : message
      };
    }
  }

  async runSingle(
    checkId: string,
    setup: PreflightSetup,
    currentFingerprint: () => string
  ): Promise<SinglePreflightResult> {
    const generation = ++this.generation;
    const fingerprint = preflightSetupFingerprint(setup);
    const preserveSnapshot = this.validFingerprint === fingerprint;
    this.validFingerprint = '';
    try {
      const response = await this.api.request<PreflightCheck | { check?: PreflightCheck }>(
        `/api/preflight/${checkId}`,
        {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            port: setup.radar_port,
            address: setup.ble_address
          })
        }
      );
      if (generation !== this.generation) return { status: 'superseded' };
      if (fingerprint !== currentFingerprint()) return { status: 'stale' };
      const check = 'check' in response && response.check ? response.check : response as PreflightCheck;
      if (preserveSnapshot && check?.id) this.validFingerprint = fingerprint;
      return { status: 'applied', check, preserveSnapshot };
    } catch (error) {
      if (generation !== this.generation) return { status: 'superseded' };
      if (fingerprint !== currentFingerprint()) return { status: 'stale' };
      return {
        status: 'error',
        message: error instanceof Error ? error.message : 'Preflight check unavailable.'
      };
    }
  }
}
