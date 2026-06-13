import { Injectable, effect, inject, signal } from '@angular/core';
import { ApiService } from './api.service';
import { StateService } from './state.service';
import { OperatorProfile, OperatorProfilesResponse, LoginResponse } from '../models/rvt.models';
import { OPERATOR_TOKEN_KEY } from './rvt-storage-keys';

@Injectable({
  providedIn: 'root'
})
export class AuthService {
  private readonly api = inject(ApiService);
  private readonly state = inject(StateService);

  readonly currentOperator = signal<OperatorProfile | null>(null);
  readonly isLocked = signal<boolean>(true);
  readonly profiles = signal<OperatorProfile[]>([]);
  readonly bootstrapping = signal<boolean>(false);
  readonly loading = signal<boolean>(false);
  readonly loginError = signal<string | null>(null);
  readonly lockoutRetryAfter = signal<number>(0);
  readonly operatorLockouts = signal<Record<string, number>>({});

  private lockoutTimers = new Map<string, any>();

  constructor() {
    effect(() => {
      const status = this.state.ctlStatus();
      if (!this.loading() && !this.isLocked() && status?.reason === 'unauthenticated') {
        this.loginError.set('Operator session expired. Sign in again to continue.');
        this.lock();
      }
    });
    void this.checkAuthInit();
  }

  private async checkAuthInit(): Promise<void> {
    try {
      await this.waitForConnectionBootstrap();
      const token = sessionStorage.getItem(OPERATOR_TOKEN_KEY);
      if (!token) {
        this.isLocked.set(true);
        return;
      }

      this.loading.set(true);
      try {
        const res = await this.api.request<{ ok: boolean; operator?: OperatorProfile }>('/api/auth/validate');
        if (res?.ok && res.operator) {
          this.currentOperator.set(res.operator);
          this.state.setup.update(s => ({
            ...s,
            operator_label: res.operator?.display_name || s.operator_label
          }));
          this.isLocked.set(false);
          this.bootstrapping.set(false);
          await this.api.detectControlMode();
          this.notifyAuthenticated();
        } else {
          this.clearLocalSession();
        }
      } catch (err) {
        console.error('Failed to validate operator token on startup', err);
        this.clearLocalSession();
      } finally {
        this.loading.set(false);
      }
    } catch (_) {
      this.isLocked.set(true);
    }
  }

  lockoutForOperator(operatorId: string | undefined): number {
    if (!operatorId) return 0;
    return this.operatorLockouts()[operatorId] || 0;
  }

  async loadProfiles(): Promise<void> {
    await this.waitForConnectionBootstrap();
    this.loading.set(true);
    try {
      const res = await this.api.request<OperatorProfilesResponse>('/api/operator-profiles');
      const list = res?.profiles || [];
      this.profiles.set(list);
      this.bootstrapping.set(list.length === 0);
    } catch (err) {
      console.error('Failed to load operator profiles', err);
      if (this.state.ctlStatus()?.mode === 'sandbox') {
        this.bootstrapping.set(this.profiles().length === 0);
      }
    } finally {
      this.loading.set(false);
    }
  }

  async login(operator_id: string, pin: string): Promise<boolean> {
    this.loading.set(true);
    this.loginError.set(null);
    try {
      const res = await this.api.request<LoginResponse>('/api/auth/login', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ operator_id, pin })
      }, false, 30000);

      if (res?.token) {
        sessionStorage.setItem(OPERATOR_TOKEN_KEY, res.token);
        this.currentOperator.set(res.operator);
        this.clearUnauthenticatedStatus();
        this.loginError.set(null);
        this.lockoutRetryAfter.set(0);

        // Reset lockout for this operator on successful login
        this.operatorLockouts.update(map => {
          const next = { ...map };
          delete next[operator_id];
          return next;
        });
        const timer = this.lockoutTimers.get(operator_id);
        if (timer) {
          clearInterval(timer);
          this.lockoutTimers.delete(operator_id);
        }

        this.state.setup.update(s => ({
          ...s,
          operator_label: res.operator.display_name
        }));

        await this.api.detectControlMode();
        this.isLocked.set(false);
        this.notifyAuthenticated();
        return true;
      }
      return false;
    } catch (err: any) {
      console.error('Login failed', err);
      const msg = err.message || '';
      if (msg.includes('LOCKOUT_ACTIVE') || msg.toLowerCase().includes('lockout')) {
        let retryAfter = 30;
        const match = msg.match(/(\d+)\s*second/i);
        if (match) {
          retryAfter = parseInt(match[1], 10);
        }
        this.startLockoutCountdown(operator_id, retryAfter);
        this.loginError.set(`Too many failed attempts. Try again in ${retryAfter} seconds.`);
      } else {
        this.loginError.set(msg || 'Invalid PIN.');
      }
      return false;
    } finally {
      this.loading.set(false);
    }
  }

  async createProfile(displayName: string, initials: string, pin: string): Promise<boolean> {
    this.loading.set(true);
    this.loginError.set(null);
    try {
      const res = await this.api.request<{ ok: boolean; operator?: OperatorProfile }>('/api/operator-profiles', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ display_name: displayName, initials, pin })
      }, false, 30000);

      if (res?.ok && res.operator) {
        await this.loadProfiles();
        return await this.login(res.operator.operator_id, pin);
      }
      return false;
    } catch (err: any) {
      console.error('Failed to create profile', err);
      this.loginError.set(err.message || 'Failed to create profile.');
      return false;
    } finally {
      this.loading.set(false);
    }
  }

  async logout(): Promise<void> {
    const token = sessionStorage.getItem(OPERATOR_TOKEN_KEY);
    await this.revokeToken(token);
    this.clearLocalSession();
  }

  lock(): void {
    const token = sessionStorage.getItem(OPERATOR_TOKEN_KEY);
    this.clearLocalSession();
    this.revokeToken(token).catch(() => {});
  }

  private clearLocalSession(): void {
    sessionStorage.removeItem(OPERATOR_TOKEN_KEY);
    this.currentOperator.set(null);
    this.isLocked.set(true);
    void this.api.detectControlMode();
  }

  private clearUnauthenticatedStatus(): void {
    const status = this.state.ctlStatus();
    if (status?.reason !== 'unauthenticated') return;
    const { reason: _reason, ...nextStatus } = status;
    this.state.ctlStatus.set(nextStatus);
  }

  private async waitForConnectionBootstrap(): Promise<void> {
    await Promise.race([
      this.api.whenInitialized(),
      new Promise<void>(resolve => setTimeout(resolve, 500))
    ]).catch(() => undefined);
  }

  private notifyAuthenticated(): void {
    try {
      window.dispatchEvent(new CustomEvent('rvt-operator-authenticated'));
    } catch (_) {}
  }

  private async revokeToken(token: string | null): Promise<void> {
    if (!token) return;
    try {
      await this.api.request('/api/auth/logout', {
        method: 'POST',
        headers: { 'X-RVT-Auth': token }
      });
    } catch (_) {}
  }

  startLockoutCountdown(operatorIdOrSeconds: string | number, seconds?: number): void {
    let operatorId = '';
    let secs = 30;
    if (typeof operatorIdOrSeconds === 'number') {
      secs = operatorIdOrSeconds;
      operatorId = 'global';
    } else {
      operatorId = operatorIdOrSeconds;
      secs = seconds !== undefined ? seconds : 30;
    }

    const existing = this.lockoutTimers.get(operatorId);
    if (existing) clearInterval(existing);

    this.operatorLockouts.update(map => ({ ...map, [operatorId]: secs }));
    this.lockoutRetryAfter.set(secs);

    const timer = setInterval(() => {
      const current = this.operatorLockouts()[operatorId] || 0;
      if (current <= 1) {
        this.operatorLockouts.update(map => {
          const next = { ...map };
          delete next[operatorId];
          return next;
        });
        this.lockoutRetryAfter.set(0);
        this.loginError.set(null);
        const t = this.lockoutTimers.get(operatorId);
        if (t) clearInterval(t);
        this.lockoutTimers.delete(operatorId);
      } else {
        this.operatorLockouts.update(map => ({ ...map, [operatorId]: current - 1 }));
        this.lockoutRetryAfter.set(current - 1);
      }
    }, 1000);

    this.lockoutTimers.set(operatorId, timer);
  }
}
