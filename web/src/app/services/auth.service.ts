import { Injectable, inject, signal } from '@angular/core';
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

  private lockoutTimer: any = null;

  constructor() {
    this.checkAuthInit();
  }

  private checkAuthInit(): void {
    try {
      const token = sessionStorage.getItem(OPERATOR_TOKEN_KEY);
      if (token) {
        this.isLocked.set(false);
        const label = this.state.setup().operator_label;
        if (label) {
          this.currentOperator.set({
            operator_id: '',
            display_name: label,
            initials: label.split(' ').map(n => n[0]).join('').toUpperCase().slice(0, 2)
          });
        }
      } else {
        this.isLocked.set(true);
      }
    } catch (_) {
      this.isLocked.set(true);
    }
  }

  async loadProfiles(): Promise<void> {
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
      });

      if (res?.token) {
        sessionStorage.setItem(OPERATOR_TOKEN_KEY, res.token);
        this.currentOperator.set(res.operator);
        this.isLocked.set(false);
        this.loginError.set(null);
        this.lockoutRetryAfter.set(0);

        this.state.setup.update(s => ({
          ...s,
          operator_label: res.operator.display_name
        }));
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
        this.startLockoutCountdown(retryAfter);
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
      });

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

  private startLockoutCountdown(seconds: number): void {
    if (this.lockoutTimer) clearInterval(this.lockoutTimer);
    this.lockoutRetryAfter.set(seconds);
    this.lockoutTimer = setInterval(() => {
      const current = this.lockoutRetryAfter();
      if (current <= 1) {
        this.lockoutRetryAfter.set(0);
        this.loginError.set(null);
        clearInterval(this.lockoutTimer);
        this.lockoutTimer = null;
      } else {
        this.lockoutRetryAfter.set(current - 1);
        this.loginError.set(`Too many failed attempts. Try again in ${current - 1} seconds.`);
      }
    }, 1000);
  }
}
