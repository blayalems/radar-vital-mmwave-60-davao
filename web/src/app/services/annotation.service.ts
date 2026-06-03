import { Injectable, inject, signal } from '@angular/core';
import { ApiService } from './api.service';
import { StateService } from './state.service';
import { ChartAnnotation } from '../models/rvt.models';

@Injectable({
  providedIn: 'root'
})
export class AnnotationService {
  private readonly api = inject(ApiService);
  private readonly state = inject(StateService);

  // Local in-memory database for sandbox session annotations
  private readonly sandboxAnnotations = signal<Record<string, ChartAnnotation[]>>({});

  // Active annotations for the current loaded session
  readonly currentAnnotations = signal<ChartAnnotation[]>([]);

  async loadAnnotations(sessionId: string): Promise<void> {
    if (!sessionId) {
      this.currentAnnotations.set([]);
      return;
    }
    const isSandbox = sessionId === 'mock' || sessionId.startsWith('mock') || this.state.autoDemoActive() || this.state.demoMode() || (this.state.ctlOn() && this.state.ctlStatus()?.mode === 'sandbox');
    if (isSandbox) {
      const sandboxMap = this.sandboxAnnotations();
      this.currentAnnotations.set(sandboxMap[sessionId] || []);
      return;
    }

    try {
      const resp = await this.api.request<{ ok?: boolean; annotations?: ChartAnnotation[] }>(`/api/sessions/${sessionId}/annotations`);
      if (resp && Array.isArray(resp.annotations)) {
        this.currentAnnotations.set(resp.annotations);
      }
    } catch (e) {
      console.warn('Failed to load annotations from API, falling back to empty', e);
      this.currentAnnotations.set([]);
    }
  }

  async saveAnnotation(chartKey: string, ann: Omit<ChartAnnotation, 'chart_key'>, action: 'upsert' | 'delete' = 'upsert'): Promise<void> {
    const sessionId = this.state.currentSessionId();
    if (!sessionId) return;

    const fullAnn: ChartAnnotation = {
      ...ann,
      chart_key: chartKey
    };

    const isSandbox = sessionId === 'mock' || sessionId.startsWith('mock') || this.state.autoDemoActive() || this.state.demoMode() || (this.state.ctlOn() && this.state.ctlStatus()?.mode === 'sandbox');
    if (isSandbox) {
      this.sandboxAnnotations.update(map => {
        const list = map[sessionId] ? [...map[sessionId]] : [];
        if (action === 'delete') {
          const filtered = list.filter(a => !(a.id === ann.id && a.chart_key === chartKey));
          return { ...map, [sessionId]: filtered };
        } else {
          const filtered = list.filter(a => !(a.id === ann.id && a.chart_key === chartKey));
          filtered.push(fullAnn);
          return { ...map, [sessionId]: filtered };
        }
      });
      // Sync current annotations signal
      this.currentAnnotations.set(this.sandboxAnnotations()[sessionId] || []);
      return;
    }

    try {
      await this.api.request<{ ok?: boolean; entry?: ChartAnnotation }>('/api/session/annotations', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          action,
          chart_key: chartKey,
          annotation: ann
        })
      });
      // Reload annotations to be fully in sync with backend logic
      await this.loadAnnotations(sessionId);
    } catch (e) {
      console.error('Failed to save annotation', e);
      throw e;
    }
  }
}
