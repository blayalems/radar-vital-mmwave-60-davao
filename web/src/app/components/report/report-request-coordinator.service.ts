import { Injectable, inject } from '@angular/core';
import {
  SessionDataPayload,
  SessionNotesPayload,
  SessionRecord,
  SessionSignoff
} from '../../models/rvt.models';
import { ApiService } from '../../services/api.service';

export interface ReportSessionLoad {
  readonly sessionId: string;
  readonly summary: SessionRecord;
  readonly data: SessionDataPayload;
  readonly comparison: {
    selected?: SessionRecord | null;
    previous?: SessionRecord | null;
    best?: SessionRecord | null;
  };
  readonly analysisStatus: { status?: string; progress_pct?: number; last_line?: string };
  readonly notes: SessionNotesPayload;
  readonly signoff: SessionSignoff;
}

export interface ReportComparisonLoad {
  readonly sessionId: string;
  readonly summary: SessionRecord;
  readonly rows: Array<Record<string, number | string | null>>;
}

@Injectable({ providedIn: 'root' })
export class ReportRequestCoordinator {
  private readonly api = inject(ApiService);
  private primaryEpoch = 0;
  private comparisonEpoch = 0;

  invalidateSelection(): void {
    ++this.primaryEpoch;
    ++this.comparisonEpoch;
  }

  async loadSession(sessionId: string): Promise<ReportSessionLoad | null> {
    const epoch = ++this.primaryEpoch;
    ++this.comparisonEpoch;
    const path = `/api/sessions/${encodeURIComponent(sessionId)}`;
    try {
      const [summary, data, comparison, analysisStatus, notes, signoff] = await Promise.all([
        this.api.request<SessionRecord>(`${path}/summary`),
        this.api.request<SessionDataPayload>(`${path}/data?points=1000`),
        this.api.request<ReportSessionLoad['comparison']>(`${path}/compare`),
        this.api.request<ReportSessionLoad['analysisStatus']>(`${path}/analyse/status`),
        this.api.request<SessionNotesPayload>(`${path}/notes`),
        this.api.request<SessionSignoff>(`${path}/signoff`)
      ]);
      if (epoch !== this.primaryEpoch) return null;
      return { sessionId, summary, data, comparison, analysisStatus, notes, signoff };
    } catch (error) {
      if (epoch !== this.primaryEpoch) return null;
      throw error;
    }
  }

  async loadComparison(sessionId: string): Promise<ReportComparisonLoad | null> {
    const epoch = ++this.comparisonEpoch;
    const path = `/api/sessions/${encodeURIComponent(sessionId)}`;
    try {
      const [summary, data] = await Promise.all([
        this.api.request<SessionRecord>(`${path}/summary`),
        this.api.request<{ rows?: Array<Record<string, number | string | null>> }>(`${path}/data?points=1000`)
      ]);
      if (epoch !== this.comparisonEpoch) return null;
      return {
        sessionId,
        summary,
        rows: Array.isArray(data?.rows) ? data.rows : []
      };
    } catch (error) {
      if (epoch !== this.comparisonEpoch) return null;
      throw error;
    }
  }

  invalidateComparison(): void {
    ++this.comparisonEpoch;
  }

  saveNotes(sessionId: string, reviewSummary: string): Promise<SessionNotesPayload> {
    return this.api.request<SessionNotesPayload>(`/api/sessions/${encodeURIComponent(sessionId)}/notes`, {
      method: 'PUT',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ review_summary: reviewSummary })
    });
  }

  saveSignoff(
    sessionId: string,
    signoff: Pick<SessionSignoff, 'operator_name' | 'initials' | 'validation_comment'>
  ): Promise<SessionSignoff> {
    return this.api.request<SessionSignoff>(`/api/sessions/${encodeURIComponent(sessionId)}/signoff`, {
      method: 'PUT',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(signoff)
    });
  }
}
