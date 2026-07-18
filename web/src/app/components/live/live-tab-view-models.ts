import type { Signal } from '@angular/core';
import type { SnapshotRecord } from '../../models/rvt.models';
import type { StateService } from '../../services/state.service';

export interface LiveSnapsTabViewModel {
  readonly state: StateService;
  readonly selectedCompareSnaps: Signal<SnapshotRecord[]>;
  readonly compareSelection: Signal<string[]>;
  readonly sortedSnaps: () => SnapshotRecord[];
  readonly captureSnapshot: () => void;
  readonly clearAllSnaps: () => Promise<void>;
  readonly snapDelta: (key: 'reported_hr' | 'reported_rr' | 'distance_cm') => string;
  readonly clearCompareSelection: () => void;
  readonly moveSnap: (snapId: string, direction: -1 | 1) => void;
  readonly toggleCompareSnap: (snapId: string) => void;
  readonly deleteSnap: (snapId: string) => void;
  readonly updateSnapNote: (snapId: string, value: string) => void;
}

export interface LiveAuditTabViewModel {
  readonly state: StateService;
  readonly analysisMetric: (key: string) => string;
  readonly analysisNested: (recordKey: string, key: string, decimals?: number) => string;
  readonly metricText: (key: string, decimals?: number, suffix?: string) => string;
  readonly histogramSummary: (key: string) => string;
  readonly exportAuditLog: (format: 'json' | 'csv') => void;
  readonly eventTime: (event: string | Record<string, unknown>) => string;
  readonly formatEvent: (event: string | Record<string, unknown>) => string;
}
