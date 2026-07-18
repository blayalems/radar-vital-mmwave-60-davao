import type { Signal, WritableSignal } from '@angular/core';
import type { ChartAnnotation, SnapshotRecord } from '../../models/rvt.models';
import type { StateService } from '../../services/state.service';

export type LiveTrendRange = 30 | 60 | 120 | 'max';

export interface LiveBiasBucket {
  readonly label: string;
  readonly bias: number;
  readonly count: number;
  readonly width: number;
}

interface LiveChartTabViewModel {
  readonly state: StateService;
  readonly seriesNumbers: (...keys: string[]) => number[];
  readonly getAnnotationsFor: (chartKey: string) => ChartAnnotation[];
  readonly deleteAnnotation: (chartKey: string, annotation: ChartAnnotation) => Promise<void>;
  readonly handleChartClick: (event: MouseEvent, chartKey: string) => void;
  readonly downloadChart: (canvas: HTMLCanvasElement, label: string) => void;
}

export interface LiveWavesTabViewModel extends LiveChartTabViewModel {
  readonly showBreathTable: WritableSignal<boolean>;
  readonly showHeartTable: WritableSignal<boolean>;
  readonly chartLabel: (label: string, key: string, unit: string) => string;
  readonly qualityPercent: (key: string) => number;
  readonly qualityLabel: (key: string) => string;
  readonly ribbonSegments: (kind: 'heart' | 'breath') => string[];
  readonly ribbonLabel: (kind: 'heart' | 'breath') => string;
  readonly resetTrendRange: () => void;
}

export interface LiveTrendTabViewModel extends LiveChartTabViewModel {
  readonly trendRange: Signal<LiveTrendRange>;
  readonly trendRangeLimit: Signal<number>;
  readonly ghostSessionActive: Signal<boolean>;
  readonly ghostSessionLabel: Signal<string | null>;
  readonly ghostHrData: Signal<number[]>;
  readonly ghostRrData: Signal<number[]>;
  readonly setTrendRange: (range: LiveTrendRange) => void;
  readonly resetTrendRange: () => void;
  readonly trendRangeLabel: () => string;
  readonly trimTrend: (points: number[]) => number[];
  readonly trendChartLabel: (
    label: string,
    key: string,
    unit: string,
    type: 'hr' | 'rr'
  ) => string;
  readonly ghostPointCount: (type: 'hr' | 'rr') => number;
  readonly ghostLegendText: (type: 'hr' | 'rr') => string;
  readonly toggleGhostSession: () => void;
  readonly metricState: (key: string) => 'Yes' | 'No' | '--';
  readonly metricText: (key: string, decimals?: number, suffix?: string) => string;
  readonly metricLabel: (key: string) => string;
}

export interface LiveHrTabViewModel extends LiveTrendTabViewModel {
  readonly showTable: WritableSignal<boolean>;
  readonly biasBuckets: Signal<LiveBiasBucket[]>;
}

export interface LiveRrTabViewModel extends LiveTrendTabViewModel {
  readonly showTable: WritableSignal<boolean>;
  readonly warnings: Signal<string[]>;
}

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
