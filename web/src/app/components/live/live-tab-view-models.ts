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

export interface LiveBlandAltmanStats {
  readonly meanDiff: number;
  readonly lo: number;
  readonly hi: number;
  readonly exceeds: boolean;
  readonly unit: string;
  readonly thresh: number;
}

export interface LiveKpiStripViewModel {
  readonly state: StateService;
  readonly kpiOrder: Signal<string[]>;
  readonly liveStatusAnnouncement: Signal<string>;
  readonly openKpiZoomDialog: (metric: 'hr' | 'rr' | 'fps' | 'dist') => void;
  readonly onKpiKeydown: (event: KeyboardEvent, kpi: string) => void;
  readonly onKpiDragStart: (event: PointerEvent, kpi: string) => void;
  readonly onKpiDragMove: (event: PointerEvent) => void;
  readonly onKpiDragEnd: (event: PointerEvent) => void;
  readonly kpiControlLabel: (label: string, key: string, unit: string) => string;
  readonly kpiReorderDescription: (kpi: string) => string;
  readonly kpiButtonTitle: (kpi: string) => string;
  readonly hrConfidencePct: Signal<number | null>;
  readonly hrConfidenceSource: Signal<string>;
  readonly rrConfidencePct: Signal<number | null>;
  readonly rrConfidenceSource: Signal<string>;
  readonly hrHolding: Signal<boolean>;
  readonly rrHolding: Signal<boolean>;
  readonly hrValueDisplay: () => string;
  readonly rrValueDisplay: () => string;
  readonly chartLabel: (label: string, key: string, unit: string) => string;
  readonly frameRateControlLabel: () => string;
  readonly frameRateLabel: () => string;
  readonly computedFps: Signal<string>;
  readonly targetRangeControlLabel: () => string;
}

export interface LiveOverviewTabViewModel extends LiveKpiStripViewModel {
  readonly baMetric: WritableSignal<'hr' | 'rr'>;
  readonly baStats: Signal<LiveBlandAltmanStats | null>;
  readonly sessionNotes: () => string;
  readonly customTag: () => string;
  readonly setCustomTag: (value: string) => void;
  readonly targetRangeDisplay: () => string;
  readonly targetZoneLabel: () => string;
  readonly targetZoneTone: () => 'ok' | 'warn' | 'muted';
  readonly signalQualityDisplay: () => string;
  readonly signalQualitySub: () => string;
  readonly signalQualityTone: () => 'ok' | 'warn' | 'muted';
  readonly motionActive: Signal<boolean>;
  readonly motionSummary: () => string;
  readonly motionSub: () => string;
  readonly exportSessionNotes: () => void;
  readonly saveSessionNotes: (value: string) => void;
  readonly addQuickTag: (tag: string) => void;
  readonly recordObservation: () => Promise<void>;
  readonly addCustomTag: () => void;
  readonly captureSnapshot: () => void;
  readonly metricState: (key: string) => 'Yes' | 'No' | '--';
  readonly metricLabel: (key: string) => string;
  readonly metricText: (key: string, decimals?: number, suffix?: string) => string;
  readonly hasBleRef: () => boolean;
  readonly baChartAriaLabel: () => string;
  readonly requestCanvasDraw: () => void;
  readonly formatFault: (fault: string | Record<string, unknown>) => string;
}

interface LiveChartTabViewModel {
  readonly state: StateService;
  readonly deleteAnnotation: (chartKey: string, annotation: ChartAnnotation) => Promise<void>;
  readonly handleChartClick: (event: MouseEvent, chartKey: string) => void | Promise<void>;
  readonly downloadChart: (canvas: HTMLCanvasElement, label: string) => void;
}

export interface LiveWavesTabViewModel extends LiveChartTabViewModel {
  readonly showBreathTable: WritableSignal<boolean>;
  readonly showHeartTable: WritableSignal<boolean>;
  readonly breathSeries: Signal<number[]>;
  readonly heartSeries: Signal<number[]>;
  readonly breathChartLabel: Signal<string>;
  readonly heartChartLabel: Signal<string>;
  readonly breathAnnotations: Signal<ChartAnnotation[]>;
  readonly heartAnnotations: Signal<ChartAnnotation[]>;
  readonly breathQualityPercent: Signal<number>;
  readonly heartQualityPercent: Signal<number>;
  readonly breathQualityLabel: Signal<string>;
  readonly heartQualityLabel: Signal<string>;
  readonly breathRibbonSegments: Signal<string[]>;
  readonly heartRibbonSegments: Signal<string[]>;
  readonly breathRibbonLabel: Signal<string>;
  readonly heartRibbonLabel: Signal<string>;
  readonly resetTrendRange: () => void;
}

export interface LiveTrendTabViewModel extends LiveChartTabViewModel {
  readonly trendRange: Signal<LiveTrendRange>;
  readonly trendRangeLimit: Signal<number>;
  readonly trendRangeLabel: Signal<string>;
  readonly ghostSessionActive: Signal<boolean>;
  readonly ghostSessionLabel: Signal<string | null>;
  readonly ghostHrData: Signal<number[]>;
  readonly ghostRrData: Signal<number[]>;
  readonly setTrendRange: (range: LiveTrendRange) => void;
  readonly resetTrendRange: () => void;
  readonly toggleGhostSession: () => void | Promise<void>;
  readonly metricState: (key: string) => 'Yes' | 'No' | '--';
  readonly metricText: (key: string, decimals?: number, suffix?: string) => string;
  readonly metricLabel: (key: string) => string;
}

export interface LiveHrTabViewModel extends LiveTrendTabViewModel {
  readonly showTable: WritableSignal<boolean>;
  readonly trimmedData: Signal<number[]>;
  readonly chartAnnotations: Signal<ChartAnnotation[]>;
  readonly chartAriaLabel: Signal<string>;
  readonly ghostSamplesCount: Signal<number>;
  readonly ghostLegend: Signal<string>;
  readonly biasBuckets: Signal<LiveBiasBucket[]>;
}

export interface LiveRrTabViewModel extends LiveTrendTabViewModel {
  readonly showTable: WritableSignal<boolean>;
  readonly trimmedData: Signal<number[]>;
  readonly chartAnnotations: Signal<ChartAnnotation[]>;
  readonly chartAriaLabel: Signal<string>;
  readonly ghostSamplesCount: Signal<number>;
  readonly ghostLegend: Signal<string>;
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
