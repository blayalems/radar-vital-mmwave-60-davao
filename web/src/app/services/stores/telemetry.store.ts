import { Injectable, signal } from '@angular/core';
import { LivePayload, SnapshotRecord } from '../../models/rvt.models';

@Injectable({
  providedIn: 'root'
})
export class TelemetryStore {
  liveBufferSeconds = signal<number>(60);
  maxChartPoints = signal<number>(3600);

  lastPayload = signal<LivePayload | null>(null);
  lastLivePayload = signal<LivePayload | null>(null);
  liveReceivedAt = signal<number | null>(null);
  telemetryStale = signal<boolean>(true);

  snaps = signal<SnapshotRecord[]>([]);
  snapNotes = signal<Record<string, string>>({});
  activeSnapNoteId = signal<string | null>(null);

  spark = signal<{ hr: number[]; rr: number[]; fps: number[]; dist: number[] }>({
    hr: [],
    rr: [],
    fps: [],
    dist: []
  });

  waveformSeekAt = signal<number | null>(null);
}
