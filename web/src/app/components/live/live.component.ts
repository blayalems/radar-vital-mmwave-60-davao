import { Component, inject, OnInit, OnDestroy, ElementRef, ViewChild, AfterViewInit, effect } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { Router } from '@angular/router';

// Angular Material 3 modules
import { MatCardModule } from '@angular/material/card';
import { MatTabsModule } from '@angular/material/tabs';
import { MatButtonModule } from '@angular/material/button';
import { MatIconModule } from '@angular/material/icon';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatInputModule } from '@angular/material/input';
import { MatTableModule } from '@angular/material/table';

import { StateService } from '../../services/state.service';
import { ApiService } from '../../services/api.service';
import { TelemetryService } from '../../services/telemetry.service';
import { AudioService } from '../../services/audio.service';

@Component({
  selector: 'app-live',
  standalone: true,
  imports: [
    CommonModule,
    FormsModule,
    MatCardModule,
    MatTabsModule,
    MatButtonModule,
    MatIconModule,
    MatFormFieldModule,
    MatInputModule,
    MatTableModule
  ],
  templateUrl: './live.component.html',
  styleUrl: './live.component.css'
})
export class LiveComponent implements OnInit, OnDestroy, AfterViewInit {
  protected readonly state = inject(StateService);
  protected readonly api = inject(ApiService);
  protected readonly telemetry = inject(TelemetryService);
  protected readonly audio = inject(AudioService);
  private readonly router = inject(Router);

  protected readonly Math = Math;

  // Canvas element references for dynamic renderers
  @ViewChild('breathCanvas', { static: false }) breathCanvas!: ElementRef<HTMLCanvasElement>;
  @ViewChild('heartCanvas', { static: false }) heartCanvas!: ElementRef<HTMLCanvasElement>;
  @ViewChild('hrTrendCanvas', { static: false }) hrTrendCanvas!: ElementRef<HTMLCanvasElement>;
  @ViewChild('rrTrendCanvas', { static: false }) rrTrendCanvas!: ElementRef<HTMLCanvasElement>;

  // Mini sparkline references
  @ViewChild('overviewHrSpark', { static: false }) overviewHrSpark!: ElementRef<HTMLCanvasElement>;
  @ViewChild('overviewRrSpark', { static: false }) overviewRrSpark!: ElementRef<HTMLCanvasElement>;
  @ViewChild('overviewFpsSpark', { static: false }) overviewFpsSpark!: ElementRef<HTMLCanvasElement>;
  @ViewChild('overviewDistSpark', { static: false }) overviewDistSpark!: ElementRef<HTMLCanvasElement>;

  private animeFrameId: number | null = null;
  
  // Local active tab selector
  activeTabIndex = 0;
  sessionNotesInput = '';

  constructor() {
    effect(() => {
      const t = this.state.activeTab();
      const tabs = ['tab-overview', 'tab-waves', 'tab-hr', 'tab-rr', 'tab-snaps', 'tab-audit'];
      const idx = tabs.indexOf(t);
      if (idx !== -1) {
        this.activeTabIndex = idx;
      }
    });
  }

  ngOnInit() {
    this.sessionNotesInput = this.state.sessionNotes()[this.state.currentSessionId() || ''] || '';
  }

  ngAfterViewInit() {
    this.initCanvasDrawing();
  }

  ngOnDestroy() {
    if (this.animeFrameId) {
      cancelAnimationFrame(this.animeFrameId);
    }
  }

  onTabChange(event: any) {
    const tabs = ['tab-overview', 'tab-waves', 'tab-hr', 'tab-rr', 'tab-snaps', 'tab-audit'];
    this.activeTabIndex = event.index;
    this.state.activeTab.set(tabs[event.index]);
    this.state.triggerHaptic('tap');
  }

  togglePause() {
    this.state.paused.set(!this.state.paused());
    this.state.triggerHaptic('tap');
  }

  async stopSession() {
    this.state.triggerHaptic('confirm');
    if (confirm('Are you sure you want to stop this monitoring session and compile the report?')) {
      try {
        this.state.ctlStopPending.set(true);
        const r = await this.api.request('/api/session/stop');
        if (r && r.ok) {
          this.state.currentSessionId.set(null);
          this.audio.speakAlert('Session completed successfully.', 'ok', true);
          this.router.navigate(['/report']);
        }
      } catch (e: any) {
        alert(`Stop session failed: ${e.message || e}`);
      } finally {
        this.state.ctlStopPending.set(false);
      }
    }
  }

  captureSnapshot() {
    const payload = this.state.lastPayload();
    if (!payload) return;

    this.state.triggerHaptic('success');
    const snapId = 'snap_' + Date.now();
    const snap = {
      id: snapId,
      ts: Date.now(),
      reported_hr: payload.radar?.reported_hr || 0,
      reported_rr: payload.radar?.reported_rr || 0,
      distance_cm: payload.radar?.distance_cm || 0,
      ble_hr: payload.ble?.hr || 0,
      ble_rr: payload.ble?.rr || 0
    };

    this.state.snaps.update(s => [...s, snap]);
    this.state.snapNotes.update(n => ({ ...n, [snapId]: '' }));
  }

  deleteSnap(snapId: string) {
    this.state.triggerHaptic('tap');
    this.state.snaps.update(s => s.filter(x => x.id !== snapId));
    this.state.snapNotes.update(n => {
      const next = { ...n };
      delete next[snapId];
      return next;
    });
  }

  clearAllSnaps() {
    this.state.triggerHaptic('destructiveAccept');
    if (confirm('Clear all snapshots pinned during this session?')) {
      this.state.snaps.set([]);
      this.state.snapNotes.set({});
    }
  }

  updateSnapNote(snapId: string, val: string) {
    this.state.snapNotes.update(n => ({ ...n, [snapId]: val }));
  }

  saveSessionNotes(val: string) {
    this.sessionNotesInput = val;
    const sid = this.state.currentSessionId();
    if (sid) {
      this.state.sessionNotes.update(n => ({ ...n, [sid]: val }));
    }
  }

  addQuickTag(tag: string) {
    this.state.triggerHaptic('tap');
    const stamp = new RegExp('\\b' + tag + '\\b', 'i');
    if (stamp.test(this.sessionNotesInput)) return;

    const next = this.sessionNotesInput ? `${this.sessionNotesInput}, ${tag}` : tag;
    this.saveSessionNotes(next);
  }

  // --- Real-time Waveforms & Trends Canvas plots ---
  private initCanvasDrawing() {
    const renderLoop = () => {
      this.drawWaves();
      this.drawTrends();
      this.drawOverviewSparklines();
      this.animeFrameId = requestAnimationFrame(renderLoop);
    };
    this.animeFrameId = requestAnimationFrame(renderLoop);
  }

  private drawWaves() {
    // Only render waveforms if active tab is Waves (index 1)
    if (this.activeTabIndex !== 1) return;

    const payload = this.state.lastPayload();
    if (!payload || !payload.series) return;

    const breathPoints = payload.series.breath || [];
    const heartPoints = payload.series.heart || [];

    const drawPhase = (canvasRef: ElementRef<HTMLCanvasElement>, points: number[], color: string) => {
      const canvas = canvasRef.nativeElement;
      const w = canvas.clientWidth;
      const h = canvas.clientHeight;
      const dpr = window.devicePixelRatio || 1;

      if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
        canvas.width = w * dpr;
        canvas.height = h * dpr;
      }
      const ctx = canvas.getContext('2d');
      if (!ctx) return;
      ctx.resetTransform();
      ctx.scale(dpr, dpr);
      ctx.clearRect(0, 0, w, h);

      if (points.length < 2) return;

      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.beginPath();

      const pad = 8;
      const innerW = w - pad * 2;
      const innerH = h - pad * 2;
      const count = points.length;

      points.forEach((val, idx) => {
        const x = pad + (idx / (count - 1)) * innerW;
        // phase signals normalized roughly between -1.5 and 1.5
        const y = pad + innerH / 2 - (val / 1.5) * (innerH / 2);
        
        if (idx === 0) {
          ctx.moveTo(x, y);
        } else {
          ctx.lineTo(x, y);
        }
      });
      ctx.stroke();
    };

    if (this.breathCanvas) {
      drawPhase(this.breathCanvas, breathPoints, 'rgba(255, 65, 248, 0.95)');
    }
    if (this.heartCanvas) {
      drawPhase(this.heartCanvas, heartPoints, 'rgba(0, 164, 150, 0.95)');
    }
  }

  private drawTrends() {
    // Render HR trend (index 2) or RR trend (index 3)
    const payload = this.state.lastPayload();
    if (!payload || !payload.series) return;

    const plotTrend = (canvasRef: ElementRef<HTMLCanvasElement>, data: number[], color: string, minV: number, maxV: number) => {
      const canvas = canvasRef.nativeElement;
      const w = canvas.clientWidth;
      const h = canvas.clientHeight;
      const dpr = window.devicePixelRatio || 1;

      if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
        canvas.width = w * dpr;
        canvas.height = h * dpr;
      }
      const ctx = canvas.getContext('2d');
      if (!ctx) return;
      ctx.resetTransform();
      ctx.scale(dpr, dpr);
      ctx.clearRect(0, 0, w, h);

      if (data.length < 2) return;

      const pad = 16;
      const innerW = w - pad * 2;
      const innerH = h - pad * 2;
      const count = data.length;

      // Draw gridlines
      ctx.strokeStyle = '#e2e8f0';
      ctx.lineWidth = 1;
      for (let i = 0; i <= 4; i++) {
        const y = pad + (innerH * i / 4);
        ctx.beginPath();
        ctx.moveTo(pad, y);
        ctx.lineTo(w - pad, y);
        ctx.stroke();
      }

      ctx.strokeStyle = color;
      ctx.lineWidth = 3;
      ctx.beginPath();

      const diff = Math.max(1, maxV - minV);

      data.forEach((val, idx) => {
        const x = pad + (idx / (count - 1)) * innerW;
        const y = pad + innerH - ((val - minV) / diff) * innerH;

        if (idx === 0) {
          ctx.moveTo(x, y);
        } else {
          ctx.lineTo(x, y);
        }
      });
      ctx.stroke();
    };

    if (this.activeTabIndex === 2 && this.hrTrendCanvas) {
      plotTrend(this.hrTrendCanvas, payload.series.hr || [], 'rgba(0, 164, 150, 0.95)', 40, 160);
    }
    if (this.activeTabIndex === 3 && this.rrTrendCanvas) {
      plotTrend(this.rrTrendCanvas, payload.series.rr || [], 'rgba(255, 65, 248, 0.95)', 5, 35);
    }
  }

  private drawOverviewSparklines() {
    // Only render mini sparklines if Overview tab is active (index 0)
    if (this.activeTabIndex !== 0) return;

    const spark = this.state.spark();
    
    const drawMiniSpark = (canvasRef: ElementRef<HTMLCanvasElement>, data: number[], color: string) => {
      const canvas = canvasRef.nativeElement;
      const w = canvas.clientWidth;
      const h = canvas.clientHeight;
      const dpr = window.devicePixelRatio || 1;

      if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
        canvas.width = w * dpr;
        canvas.height = h * dpr;
      }
      const ctx = canvas.getContext('2d');
      if (!ctx) return;
      ctx.resetTransform();
      ctx.scale(dpr, dpr);
      ctx.clearRect(0, 0, w, h);

      if (data.length < 2) return;

      const minV = Math.min(...data);
      const maxV = Math.max(...data);
      const diff = Math.max(1, maxV - minV);

      ctx.strokeStyle = color;
      ctx.lineWidth = 1.5;
      ctx.beginPath();

      const count = data.length;
      data.forEach((val, idx) => {
        const x = (idx / (count - 1)) * w;
        const y = h - 2 - ((val - minV) / diff) * (h - 4);
        
        if (idx === 0) {
          ctx.moveTo(x, y);
        } else {
          ctx.lineTo(x, y);
        }
      });
      ctx.stroke();
    };

    if (this.overviewHrSpark) drawMiniSpark(this.overviewHrSpark, spark.hr, 'rgba(0, 164, 150, 0.8)');
    if (this.overviewRrSpark) drawMiniSpark(this.overviewRrSpark, spark.rr, 'rgba(255, 65, 248, 0.8)');
    if (this.overviewFpsSpark) drawMiniSpark(this.overviewFpsSpark, spark.fps, 'rgba(100, 116, 139, 0.8)');
    if (this.overviewDistSpark) drawMiniSpark(this.overviewDistSpark, spark.dist, 'rgba(14, 165, 233, 0.8)');
  }
}
