import { Component, inject, OnInit, ViewChild, ElementRef, AfterViewInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';

// Angular Material 3 modules
import { MatCardModule } from '@angular/material/card';
import { MatButtonModule } from '@angular/material/button';
import { MatIconModule } from '@angular/material/icon';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatInputModule } from '@angular/material/input';
import { MatSelectModule } from '@angular/material/select';

import { StateService } from '../../services/state.service';
import { ApiService } from '../../services/api.service';

@Component({
  selector: 'app-report',
  standalone: true,
  imports: [
    CommonModule,
    FormsModule,
    MatCardModule,
    MatButtonModule,
    MatIconModule,
    MatFormFieldModule,
    MatInputModule,
    MatSelectModule
  ],
  templateUrl: './report.component.html',
  styleUrl: './report.component.css'
})
export class ReportComponent implements OnInit, AfterViewInit {
  protected readonly state = inject(StateService);
  protected readonly api = inject(ApiService);

  @ViewChild('hrReportCanvas', { static: false }) hrReportCanvas!: ElementRef<HTMLCanvasElement>;
  @ViewChild('rrReportCanvas', { static: false }) rrReportCanvas!: ElementRef<HTMLCanvasElement>;

  sessions: any[] = [];
  selectedSessionId = '';
  selectedSession: any = null;
  sessionNotesInput = '';

  ngOnInit() {
    this.loadSessions();
  }

  ngAfterViewInit() {
    this.drawReportTrends();
  }

  async loadSessions() {
    try {
      const resp = await this.api.request('/api/sessions');
      if (resp && Array.isArray(resp.items)) {
        this.sessions = resp.items;
        
        // Auto select current session if available, else first past run
        const activeId = this.state.currentSessionId();
        if (activeId && this.sessions.some(s => s.session_id === activeId)) {
          this.selectedSessionId = activeId;
        } else if (this.sessions.length > 0) {
          this.selectedSessionId = this.sessions[0].session_id;
        }
        this.onSessionChange();
      }
    } catch (e) {
      console.warn('Could not load sessions', e);
    }
  }

  onSessionChange() {
    this.selectedSession = this.sessions.find(s => s.session_id === this.selectedSessionId) || null;
    if (this.selectedSession) {
      this.sessionNotesInput = this.state.sessionNotes()[this.selectedSessionId] || this.selectedSession.summary || '';
      setTimeout(() => this.drawReportTrends(), 50);
    }
  }

  saveReportNotes() {
    if (this.selectedSessionId) {
      this.state.sessionNotes.update(notes => ({
        ...notes,
        [this.selectedSessionId]: this.sessionNotesInput
      }));
      this.state.triggerHaptic('success');
    }
  }

  downloadCsv() {
    this.state.triggerHaptic('tap');
    if (!this.selectedSession) return;

    // Simulate CSV generation for download
    const headers = 'Timestamp,Reported_HR,Reported_RR,Reference_HR,Reference_RR,Distance_cm\n';
    const rows = [
      `14:02:00,72,15,71,15,50\n`,
      `14:02:01,73,15,72,15,51\n`,
      `14:02:02,72,16,72,16,50\n`,
      `14:02:03,74,15,73,15,52\n`
    ].join('');
    
    const blob = new Blob([headers + rows], { type: 'text/csv' });
    const url = window.URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.setAttribute('href', url);
    a.setAttribute('download', `session_export_${this.selectedSessionId}.csv`);
    a.click();
  }

  // --- Draw simulated trend lines based on historical reports ---
  private drawReportTrends() {
    if (!this.hrReportCanvas || !this.rrReportCanvas) return;

    const plotReportTrend = (canvasRef: ElementRef<HTMLCanvasElement>, color: string, baseVal: number, rangeVal: number) => {
      const canvas = canvasRef.nativeElement;
      const w = canvas.clientWidth;
      const h = canvas.clientHeight;
      const dpr = window.devicePixelRatio || 1;

      canvas.width = w * dpr;
      canvas.height = h * dpr;
      const ctx = canvas.getContext('2d');
      if (!ctx) return;
      ctx.resetTransform();
      ctx.scale(dpr, dpr);
      ctx.clearRect(0, 0, w, h);

      // Generate stable simulated trend points based on session_id hash
      const seed = this.selectedSessionId.split('').reduce((acc, char) => acc + char.charCodeAt(0), 0);
      const points: number[] = [];
      for (let i = 0; i < 40; i++) {
        points.push(baseVal + Math.sin(i / 3 + seed) * (rangeVal * 0.3) + Math.cos(i / 5) * (rangeVal * 0.1));
      }

      const pad = 12;
      const innerW = w - pad * 2;
      const innerH = h - pad * 2;
      const count = points.length;

      // Draw gridlines
      ctx.strokeStyle = '#f1f5f9';
      ctx.lineWidth = 1;
      for (let i = 0; i <= 3; i++) {
        const y = pad + (innerH * i / 3);
        ctx.beginPath();
        ctx.moveTo(pad, y);
        ctx.lineTo(w - pad, y);
        ctx.stroke();
      }

      ctx.strokeStyle = color;
      ctx.lineWidth = 2.5;
      ctx.beginPath();

      const minV = Math.min(...points) - 5;
      const maxV = Math.max(...points) + 5;
      const diff = maxV - minV;

      points.forEach((val, idx) => {
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

    plotReportTrend(this.hrReportCanvas, 'rgba(0, 164, 150, 0.95)', 70, 30);
    plotReportTrend(this.rrReportCanvas, 'rgba(255, 65, 248, 0.95)', 15, 8);
  }
}
