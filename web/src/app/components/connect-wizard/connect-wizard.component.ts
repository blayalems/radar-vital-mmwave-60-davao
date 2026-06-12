import { ChangeDetectionStrategy, Component, ElementRef, ViewChild, signal, inject, OnDestroy } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { Router } from '@angular/router';
import { MatCardModule } from '@angular/material/card';
import { MatButtonModule } from '@angular/material/button';
import { MatIconModule } from '@angular/material/icon';
import { MatInputModule } from '@angular/material/input';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatProgressSpinnerModule } from '@angular/material/progress-spinner';
import { MatExpansionModule } from '@angular/material/expansion';

import { StateService } from '../../services/state.service';
import { ApiService } from '../../services/api.service';
import { ServerLifecycleService } from '../../services/server-lifecycle.service';

@Component({
  selector: 'app-connect-wizard',
  standalone: true,
  imports: [
    CommonModule,
    FormsModule,
    MatCardModule,
    MatButtonModule,
    MatIconModule,
    MatInputModule,
    MatFormFieldModule,
    MatProgressSpinnerModule,
    MatExpansionModule
  ],
  template: `
    <div class="connect-wizard-container" style="display: flex; justify-content: center; align-items: center; min-height: 100vh; padding: 16px; box-sizing: border-box; background: var(--md-sys-color-background, #fafafa);">
      <mat-card style="max-width: 480px; width: 100%;">
        <mat-card-header style="text-align: center; display: block; margin-bottom: 16px;">
          <mat-card-title style="font-size: 24px; font-weight: bold; margin-bottom: 8px;">Welcome to Radar Vital</mat-card-title>
          <mat-card-subtitle>Configure your connection to the 60 GHz mmWave trainer</mat-card-subtitle>
        </mat-card-header>

        <mat-card-content>
          @if (error()) {
            <div class="error-banner" style="background: rgba(239, 68, 68, 0.1); color: #ef4444; padding: 12px; border-radius: 8px; margin-bottom: 16px; font-size: 14px;">
              {{ error() }}
            </div>
          }

          @if (loading()) {
            <div style="display: flex; flex-direction: column; align-items: center; justify-content: center; padding: 32px 0;">
              <mat-spinner diameter="40"></mat-spinner>
              <p style="margin-top: 16px; color: var(--md-sys-color-on-surface-variant);">Connecting and pairing...</p>
            </div>
          } @else if (scanning()) {
            <div class="camera-preview-container" style="position: relative; width: 100%; border-radius: 8px; overflow: hidden; background: black; aspect-ratio: 4/3;">
              <video #videoElement style="width: 100%; height: 100%; object-fit: cover;" autoplay playsinline></video>
              <div style="position: absolute; inset: 0; border: 2px dashed rgba(255,255,255,0.6); margin: 40px; pointer-events: none;"></div>
              <button mat-flat-button color="warn" style="position: absolute; bottom: 16px; left: 50%; transform: translateX(-50%);" (click)="stopScanner()">
                Cancel Scan
              </button>
            </div>
          } @else {
            <div style="display: flex; flex-direction: column; gap: 16px;">
              <!-- Demo Mode Option -->
              <button mat-flat-button color="primary" style="height: 56px; font-size: 16px;" (click)="demoNow()">
                <mat-icon>science</mat-icon>
                Demo Now (Sandbox Mode)
              </button>

              <div style="display: flex; align-items: center; margin: 8px 0;">
                <hr style="flex-grow: 1; border: none; border-top: 1px solid var(--md-sys-color-outline-variant, #e0e0e0);">
                <span style="padding: 0 16px; color: var(--md-sys-color-on-surface-variant); font-size: 14px;">or connect to a trainer</span>
                <hr style="flex-grow: 1; border: none; border-top: 1px solid var(--md-sys-color-outline-variant, #e0e0e0);">
              </div>

              <!-- Scan QR Option -->
              <button mat-stroked-button style="height: 56px; font-size: 16px;" (click)="startScanner()">
                <mat-icon>qr_code_scanner</mat-icon>
                Scan Pairing QR Code
              </button>

              <!-- Manual Connect Form -->
              <mat-expansion-panel [expanded]="manualExpanded()" (opened)="manualExpanded.set(true)" (closed)="manualExpanded.set(false)" style="box-shadow: none; border: 1px solid var(--md-sys-color-outline-variant, #e0e0e0); border-radius: 8px;">
                <mat-expansion-panel-header>
                  <mat-panel-title>Enter Address & PIN Manually</mat-panel-title>
                </mat-expansion-panel-header>

                <div style="display: flex; flex-direction: column; gap: 16px; padding-top: 8px;">
                  <mat-form-field appearance="outline" style="width: 100%;">
                    <mat-label>Trainer Address</mat-label>
                    <input matInput [(ngModel)]="manualAddress" placeholder="e.g. http://192.168.1.100:8765">
                  </mat-form-field>

                  <mat-form-field appearance="outline" style="width: 100%;">
                    <mat-label>6-Digit Pairing PIN</mat-label>
                    <input matInput [(ngModel)]="manualPin" type="text" pattern="[0-9]*" inputmode="numeric" placeholder="e.g. 123456">
                  </mat-form-field>

                  <button mat-flat-button color="accent" style="height: 48px;" [disabled]="!manualAddress || !manualPin" (click)="connectManual()">
                    Connect & Pair
                  </button>
                </div>
              </mat-expansion-panel>
            </div>
          }
        </mat-card-content>
      </mat-card>
    </div>
  `,
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class ConnectWizardComponent implements OnDestroy {
  private readonly state = inject(StateService);
  private readonly api = inject(ApiService);
  private readonly serverLifecycle = inject(ServerLifecycleService);
  private readonly router = inject(Router);

  loading = signal(false);
  scanning = signal(false);
  error = signal<string | null>(null);
  manualExpanded = signal(false);

  manualAddress = '';
  manualPin = '';

  @ViewChild('videoElement') videoElement?: ElementRef<HTMLVideoElement>;
  private scannerStream?: MediaStream;
  private scanTimer?: number;

  ngOnDestroy() {
    this.stopScanner();
  }

  demoNow() {
    this.state.demoMode.set(true);
    try {
      localStorage.setItem('rvt-demo-mode', '1');
    } catch (_) {}
    this.serverLifecycle.setServerAddress('http://127.0.0.1:8765');
    this.state.triggerHaptic('tap');
    void this.router.navigate(['/live']);
  }

  async startScanner() {
    this.error.set(null);
    try {
      await this.loadJsQR();

      this.scannerStream = await navigator.mediaDevices.getUserMedia({
        video: { facingMode: 'environment' }
      });

      this.scanning.set(true);

      setTimeout(() => {
        if (this.videoElement && this.scannerStream) {
          this.videoElement.nativeElement.srcObject = this.scannerStream;
          this.startScanLoop();
        }
      }, 50);
    } catch (e: any) {
      this.error.set(e.message || 'Could not access camera.');
      this.stopScanner();
    }
  }

  stopScanner() {
    this.scanning.set(false);
    if (this.scanTimer) {
      cancelAnimationFrame(this.scanTimer);
      this.scanTimer = undefined;
    }
    if (this.scannerStream) {
      this.scannerStream.getTracks().forEach(track => track.stop());
      this.scannerStream = undefined;
    }
  }

  private startScanLoop() {
    const video = this.videoElement?.nativeElement;
    if (!video) return;

    const canvas = document.createElement('canvas');
    const ctx = canvas.getContext('2d');

    const scan = () => {
      if (!this.scanning()) return;

      if (video.readyState === video.HAVE_ENOUGH_DATA) {
        canvas.width = video.videoWidth;
        canvas.height = video.videoHeight;
        ctx?.drawImage(video, 0, 0, canvas.width, canvas.height);
        const imageData = ctx?.getImageData(0, 0, canvas.width, canvas.height);

        if (imageData) {
          const code = (window as any).jsQR(imageData.data, imageData.width, imageData.height, {
            inversionAttempts: 'dontInvert'
          });

          if (code && code.data) {
            this.handleScannedQR(code.data);
            return;
          }
        }
      }
      this.scanTimer = requestAnimationFrame(scan);
    };

    this.scanTimer = requestAnimationFrame(scan);
  }

  private handleScannedQR(qrText: string) {
    try {
      const url = new URL(qrText);
      const pin = url.searchParams.get('pair');
      if (pin && /^\d{6}$/.test(pin)) {
        const serverAddress = url.origin;
        this.stopScanner();
        void this.pairAndConnect(serverAddress, pin);
      } else {
        this.scanTimer = requestAnimationFrame(() => this.startScanLoop());
      }
    } catch (e) {
      this.scanTimer = requestAnimationFrame(() => this.startScanLoop());
    }
  }

  async connectManual() {
    if (!this.manualAddress || !this.manualPin) {
      this.error.set('Please fill in both fields.');
      return;
    }
    await this.pairAndConnect(this.manualAddress, this.manualPin);
  }

  async pairAndConnect(address: string, pin: string) {
    this.loading.set(true);
    this.error.set(null);
    try {
      this.state.demoMode.set(false);
      try {
        localStorage.removeItem('rvt-demo-mode');
      } catch (_) {}

      this.serverLifecycle.setServerAddress(address);
      await this.api.exchangePairPin(pin);

      this.state.triggerHaptic('success');
      await this.router.navigate(['/live']);
    } catch (e: any) {
      this.error.set(e.message || 'Pairing failed.');
      this.loading.set(false);
    }
  }

  private loadJsQR(): Promise<void> {
    return new Promise((resolve, reject) => {
      if ((window as any).jsQR) {
        resolve();
        return;
      }
      const script = document.createElement('script');
      script.src = './lib/jsqr.min.js';
      script.onload = () => {
        if ((window as any).jsQR) {
          resolve();
        } else {
          reject(new Error('jsQR loaded but not found on window object.'));
        }
      };
      script.onerror = () => reject(new Error('Failed to load jsQR library from assets.'));
      document.body.appendChild(script);
    });
  }
}
