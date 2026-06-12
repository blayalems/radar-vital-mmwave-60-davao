import {
  ChangeDetectionStrategy,
  Component,
  computed,
  inject,
  OnInit,
  signal
} from '@angular/core';
import { MatButtonModule } from '@angular/material/button';
import { MatDialogModule, MatDialogRef } from '@angular/material/dialog';
import { MatIconModule } from '@angular/material/icon';
import { FirstRunService } from '../../services/first-run.service';
import { ServerLifecycleService } from '../../services/server-lifecycle.service';

export type TutorialPlatform = 'exe' | 'native' | 'pwa';

export interface TutorialStep {
  icon: string;
  title: string;
  body: string;
}

/** Body text per platform for the "Connect" step. */
const CONNECT_BODY: Record<TutorialPlatform, string> = {
  exe:
    'The bundled trainer starts automatically in the background. Open <strong>Settings</strong> ' +
    'to check the trainer status or switch to LAN mode to share with phones via QR code.',
  native:
    'Pair your mobile device to a running trainer using the QR code or 6-digit PIN displayed ' +
    "in Settings. Tap <strong>Pair with PIN</strong>, enter the code, and you're connected.",
  pwa:
    'Enter your trainer\'s network address in <strong>Settings → Trainer address</strong>. ' +
    'If you don\'t have a trainer running, enable <strong>Demo mode</strong> (press D) to ' +
    'explore the interface with simulated data.'
};

function buildSteps(platform: TutorialPlatform): TutorialStep[] {
  return [
    {
      icon: 'waving_hand',
      title: 'Welcome to Radar Vital',
      body:
        'This guided tour takes about 2 minutes. It covers connecting your sensor, ' +
        'running a session, reading the Live view, reviewing reports, and getting help. ' +
        'You can replay this tour any time from the Help page.'
    },
    {
      icon: 'sensors',
      title: 'Connect Your Trainer',
      body: CONNECT_BODY[platform]
    },
    {
      icon: 'home',
      title: 'Home — Preflight &amp; Placement',
      body:
        'Before every session, open <strong>Home</strong> and run the hardware preflight ' +
        'checklist. Place the mmWave sensor so the subject falls within the highlighted ' +
        '<strong>Optimal zone</strong> (0.3 – 1.5 m) shown in the Radar Scope card. ' +
        'A green <em>Optimal</em> chip confirms correct placement.'
    },
    {
      icon: 'monitor_heart',
      title: 'Live View — Monitoring',
      body:
        'The <strong>Live</strong> tab streams real-time heart rate and respiration. ' +
        'Watch the <strong>lock-state chip</strong> (unlocked → locking → locked) in the ' +
        'KPI cards for measurement confidence. The <strong>SQI ribbon</strong> shows ' +
        'signal-quality index over time. Use <strong>P</strong> to pause, ' +
        '<strong>Alt+5</strong> to save a snapshot.'
    },
    {
      icon: 'assignment',
      title: 'Report — Quality &amp; Sign-off',
      body:
        'After a session, open <strong>Report</strong> to review the quality scorecard: ' +
        'PQI lock %, session quality score, and per-metric gates. Add operator notes, ' +
        'choose a comparison session to overlay a delta table, then record your ' +
        '<strong>sign-off</strong> with name, initials, and validation comments.'
    },
    {
      icon: 'help',
      title: 'Help &amp; Shortcuts',
      body:
        'Press <kbd>?</kbd> at any time to see the full keyboard shortcut reference. ' +
        'Press <kbd>Ctrl+K</kbd> (or <kbd>/</kbd>) to open the Command Palette and search ' +
        'any feature. The <strong>Help</strong> page contains the full Operator Playbook, ' +
        'field dictionary, and a recovery checklist. You can replay this tutorial any time ' +
        'from Help → <em>Replay onboarding tutorial</em>.'
    }
  ];
}

@Component({
  selector: 'app-onboarding-tutorial',
  imports: [MatButtonModule, MatDialogModule, MatIconModule],
  template: `
    <h2 mat-dialog-title class="tutorial-title">
      <mat-icon aria-hidden="true">{{ currentStep().icon }}</mat-icon>
      <span [innerHTML]="currentStep().title"></span>
    </h2>

    <mat-dialog-content class="tutorial-content">
      <p class="tutorial-body" [innerHTML]="currentStep().body"></p>
    </mat-dialog-content>

    <!-- Progress dots -->
    <div class="tutorial-progress" role="tablist" aria-label="Tutorial progress">
      @for (step of steps(); track $index) {
        <button
          role="tab"
          class="progress-dot"
          [class.active]="$index === currentIndex()"
          [attr.aria-selected]="$index === currentIndex()"
          [attr.aria-label]="'Step ' + ($index + 1) + ' of ' + steps().length + ': ' + step.title"
          (click)="goToStep($index)"
          type="button"
        ></button>
      }
    </div>

    <mat-dialog-actions align="end" class="tutorial-actions">
      <button mat-button type="button" (click)="skip()" class="tutorial-skip-btn">
        Skip
      </button>
      @if (currentIndex() > 0) {
        <button mat-button type="button" (click)="back()" aria-label="Previous step">
          Back
        </button>
      }
      @if (isLastStep()) {
        <button mat-flat-button type="button" (click)="done()" cdkFocusInitial>
          Done
        </button>
      } @else {
        <button mat-flat-button type="button" (click)="next()" cdkFocusInitial>
          Next
        </button>
      }
    </mat-dialog-actions>
  `,
  styleUrl: './onboarding-tutorial.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush,
  host: {
    '(keydown)': 'onKeydown($event)'
  }
})
export class OnboardingTutorialComponent implements OnInit {
  private readonly dialogRef = inject(MatDialogRef<OnboardingTutorialComponent>);
  private readonly firstRun = inject(FirstRunService);
  private readonly serverLifecycle = inject(ServerLifecycleService);

  readonly currentIndex = signal(0);

  /** Resolved once on init and exposed as a signal to avoid late mutations. */
  readonly steps = signal<TutorialStep[]>([]);

  ngOnInit(): void {
    const slPlatform = this.serverLifecycle.platform();
    const isNative = (window as any)?.Capacitor?.isNativePlatform?.() === true;
    let platform: TutorialPlatform = 'pwa';
    if (isNative) {
      platform = 'native';
    } else if (slPlatform === 'exe') {
      platform = 'exe';
    }
    this.steps.set(buildSteps(platform));
  }

  readonly currentStep = computed<TutorialStep>(
    () => this.steps()[this.currentIndex()] ?? { icon: 'waving_hand', title: '', body: '' }
  );

  readonly isLastStep = computed(
    () => this.currentIndex() === this.steps().length - 1
  );

  next(): void {
    const max = this.steps().length - 1;
    if (this.currentIndex() < max) {
      this.currentIndex.update(i => i + 1);
    }
  }

  back(): void {
    if (this.currentIndex() > 0) {
      this.currentIndex.update(i => i - 1);
    }
  }

  goToStep(index: number): void {
    const max = this.steps().length - 1;
    if (index >= 0 && index <= max) {
      this.currentIndex.set(index);
    }
  }

  skip(): void {
    this.firstRun.markTutorialDone();
    this.dialogRef.close('skipped');
  }

  done(): void {
    this.firstRun.markTutorialDone();
    this.dialogRef.close('done');
  }

  onKeydown(event: KeyboardEvent): void {
    if (event.key === 'ArrowRight' || event.key === 'ArrowDown') {
      event.preventDefault();
      if (this.isLastStep()) {
        this.done();
      } else {
        this.next();
      }
    } else if (event.key === 'ArrowLeft' || event.key === 'ArrowUp') {
      event.preventDefault();
      this.back();
    }
  }
}
