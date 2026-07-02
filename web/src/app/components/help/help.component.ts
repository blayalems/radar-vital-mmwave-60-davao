import { ChangeDetectionStrategy, Component, computed, inject, OnInit, signal, ChangeDetectorRef } from '@angular/core';
import { ActivatedRoute, Router, RouterModule } from '@angular/router';
import { MatButtonModule } from '@angular/material/button';
import { MatCardModule } from '@angular/material/card';
import { MatCheckboxModule } from '@angular/material/checkbox';
import { MatChipsModule } from '@angular/material/chips';
import { MatExpansionModule } from '@angular/material/expansion';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatIconModule } from '@angular/material/icon';
import { MatInputModule } from '@angular/material/input';
import { MatListModule } from '@angular/material/list';
import { MatProgressBarModule } from '@angular/material/progress-bar';
import { MatSnackBar, MatSnackBarModule } from '@angular/material/snack-bar';
import { MatSlideToggleModule } from '@angular/material/slide-toggle';
import { MatDialog, MatDialogModule } from '@angular/material/dialog';

import { ApiService } from '../../services/api.service';
import { FirstRunService } from '../../services/first-run.service';
import { GITHUB_REPO_URL } from '../../services/app-meta';
import { StateService } from '../../services/state.service';
import { KeyboardShortcutsDialogComponent } from '../keyboard-shortcuts-dialog/keyboard-shortcuts-dialog.component';

type HelpCategory = 'workflow' | 'vitals' | 'quality' | 'truthfulness' | 'reference' | 'troubleshooting' | 'shortcuts';

interface HelpEntry {
  id: string;
  title: string;
  category: HelpCategory;
  summary: string;
  detail?: string;
  advanced?: string;
}

interface DspStep {
  n: number;
  title: string;
  summary: string;
  detail: string;
}

interface TroubleshootingEntry {
  id: string;
  title: string;
  steps: string[];
}

interface HelpSchema {
  version?: string;
  glossary?: Array<{ id?: string; term?: string; short?: string; long?: string; category?: string }>;
  faq?: Array<{ q?: string; a?: string }>;
  tooltips?: Record<string, string>;
  dsp_steps?: DspStep[];
  troubleshooting?: TroubleshootingEntry[];
}

interface RecoveryStep {
  id: string;
  label: string;
  detail: string;
}

@Component({
  selector: 'app-help',
  imports: [
    RouterModule,
    MatButtonModule,
    MatCardModule,
    MatCheckboxModule,
    MatChipsModule,
    MatExpansionModule,
    MatFormFieldModule,
    MatIconModule,
    MatInputModule,
    MatListModule,
    MatProgressBarModule,
    MatSlideToggleModule,
    MatSnackBarModule,
    MatDialogModule
  ],
  templateUrl: './help.component.html',
  styleUrl: './help.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class HelpComponent implements OnInit {
  protected readonly state = inject(StateService);
  protected readonly api = inject(ApiService);
  protected readonly firstRun = inject(FirstRunService);
  private readonly router = inject(Router);
  private readonly route = inject(ActivatedRoute);
  private readonly snackBar = inject(MatSnackBar);
  private readonly dialog = inject(MatDialog);
  private readonly cdr = inject(ChangeDetectorRef);

  protected readonly searchQuery = signal('');
  protected readonly activeCategory = signal<HelpCategory | 'all'>('all');
  protected readonly schemaVersion = signal('');
  protected readonly entries = signal<HelpEntry[]>(this.baseEntries());
  protected readonly dspSteps = signal<DspStep[]>(this.embeddedDspSteps());
  protected readonly troubleshooting = signal<TroubleshootingEntry[]>(this.embeddedTroubleshooting());
  protected readonly tooltips = signal<Array<{ key: string; value: string }>>(this.embeddedTooltips());
  protected readonly loading = signal(false);
  protected readonly allTopicsVisible = signal(false);
  protected readonly advancedMode = signal(this.storageValue('rvt-help-advanced') === '1');
  protected readonly selectedTopic = signal(this.storageValue('rvt-help-topic') || 'getting_started');
  protected readonly recoverySteps = this.embeddedRecoverySteps();
  protected readonly completedRecoverySteps = signal<Set<string>>(this.readRecoveryProgress());
  protected readonly termsUrl = `${GITHUB_REPO_URL}/blob/main/TERMS.md`;
  protected readonly privacyUrl = `${GITHUB_REPO_URL}/blob/main/PRIVACY.md`;
  protected readonly licenseUrl = `${GITHUB_REPO_URL}/blob/main/LICENSE`;
  protected readonly githubRepoUrl = GITHUB_REPO_URL;

  protected readonly topicLinks: Array<{ id: string; label: string; icon: string }> = [
    { id: 'getting_started', label: 'Getting started', icon: 'rocket_launch' },
    { id: 'hardware_setup', label: 'Hardware', icon: 'settings_input_antenna' },
    { id: 'troubleshooting', label: 'Recovery', icon: 'build' },
    { id: 'report_readiness', label: 'Reports', icon: 'assignment_turned_in' },
    { id: 'firmware_truthfulness', label: 'Firmware', icon: 'verified_user' }
  ];

  protected readonly categories: Array<{ id: HelpCategory | 'all'; label: string; icon: string }> = [
    { id: 'all', label: 'All', icon: 'menu_book' },
    { id: 'workflow', label: 'Workflow', icon: 'route' },
    { id: 'vitals', label: 'Vitals', icon: 'ecg_heart' },
    { id: 'quality', label: 'Quality', icon: 'verified' },
    { id: 'truthfulness', label: 'Truthfulness', icon: 'fact_check' },
    { id: 'reference', label: 'Reference', icon: 'bluetooth' },
    { id: 'troubleshooting', label: 'Recovery', icon: 'build' },
    { id: 'shortcuts', label: 'Shortcuts', icon: 'keyboard' }
  ];

  protected readonly filteredEntries = computed(() => {
    const category = this.activeCategory();
    const query = this.searchQuery().trim().toLowerCase();
    const matches = this.entries().filter(entry => {
      if (category !== 'all' && entry.category !== category) return false;
      return !query || `${entry.title} ${entry.summary} ${entry.detail || ''}`.toLowerCase().includes(query);
    });
    if (category === 'all' && !query && !this.allTopicsVisible()) {
      return matches.filter(entry => this.topicLinks.some(topic => topic.id === entry.id));
    }
    return matches;
  });

  protected readonly overviewMode = computed(() =>
    this.activeCategory() === 'all' && !this.searchQuery().trim() && !this.allTopicsVisible()
  );

  protected readonly additionalTopicCount = computed(() =>
    Math.max(0, this.entries().length - this.filteredEntries().length)
  );

  protected readonly filteredTroubleshooting = computed(() => {
    if (this.activeCategory() !== 'all' && this.activeCategory() !== 'troubleshooting' && !this.searchQuery().trim()) {
      return [];
    }
    const query = this.searchQuery().trim().toLowerCase();
    return this.troubleshooting().filter(issue =>
      !query || `${issue.title} ${issue.steps.join(' ')}`.toLowerCase().includes(query)
    );
  });

  protected readonly filteredTooltips = computed(() => {
    if (this.activeCategory() !== 'all' && !['reference', 'vitals', 'quality', 'truthfulness'].includes(this.activeCategory()) && !this.searchQuery().trim()) {
      return [];
    }
    const query = this.searchQuery().trim().toLowerCase();
    return this.tooltips().filter(tip =>
      !query || `${tip.key} ${tip.value}`.toLowerCase().includes(query)
    );
  });

  protected readonly filteredDspSteps = computed(() => {
    if (this.activeCategory() !== 'all' && !['workflow', 'quality', 'truthfulness'].includes(this.activeCategory()) && !this.searchQuery().trim()) {
      return [];
    }
    const query = this.searchQuery().trim().toLowerCase();
    return this.dspSteps().filter(step =>
      !query || `${step.title} ${step.summary} ${step.detail}`.toLowerCase().includes(query)
    );
  });

  protected readonly recoveryProgress = computed(() =>
    Math.round((this.completedRecoverySteps().size / this.recoverySteps.length) * 100)
  );

  protected readonly hasResults = computed(() =>
    this.filteredEntries().length > 0
    || this.filteredDspSteps().length > 0
    || this.filteredTroubleshooting().length > 0
    || this.filteredTooltips().length > 0
  );

  ngOnInit(): void {
    const requestedTopic = this.route.snapshot.queryParamMap.get('help') || this.route.snapshot.queryParamMap.get('helpTopic');
    if (requestedTopic && this.entries().some(entry => entry.id === requestedTopic)) {
      this.openTopic(requestedTopic, false);
    }
    void this.fetchHelpSchema();
  }

  protected updateSearch(event: Event): void {
    this.searchQuery.set((event.target as HTMLInputElement).value);
  }

  protected setCategory(category: HelpCategory | 'all'): void {
    this.activeCategory.set(category);
    this.allTopicsVisible.set(category !== 'all');
    this.state.triggerHaptic('tap');
  }

  protected revealAllTopics(): void {
    this.allTopicsVisible.set(true);
    this.state.triggerHaptic('tap');
  }

  protected openTopic(topicId: string, scroll = true): void {
    this.searchQuery.set('');
    this.activeCategory.set('all');
    this.allTopicsVisible.set(true);
    this.selectedTopic.set(topicId);
    this.writeStorage('rvt-help-topic', topicId);
    this.state.triggerHaptic('tap');
    this.cdr.detectChanges();
    if (scroll) {
      setTimeout(() => {
        document.querySelector(`[data-help-topic="${topicId}"]`)?.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
        this.cdr.detectChanges();
      });
    }
  }

  protected setAdvancedMode(enabled: boolean): void {
    this.advancedMode.set(enabled);
    this.writeStorage('rvt-help-advanced', enabled ? '1' : '0');
  }

  protected setRecoveryStep(id: string, checked: boolean): void {
    const selected = new Set(this.completedRecoverySteps());
    if (checked) selected.add(id); else selected.delete(id);
    this.completedRecoverySteps.set(selected);
    this.writeStorage('rvt-help-recovery-state', JSON.stringify([...selected]));
  }

  protected resetRecoveryChecklist(): void {
    this.completedRecoverySteps.set(new Set());
    this.writeStorage('rvt-help-recovery-state', '[]');
    this.snackBar.open('Recovery checklist reset.', 'Dismiss', { duration: 3000 });
  }

  protected async copySupportSummary(): Promise<void> {
    const status = this.state.ctlStatus()?.mode === 'sandbox' ? 'Demo source' : (this.state.ctlStatus()?.ok ? 'Trainer connected' : 'Trainer unavailable');
    const summary = [
      'Radar Vital support summary',
      `Source: ${status}`,
      `Topic: ${this.selectedTopic()}`,
      `Recovery checklist: ${this.completedRecoverySteps().size}/${this.recoverySteps.length}`,
      `Guide mode: ${this.advancedMode() ? 'Advanced' : 'Beginner'}`
    ].join('\n');
    try {
      await navigator.clipboard.writeText(summary);
      this.snackBar.open('Support summary copied.', 'Dismiss', { duration: 3000 });
    } catch (_) {
      this.snackBar.open('Clipboard unavailable. Review the support status on screen.', 'Dismiss', { duration: 4000 });
    }
  }

  protected async reconnect(): Promise<void> {
    const connected = await this.api.detectControlMode();
    this.snackBar.open(
      connected ? 'Trainer connection established.' : 'Trainer unavailable; review source settings.',
      'Dismiss',
      { duration: 5000 }
    );
  }

  protected openShortcuts(): void {
    this.state.triggerHaptic('tap');
    this.dialog.open(KeyboardShortcutsDialogComponent, {
      restoreFocus: true,
      panelClass: 'm3-dialog-panel'
    });
  }

  protected replayTutorial(): void {
    this.state.triggerHaptic('tap');
    this.firstRun.replayTutorial();
  }

  protected openPreflight(): void {
    void this.router.navigate(['/home']);
  }

  private async fetchHelpSchema(): Promise<void> {
    this.loading.set(true);
    try {
      const schema = await this.api.request<HelpSchema>('/api/help/schema');
      this.schemaVersion.set(schema.version || '');
      const glossary = (schema.glossary || []).map(item => ({
        id: String(item.id || item.term || 'glossary'),
        title: String(item.term || 'Telemetry field'),
        category: this.mapCategory(item.category),
        summary: String(item.short || ''),
        detail: String(item.long || ''),
        advanced: String(item.long || '')
      }));
      const faq = (schema.faq || []).map((item, index) => ({
        id: `faq-${index}`,
        title: String(item.q || 'Question'),
        category: 'workflow' as const,
        summary: String(item.a || '')
      }));
      this.entries.set(this.mergeById(this.baseEntries(), [...faq, ...glossary]));
      this.dspSteps.set(this.mergeByKey(this.embeddedDspSteps(), schema.dsp_steps || [], step => String(step.n)));
      this.troubleshooting.set(this.mergeById(this.embeddedTroubleshooting(), schema.troubleshooting || []));
      this.tooltips.set(this.mergeByKey(
        this.embeddedTooltips(),
        Object.entries(schema.tooltips || {}).map(([key, value]) => ({ key, value })),
        tip => tip.key
      ));
      this.cdr.detectChanges();
    } catch (_) {
      this.snackBar.open('Trainer help schema is unavailable; showing embedded guidance.', 'Dismiss', { duration: 5000 });
      this.cdr.detectChanges();
    } finally {
      this.loading.set(false);
      this.cdr.detectChanges();
    }
  }

  private mapCategory(category?: string): HelpCategory {
    const known = ['vitals', 'quality', 'truthfulness', 'reference'] as HelpCategory[];
    return known.includes(category as HelpCategory) ? category as HelpCategory : 'workflow';
  }

  private baseEntries(): HelpEntry[] {
    return [
      {
        id: 'getting_started',
        title: 'Getting started',
        category: 'workflow',
        summary: 'Choose the subject profile, confirm source hardware, run preflight and only then start a recording.',
        detail: 'Start remains a controlled workflow: select duration and labels in Home, confirm radar and BLE status, then monitor Live for stable signal quality before trusting a report.',
        advanced: 'The trainer records raw and published measurements separately. Published HR/RR can be intentionally blank when freshness, gate or contract checks fail.'
      },
      {
        id: 'hardware_setup',
        title: 'Hardware setup',
        category: 'reference',
        summary: 'Place the radar toward the upper torso at roughly 40 to 80 cm and pair the BLE oximeter before collection.',
        detail: 'Select the detected COM port, verify BLE address/profile, clear moving reflectors and ask the participant to remain still during warmup.',
        advanced: 'Use radar placement, BLE coverage and freshness diagnostics together. A connected device alone does not establish a usable reference trace.'
      },
      {
        id: 'troubleshooting',
        title: 'Troubleshooting and recovery',
        category: 'troubleshooting',
        summary: 'Resolve missing ports, unreachable BLE, stale data and firmware mismatches before using recorded values.',
        detail: 'Use the interactive recovery checklist and the fault-specific checklists below. Re-run preflight after correcting a source issue.',
        advanced: 'Do not bypass a firmware contract mismatch or fabricate missing reference coverage; those conditions invalidate readiness conclusions.'
      },
      {
        id: 'report_readiness',
        title: 'Report readiness',
        category: 'truthfulness',
        summary: 'A finished session is not automatically ready for analysis or ML use.',
        detail: 'Review signal quality, reference coverage, gate reasons, validity rates and truthfulness checks in Report and Audit before export.',
        advanced: 'Readiness must preserve conditional and rejected outcomes. Short or low-coverage sessions remain evidence, but not training-ready samples.'
      },
      {
        id: 'firmware_truthfulness',
        title: 'Firmware and schema truthfulness',
        category: 'truthfulness',
        summary: 'Firmware, trainer and dashboard contracts must agree before interpreting measurements.',
        detail: 'Use the expected v16 dashboard workflow with the matching trainer and firmware line; a mismatched CSV/schema is a blocking condition.',
        advanced: 'The serial DATA header is a fixed contract. Do not reinterpret unknown columns or treat mismatched firmware as comparable evidence.'
      },
      {
        id: 'demo',
        title: 'Demo mode and evidence boundaries',
        category: 'truthfulness',
        summary: 'Demo mode renders simulated telemetry only and always displays a DEMO banner.',
        detail: 'Use it to exercise UI behavior, not to produce a physiological readiness verdict.'
      },
      {
        id: 'palette',
        title: 'Keyboard and command palette',
        category: 'shortcuts',
        summary: 'Press Ctrl+K to search navigation, capture, alerts, exports, source and appearance controls.',
        detail: 'Alt+1 through Alt+6 open Overview, Waves, HR, RR, Snaps and Audit tabs. Escape dismisses dialogs.'
      }
    ];
  }

  private embeddedDspSteps(): DspStep[] {
    return [
      { n: 1, title: 'Radar frames', summary: 'Acquire range and phase telemetry.', detail: 'Firmware emits frames into the fixed session CSV contract.' },
      { n: 2, title: 'Phase stabilization', summary: 'Control slow drift and missed-frame fill.', detail: 'Stabilized phase is retained alongside explicit skip/fill audit state.' },
      { n: 3, title: 'Target tracking', summary: 'Choose the primary torso reflector.', detail: 'Range and Doppler diagnostics help detect placement or motion errors.' },
      { n: 4, title: 'Vital candidates', summary: 'Derive HR and RR candidates.', detail: 'Candidate, raw, reference and published traces remain distinguishable.' },
      { n: 5, title: 'Quality gates', summary: 'Apply PQI, freshness and policy checks.', detail: 'Rejected values remain visible through gate reasons rather than appearing live.' },
      { n: 6, title: 'Session report', summary: 'Align the reference and score evidence.', detail: 'Reports preserve readiness, truthfulness and remediation conclusions.' }
    ];
  }

  private embeddedTroubleshooting(): TroubleshootingEntry[] {
    return [
      { id: 'port', title: 'Radar port not found', steps: ['Check USB power and cable.', 'Confirm the assigned COM port in Device Manager.', 'Select the detected port and re-run preflight.'] },
      { id: 'ble', title: 'BLE device not responding', steps: ['Wake the oximeter and enable Bluetooth.', 'Confirm its address/profile in Setup.', 'Retry the BLE probe before recording.'] },
      { id: 'stale', title: 'Telemetry is stale', steps: ['Confirm the trainer session is running.', 'Check serial input and placement stability.', 'Do not treat stale KPIs as current readings.'] },
      { id: 'contract', title: 'Firmware contract mismatch', steps: ['Stop the session.', 'Flash the matching firmware and verify the header.', 'Do not export the mismatched session for ML use.'] }
    ];
  }

  private embeddedTooltips(): Array<{ key: string; value: string }> {
    return [
      { key: 'HR publish', value: 'Heart rate shown only after validity and freshness gates pass.' },
      { key: 'RR publish', value: 'Respiration rate shown only after source and policy gates pass.' },
      { key: 'PQI', value: 'Phase quality indicator used to assess signal usability.' },
      { key: 'BLE coverage', value: 'Share of the session with usable reference readings.' },
      { key: 'RMSE', value: 'Error measure that penalizes large radar-to-reference misses.' },
      { key: 'Truthfulness', value: 'Contract and provenance checks that protect interpretation.' }
    ];
  }

  private embeddedRecoverySteps(): RecoveryStep[] {
    return [
      { id: 'source', label: 'Confirm source mode', detail: 'Verify live trainer or explicitly labeled demo mode.' },
      { id: 'hardware', label: 'Check sensor links', detail: 'Confirm radar port, BLE power and address.' },
      { id: 'preflight', label: 'Run preflight', detail: 'Resolve failed checks before recording.' },
      { id: 'quality', label: 'Inspect quality and freshness', detail: 'Review PQI, movement and stale flags in Live.' },
      { id: 'report', label: 'Review readiness before export', detail: 'Do not train from blocked or mismatched data.' }
    ];
  }

  private mergeById<T extends { id: string }>(base: T[], additions: T[]): T[] {
    return this.mergeByKey(base, additions, item => item.id);
  }

  private mergeByKey<T>(base: T[], additions: T[], key: (item: T) => string): T[] {
    const merged = new Map(base.map(item => [key(item), item]));
    additions.forEach(item => merged.set(key(item), item));
    return [...merged.values()];
  }

  private storageValue(key: string): string {
    try { return localStorage.getItem(key) || ''; } catch (_) { return ''; }
  }

  private writeStorage(key: string, value: string): void {
    try { localStorage.setItem(key, value); } catch (_) { /* storage is optional in embedded views */ }
  }

  private readRecoveryProgress(): Set<string> {
    try {
      const saved = JSON.parse(this.storageValue('rvt-help-recovery-state') || '[]') as unknown;
      return new Set(Array.isArray(saved) ? saved.filter(item => typeof item === 'string') : []);
    } catch (_) {
      return new Set();
    }
  }
}
