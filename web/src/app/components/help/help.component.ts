import { ChangeDetectionStrategy, Component, computed, inject, OnInit, signal } from '@angular/core';
import { Router, RouterModule } from '@angular/router';
import { MatButtonModule } from '@angular/material/button';
import { MatCardModule } from '@angular/material/card';
import { MatChipsModule } from '@angular/material/chips';
import { MatExpansionModule } from '@angular/material/expansion';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatIconModule } from '@angular/material/icon';
import { MatInputModule } from '@angular/material/input';
import { MatListModule } from '@angular/material/list';
import { MatSnackBar, MatSnackBarModule } from '@angular/material/snack-bar';

import { ApiService } from '../../services/api.service';
import { StateService } from '../../services/state.service';

type HelpCategory = 'workflow' | 'vitals' | 'quality' | 'truthfulness' | 'reference' | 'troubleshooting' | 'shortcuts';

interface HelpEntry {
  id: string;
  title: string;
  category: HelpCategory;
  summary: string;
  detail?: string;
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

@Component({
  selector: 'app-help',
  imports: [
    RouterModule,
    MatButtonModule,
    MatCardModule,
    MatChipsModule,
    MatExpansionModule,
    MatFormFieldModule,
    MatIconModule,
    MatInputModule,
    MatListModule,
    MatSnackBarModule
  ],
  templateUrl: './help.component.html',
  styleUrl: './help.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class HelpComponent implements OnInit {
  protected readonly state = inject(StateService);
  protected readonly api = inject(ApiService);
  private readonly router = inject(Router);
  private readonly snackBar = inject(MatSnackBar);

  protected readonly searchQuery = signal('');
  protected readonly activeCategory = signal<HelpCategory | 'all'>('all');
  protected readonly schemaVersion = signal('');
  protected readonly entries = signal<HelpEntry[]>(this.baseEntries());
  protected readonly dspSteps = signal<DspStep[]>([]);
  protected readonly troubleshooting = signal<TroubleshootingEntry[]>([]);
  protected readonly tooltips = signal<Array<{ key: string; value: string }>>([]);
  protected readonly loading = signal(false);
  protected readonly allTopicsVisible = signal(false);

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
      return matches.filter(entry => ['setup', 'demo', 'palette'].includes(entry.id));
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
    const query = this.searchQuery().trim().toLowerCase();
    return this.troubleshooting().filter(issue =>
      !query || `${issue.title} ${issue.steps.join(' ')}`.toLowerCase().includes(query)
    );
  });

  protected readonly filteredTooltips = computed(() => {
    const query = this.searchQuery().trim().toLowerCase();
    return this.tooltips().filter(tip =>
      !query || `${tip.key} ${tip.value}`.toLowerCase().includes(query)
    );
  });

  ngOnInit(): void {
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

  protected async reconnect(): Promise<void> {
    const connected = await this.api.detectControlMode();
    this.snackBar.open(
      connected ? 'Trainer connection established.' : 'Trainer unavailable; review source settings.',
      'Dismiss',
      { duration: 5000 }
    );
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
        detail: String(item.long || '')
      }));
      const faq = (schema.faq || []).map((item, index) => ({
        id: `faq-${index}`,
        title: String(item.q || 'Question'),
        category: 'workflow' as const,
        summary: String(item.a || '')
      }));
      this.entries.set([...this.baseEntries(), ...faq, ...glossary]);
      this.dspSteps.set(schema.dsp_steps || []);
      this.troubleshooting.set(schema.troubleshooting || []);
      this.tooltips.set(Object.entries(schema.tooltips || {}).map(([key, value]) => ({ key, value })));
    } catch (_) {
      this.snackBar.open('Trainer help schema is unavailable; showing embedded guidance.', 'Dismiss', { duration: 5000 });
    } finally {
      this.loading.set(false);
    }
  }

  private mapCategory(category?: string): HelpCategory {
    const known = ['vitals', 'quality', 'truthfulness', 'reference'] as HelpCategory[];
    return known.includes(category as HelpCategory) ? category as HelpCategory : 'workflow';
  }

  private baseEntries(): HelpEntry[] {
    return [
      {
        id: 'setup',
        title: 'Recording workflow',
        category: 'workflow',
        summary: 'Use Setup to select hardware and run preflight before Start is enabled.',
        detail: 'Do not interpret demo data as a recorded subject session. Connected trainer mode performs structural preflight before capture.'
      },
      {
        id: 'demo',
        title: 'Demo mode',
        category: 'truthfulness',
        summary: 'Demo mode renders simulated telemetry only and always displays a banner.',
        detail: 'Reports exported from demo mode carry no physiological readiness verdict.'
      },
      {
        id: 'palette',
        title: 'Command palette',
        category: 'shortcuts',
        summary: 'Press Ctrl+K to search navigation, capture, alerts, exports, source and appearance controls.',
        detail: 'Alt+1 through Alt+6 open Overview, Waves, HR, RR, Snapshots and Audit tabs.'
      }
    ];
  }
}
