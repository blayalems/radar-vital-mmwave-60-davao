import { ChangeDetectionStrategy, Component, DestroyRef, inject, OnInit, signal, ViewChild, ElementRef, HostListener, effect, computed } from '@angular/core';
import { Router, NavigationEnd, RouterModule } from '@angular/router';
import { BreakpointObserver } from '@angular/cdk/layout';
import { filter, map } from 'rxjs/operators';
import { takeUntilDestroyed, toSignal } from '@angular/core/rxjs-interop';
import { MatButtonModule } from '@angular/material/button';
import { MatDialog, MatDialogModule } from '@angular/material/dialog';
import { MatIconModule } from '@angular/material/icon';
import { MatListModule } from '@angular/material/list';
import { MatSidenavContainer, MatSidenavModule } from '@angular/material/sidenav';
import { MatToolbarModule } from '@angular/material/toolbar';
import { MatProgressSpinnerModule } from '@angular/material/progress-spinner';
import { MatSnackBar, MatSnackBarModule } from '@angular/material/snack-bar';
import { MatProgressBarModule } from '@angular/material/progress-bar';
import { MatNavigationBar, MatNavigationTab } from '@app/navigation-bar';
import { MatChipsModule } from '@angular/material/chips';

import { StateService } from '../../services/state.service';
import { ApiService } from '../../services/api.service';
import { TopbarComponent } from '../topbar/topbar.component';
import { CommandPaletteComponent } from '../command-palette/command-palette.component';
import { KeyboardShortcutsDialogComponent } from '../keyboard-shortcuts-dialog/keyboard-shortcuts-dialog.component';
import { AlertsDialogComponent } from '../alerts-dialog/alerts-dialog.component';
import { CommandPinningService } from '../../services/command-pinning.service';
import { IdleLockOverlayComponent } from '../idle-lock-overlay/idle-lock-overlay.component';

@Component({
  selector: 'app-layout',
  imports: [
    RouterModule,
    IdleLockOverlayComponent,
    TopbarComponent,
    MatButtonModule,
    MatDialogModule,
    MatIconModule,
    MatListModule,
    MatSidenavModule,
    MatToolbarModule,
    MatProgressSpinnerModule,
    MatSnackBarModule,
    MatChipsModule,
    MatNavigationBar,
    MatNavigationTab,
    MatProgressBarModule
  ],
  templateUrl: './layout.component.html',
  styleUrl: './layout.component.css',
  changeDetection: ChangeDetectionStrategy.OnPush,
  host: {
    '(document:keydown)': 'onKeyboardShortcut($event)'
  }
})
export class LayoutComponent implements OnInit {
  protected readonly state = inject(StateService);
  protected readonly api = inject(ApiService);
  private readonly router = inject(Router);
  private readonly dialog = inject(MatDialog);
  private readonly snackBar = inject(MatSnackBar);
  private readonly destroyRef = inject(DestroyRef);
  private readonly breakpointObserver = inject(BreakpointObserver);
  protected readonly pinning = inject(CommandPinningService);

  @ViewChild('mainContentScroll', { static: false }) mainContentScroll?: ElementRef<HTMLElement>;
  @ViewChild(MatSidenavContainer) sidenavContainer?: MatSidenavContainer;
  protected readonly showScrollTop = signal(false);

  protected readonly railCollapsed = signal(false);

  // Map BreakpointObserver outputs to M3 WindowSizeClasses (A10)
  protected readonly windowSizeClass = toSignal(
    this.breakpointObserver.observe([
      '(max-width: 599.98px)',
      '(min-width: 600px) and (max-width: 839.98px)',
      '(min-width: 840px)'
    ]).pipe(
      map(result => {
        if (result.breakpoints['(max-width: 599.98px)']) return 'Compact';
        if (result.breakpoints['(min-width: 600px) and (max-width: 839.98px)']) return 'Medium';
        return 'Expanded';
      })
    ),
    { initialValue: 'Expanded' }
  );

  protected readonly showRail = toSignal(
    this.breakpointObserver.observe('(min-width: 600px)').pipe(
      map(r => r.matches)
    ),
    { initialValue: true }
  );

  readonly isStaleBannerVisible = signal(false);
  private staleTimer: ReturnType<typeof setTimeout> | null = null;

  readonly sessionProgress = computed(() => {
    const elapsed = this.state.lastPayload()?.meta?.elapsed_s ?? 0;
    const duration = this.state.setup().duration_s || 60;
    return Math.min(100, Math.max(0, (elapsed / duration) * 100));
  });

  constructor() {
    this.destroyRef.onDestroy(() => {
      if (this.staleTimer) {
        clearTimeout(this.staleTimer);
        this.staleTimer = null;
      }
    });

    effect(() => {
      const active = this.state.sessionActive();
      const payload = this.state.lastPayload();

      if (this.staleTimer) {
        clearTimeout(this.staleTimer);
        this.staleTimer = null;
      }

      if (!active) {
        this.isStaleBannerVisible.set(false);
        return;
      }

      this.isStaleBannerVisible.set(false);
      this.staleTimer = setTimeout(() => {
        this.isStaleBannerVisible.set(true);
      }, 30000);
    });
  }

  ngOnInit() {
    this.setRailCollapsed(localStorage.getItem('rvt-rail-collapsed') === '1');
    this.syncCurrentView(this.router.url);
    this.router.events.pipe(
      filter((event): event is NavigationEnd => event instanceof NavigationEnd),
      takeUntilDestroyed(this.destroyRef)
    ).subscribe(event => {
      this.syncCurrentView(event.urlAfterRedirects);
    });

    // Wire up Capacitor App backButton listener to Router navigation for back gesture (A5)
    try {
      const cap = (window as any).Capacitor;
      if (cap?.isNativePlatform?.()) {
        const appPlugin = cap?.Plugins?.App;
        if (appPlugin) {
          appPlugin.addListener('backButton', ({ canGoBack }: { canGoBack: boolean }) => {
            if (canGoBack) {
              window.history.back();
            } else {
              appPlugin.exitApp?.();
            }
          });
        }
      }
    } catch (_) {}
  }

  toggleRailCollapse() {
    this.setRailCollapsed(!this.railCollapsed());
    this.state.triggerHaptic('tap');
  }

  openPalette() {
    this.state.triggerHaptic('tap');
    this.dialog.open(CommandPaletteComponent, {
      autoFocus: 'input',
      restoreFocus: true,
      maxWidth: 'calc(100vw - 24px)',
      panelClass: 'm3-dialog-panel',
      backdropClass: 'rvt-palette-backdrop'
    });
  }

  onKeyboardShortcut(event: KeyboardEvent) {
    if (event.defaultPrevented) return;
    const target = event.target instanceof Element ? event.target : null;
    if (target?.closest('input, textarea, select, [contenteditable], [role="textbox"]')) {
      return;
    }
    const key = event.key.toLowerCase();
    const primaryModifier = event.ctrlKey || event.metaKey;
    const plainKey = !primaryModifier && !event.altKey && !event.shiftKey;

    if (primaryModifier && event.shiftKey && key === 'c') {
      event.preventDefault();
      void this.copyOperatorSummary();
      return;
    }
    if (primaryModifier && !event.altKey && !event.shiftKey && key === 'z') {
      event.preventDefault();
      this.state.triggerHaptic('tap');
      this.snackBar.open('Shortcut recognized: Undo is not yet available in the modern dashboard.', 'Dismiss', { duration: 3000 });
      return;
    }
    if (primaryModifier && !event.altKey && !event.shiftKey && key === 'h') {
      event.preventDefault();
      this.state.triggerHaptic('tap');
      this.snackBar.open('Shortcut recognized: Operator handoff modal is not yet available. Use Ctrl+Shift+C to copy operator brief.', 'Dismiss', { duration: 4000 });
      return;
    }
    if (primaryModifier && !event.altKey && !event.shiftKey && key === 'l') {
      event.preventDefault();
      this.state.triggerHaptic('tap');
      this.snackBar.open('Shortcut recognized: Idle auto-lock is not yet available.', 'Dismiss', { duration: 3000 });
      return;
    }
    if (primaryModifier && !event.altKey && key === 'k') {
      event.preventDefault();
      this.openPalette();
      return;
    }
    if (plainKey && key === '/') {
      event.preventDefault();
      this.openPalette();
      return;
    }
    if (event.key === '?' && !primaryModifier && !event.altKey) {
      event.preventDefault();
      this.openShortcuts();
      return;
    }
    if (event.altKey && !primaryModifier && !event.shiftKey && ['1', '2', '3', '4', '5', '6'].includes(event.key)) {
      event.preventDefault();
      void this.router.navigate(['/live']).then(() => {
        const tabs = ['tab-overview', 'tab-waves', 'tab-hr', 'tab-rr', 'tab-snaps', 'tab-audit'];
        this.state.activeTab.set(tabs[Number(event.key) - 1]);
      });
      return;
    }
    if ((plainKey && key === '[') || (event.altKey && !primaryModifier && key === '\\')) {
      event.preventDefault();
      this.toggleRailCollapse();
      return;
    }
    if (event.shiftKey && !primaryModifier && !event.altKey && key === 'f') {
      event.preventDefault();
      this.state.zenMode.update(enabled => !enabled);
      this.state.triggerHaptic('tap');
      return;
    }
    if (!plainKey) return;

    if (key === 't') {
      event.preventDefault();
      const themes = ['light', 'dark', 'night', 'hc'] as const;
      const current = themes.indexOf(this.state.theme());
      this.state.theme.set(themes[(current + 1) % themes.length]);
      this.state.triggerHaptic('tap');
      return;
    }
    if (key === ' ' && this.state.currentView() === 'live') {
      event.preventDefault();
      this.state.paused.update(paused => !paused);
      return;
    }
    if (key === 'a') {
      event.preventDefault();
      this.openAlerts();
      return;
    }
    if (key === 'e' || key === 'x') {
      event.preventDefault();
      this.exportCurrentPayload();
      return;
    }
    if (key === ',' || (key === 'm' && this.state.currentView() !== 'live')) {
      event.preventDefault();
      if (key === 'm') {
        this.toggleRailCollapse();
      } else {
        void this.router.navigate(['/settings']);
      }
      return;
    }
    if (key === 'b' && this.state.currentView() !== 'live') {
      event.preventDefault();
      this.state.audioAlertsEnabled.update(enabled => !enabled);
      return;
    }
    if (key === 'v' && this.state.currentView() !== 'live') {
      event.preventDefault();
      this.state.voiceAlertsEnabled.update(enabled => !enabled);
      return;
    }

    const paths: Record<string, string> = {
      '1': '/home',
      '2': '/live',
      '3': '/report',
      '4': '/help',
      '5': '/settings',
      h: '/home',
      l: '/live',
      r: '/report',
      w: '/help'
    };
    const path = paths[key] || (key === 's' && this.state.currentView() !== 'live' ? '/settings' : '');
    if (path) {
      event.preventDefault();
      void this.router.navigate([path]);
    }
  }

  private touchStartX = 0;
  private touchStartY = 0;

  @HostListener('touchstart', ['$event'])
  onTouchStart(event: TouchEvent) {
    if (this.windowSizeClass() !== 'Compact' || this.state.currentView() !== 'live') return;
    const touch = event.touches[0];
    this.touchStartX = touch.clientX;
    this.touchStartY = touch.clientY;
  }

  @HostListener('touchend', ['$event'])
  onTouchEnd(event: TouchEvent) {
    if (this.windowSizeClass() !== 'Compact' || this.state.currentView() !== 'live') return;
    const touch = event.changedTouches[0];
    const deltaX = touch.clientX - this.touchStartX;
    const deltaY = touch.clientY - this.touchStartY;

    if (Math.abs(deltaX) > 60 && Math.abs(deltaX) > 2 * Math.abs(deltaY)) {
      const tabs = ['tab-overview', 'tab-waves', 'tab-hr', 'tab-rr', 'tab-snaps', 'tab-audit'];
      const currentTab = this.state.activeTab();
      const currentIdx = tabs.indexOf(currentTab);
      if (currentIdx === -1) return;

      let nextIdx = currentIdx;
      if (deltaX < 0) {
        nextIdx = currentIdx + 1;
      } else {
        nextIdx = currentIdx - 1;
      }

      if (nextIdx >= 0 && nextIdx < tabs.length) {
        const nextTab = tabs[nextIdx];
        this.state.activeTab.set(nextTab);
        this.state.triggerHaptic('tap');

        const label = nextTab.replace('tab-', '').toUpperCase();
        this.snackBar.open(`Switched to tab: ${label}`, 'Dismiss', { duration: 1500 });
      }
    }
  }

  onMainScroll(event: Event) {
    const target = event.target as HTMLElement;
    if (target) {
      this.showScrollTop.set(target.scrollTop > 300);
    }
  }



  scrollToTop() {
    this.state.triggerHaptic('tap');
    if (this.mainContentScroll) {
      this.mainContentScroll.nativeElement.scrollTo({
        top: 0,
        behavior: 'smooth'
      });
    }
  }

  skipToMainContent(event: Event): void {
    event.preventDefault();
    this.mainContentScroll?.nativeElement.focus();
  }

  openShortcuts() {
    this.state.triggerHaptic('tap');
    this.dialog.open(KeyboardShortcutsDialogComponent, {
      restoreFocus: true,
      panelClass: 'm3-dialog-panel'
    });
  }

  private openAlerts(): void {
    this.state.triggerHaptic('tap');
    this.dialog.open(AlertsDialogComponent, {
      maxWidth: 'calc(100vw - 24px)',
      restoreFocus: true,
      panelClass: 'm3-dialog-panel'
    });
  }

  private exportCurrentPayload(): void {
    const payload = this.state.lastLivePayload() || this.state.lastPayload();
    if (!payload) return;
    const anchor = document.createElement('a');
    const href = URL.createObjectURL(new Blob([JSON.stringify(payload, null, 2)], { type: 'application/json' }));
    anchor.href = href;
    anchor.download = `rvt_payload_${Date.now()}.json`;
    anchor.click();
    URL.revokeObjectURL(href);
    this.state.triggerHaptic('success');
  }

  private async copyOperatorSummary(): Promise<void> {
    if (!navigator.clipboard) return;
    const payload = this.state.lastPayload();
    const summary = [
      `Operator: ${this.state.setup().operator_label || 'Operator A'}`,
      `View: ${this.state.currentView()}`,
      `Source: ${this.state.ctlStatus()?.mode === 'sandbox' ? 'DEMO' : 'Trainer API'}`,
      `Heart rate: ${payload?.radar?.reported_hr ?? '--'} bpm`,
      `Respiration: ${payload?.radar?.reported_rr ?? '--'} br/min`,
      `Session active: ${this.state.sessionActive() ? 'yes' : 'no'}`
    ].join('\n');
    try {
      await navigator.clipboard.writeText(summary);
      this.state.triggerHaptic('confirm');
    } catch {
      // Clipboard access may be blocked by a browser permission policy.
    }
  }

  private syncCurrentView(url: string): void {
    const path = url.split(/[/?#]/).filter(Boolean)[0] || 'home';
    this.state.currentView.set(path);
    document.body.dataset['view'] = path;
  }

  private setRailCollapsed(collapsed: boolean): void {
    this.railCollapsed.set(collapsed);
    document.body.dataset['railCollapsed'] = collapsed ? '1' : '0';
    localStorage.setItem('rvt-rail-collapsed', collapsed ? '1' : '0');
    requestAnimationFrame(() => this.sidenavContainer?.updateContentMargins());
  }
}
