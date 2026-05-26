import { ChangeDetectionStrategy, Component, DestroyRef, inject, OnInit, signal, ViewChild, ElementRef } from '@angular/core';
import { CommonModule } from '@angular/common';
import { Router, NavigationEnd, RouterModule } from '@angular/router';
import { BreakpointObserver } from '@angular/cdk/layout';
import { filter } from 'rxjs/operators';
import { takeUntilDestroyed } from '@angular/core/rxjs-interop';
import { MatButtonModule } from '@angular/material/button';
import { MatDialog, MatDialogModule } from '@angular/material/dialog';
import { MatIconModule } from '@angular/material/icon';
import { MatListModule } from '@angular/material/list';
import { MatSidenavModule } from '@angular/material/sidenav';
import { MatToolbarModule } from '@angular/material/toolbar';
import { MatProgressSpinnerModule } from '@angular/material/progress-spinner';

import { StateService } from '../../services/state.service';
import { ApiService } from '../../services/api.service';
import { TopbarComponent } from '../topbar/topbar.component';
import { CommandPaletteComponent } from '../command-palette/command-palette.component';
import { KeyboardShortcutsDialogComponent } from '../keyboard-shortcuts-dialog/keyboard-shortcuts-dialog.component';

@Component({
  selector: 'app-layout',
  imports: [
    CommonModule,
    RouterModule,
    TopbarComponent,
    MatButtonModule,
    MatDialogModule,
    MatIconModule,
    MatListModule,
    MatSidenavModule,
    MatToolbarModule,
    MatProgressSpinnerModule
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
  private readonly destroyRef = inject(DestroyRef);
  private readonly breakpointObserver = inject(BreakpointObserver);

  @ViewChild('mainContentScroll', { static: false }) mainContentScroll?: ElementRef<HTMLElement>;
  protected readonly showScrollTop = signal(false);

  railCollapsed = false;
  protected readonly showRail = signal(false);

  ngOnInit() {
    this.breakpointObserver.observe('(min-width: 1024px)').pipe(
      takeUntilDestroyed(this.destroyRef)
    ).subscribe(result => this.showRail.set(result.matches));
    this.syncCurrentView(this.router.url);
    this.router.events.pipe(
      filter((event): event is NavigationEnd => event instanceof NavigationEnd),
      takeUntilDestroyed(this.destroyRef)
    ).subscribe(event => {
      this.syncCurrentView(event.urlAfterRedirects);
    });
  }

  toggleRailCollapse() {
    this.railCollapsed = !this.railCollapsed;
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
    const target = event.target instanceof Element ? event.target : null;
    if (target?.closest('input, textarea, select, [contenteditable], [role="textbox"]')) {
      return;
    }
    if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === 'k') {
      event.preventDefault();
      this.openPalette();
      return;
    }
    if (((event.ctrlKey || event.metaKey) && event.key === '/') || event.key === '?') {
      event.preventDefault();
      this.openShortcuts();
      return;
    }
    if (event.altKey && ['1', '2', '3', '4', '5', '6'].includes(event.key)) {
      event.preventDefault();
      void this.router.navigate(['/live']).then(() => {
        const tabs = ['tab-overview', 'tab-waves', 'tab-hr', 'tab-rr', 'tab-snaps', 'tab-audit'];
        this.state.activeTab.set(tabs[Number(event.key) - 1]);
      });
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

  openShortcuts() {
    this.state.triggerHaptic('tap');
    this.dialog.open(KeyboardShortcutsDialogComponent, {
      restoreFocus: true,
      panelClass: 'm3-dialog-panel'
    });
  }

  private syncCurrentView(url: string): void {
    const path = url.split(/[/?#]/).filter(Boolean)[0] || 'home';
    this.state.currentView.set(path);
    document.body.dataset['view'] = path;
  }
}
