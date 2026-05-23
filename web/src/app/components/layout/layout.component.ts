import { Component, inject, OnInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import { Router, NavigationEnd, RouterModule } from '@angular/router';
import { filter } from 'rxjs/operators';
import { MatDialog } from '@angular/material/dialog';

import { StateService } from '../../services/state.service';
import { TopbarComponent } from '../topbar/topbar.component';
import { SettingsComponent } from '../settings/settings.component';

@Component({
  selector: 'app-layout',
  standalone: true,
  imports: [
    CommonModule,
    RouterModule,
    TopbarComponent
  ],
  templateUrl: './layout.component.html',
  styleUrl: './layout.component.css'
})
export class LayoutComponent implements OnInit {
  protected readonly state = inject(StateService);
  private readonly router = inject(Router);
  private readonly dialog = inject(MatDialog);

  railCollapsed = false;
  mobileDrawerOpen = false;

  ngOnInit() {
    // Keep StateService currentView signal perfectly in sync with active Angular routes
    this.router.events.pipe(
      filter(event => event instanceof NavigationEnd)
    ).subscribe((event: any) => {
      const path = event.urlAfterRedirects.split('/')[1] || 'live';
      this.state.currentView.set(path);
    });
  }

  toggleRailCollapse() {
    this.railCollapsed = !this.railCollapsed;
    this.state.triggerHaptic('tap');
  }

  toggleMobileDrawer() {
    this.mobileDrawerOpen = !this.mobileDrawerOpen;
    this.state.triggerHaptic('tap');
  }

  openPalette() {
    this.state.triggerHaptic('tap');
    // Renders the command palette if integrated or prompts
    const command = prompt('Command Palette:\nEnter search term or keyboard shortcut (e.g. "/theme", "/help", "/reset"):');
    if (!command) return;
    
    const cmd = command.trim().toLowerCase();
    if (cmd === '/theme') {
      const cycle: ('light' | 'dark' | 'night' | 'hc')[] = ['light', 'dark', 'night', 'hc'];
      this.state.theme.set(cycle[(cycle.indexOf(this.state.theme()) + 1) % cycle.length]);
      this.state.triggerHaptic('success');
    } else if (cmd === '/help') {
      this.router.navigate(['/help']);
    } else if (cmd === '/reset') {
      if (confirm('Reset all defaults?')) {
        this.state.theme.set('dark');
        this.state.density.set('comfortable');
        this.state.kpiThresholds.set({ hrLow: 40, hrHigh: 140, rrLow: 6, rrHigh: 30 });
        this.state.triggerHaptic('destructiveAccept');
      }
    } else {
      alert(`Command "${command}" not recognized.`);
    }
  }

  openSettings() {
    this.dialog.open(SettingsComponent, {
      width: '600px',
      maxWidth: '95vw',
      panelClass: 'm3-dialog-panel'
    });
    this.state.triggerHaptic('tap');
  }
}
