import { Routes } from '@angular/router';
import { LayoutComponent } from './components/layout/layout.component';
import { activeSessionGuard } from './guards/active-session.guard';

export const routes: Routes = [
  {
    path: '',
    component: LayoutComponent,
    children: [
      { path: '', redirectTo: 'live', pathMatch: 'full' },
      {
        path: 'home',
        loadComponent: () => import('./components/home/home.component').then(m => m.HomeComponent)
      },
      {
        path: 'live',
        loadComponent: () => import('./components/live/live.component').then(m => m.LiveComponent),
        canDeactivate: [activeSessionGuard]
      },
      {
        path: 'report',
        loadComponent: () => import('./components/report/report.component').then(m => m.ReportComponent)
      },
      {
        path: 'help',
        loadComponent: () => import('./components/help/help.component').then(m => m.HelpComponent)
      },
      {
        path: 'settings',
        loadComponent: () => import('./components/settings/settings.component').then(m => m.SettingsComponent)
      }
    ]
  },
  { path: '**', redirectTo: 'live' }
];
