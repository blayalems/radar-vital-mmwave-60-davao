import { Routes } from '@angular/router';
import { activeSessionGuard } from './guards/active-session.guard';
import { connectionGuard } from './guards/connection.guard';
import { firstRunGuard } from './guards/first-run.guard';
import { connectRouteGuard } from './guards/connect-route.guard';

export const routes: Routes = [
  {
    path: 'connect',
    loadComponent: () => import('./components/connect-wizard/connect-wizard.component')
      .then(module => module.ConnectWizardComponent),
    canActivate: [connectRouteGuard]
  },
  {
    path: '',
    loadComponent: () => import('./components/layout/layout.component')
      .then(module => module.LayoutComponent),
    canActivate: [firstRunGuard],
    children: [
      { path: '', redirectTo: 'live', pathMatch: 'full' },
      {
        path: 'home',
        loadComponent: () => import('./components/home/home.component')
          .then(module => module.HomeComponent)
      },
      {
        path: 'live',
        loadComponent: () => import('./components/live/live.component')
          .then(module => module.LiveComponent),
        canActivate: [connectionGuard],
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


