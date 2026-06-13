import { Routes } from '@angular/router';
import { LayoutComponent } from './components/layout/layout.component';
import { LiveComponent } from './components/live/live.component';
import { HomeComponent } from './components/home/home.component';
import { activeSessionGuard } from './guards/active-session.guard';
import { connectionGuard } from './guards/connection.guard';
import { firstRunGuard } from './guards/first-run.guard';
import { connectRouteGuard } from './guards/connect-route.guard';
import { ConnectWizardComponent } from './components/connect-wizard/connect-wizard.component';

export const routes: Routes = [
  {
    path: 'connect',
    component: ConnectWizardComponent,
    canActivate: [connectRouteGuard]
  },
  {
    path: '',
    component: LayoutComponent,
    canActivate: [firstRunGuard],
    children: [
      { path: '', redirectTo: 'live', pathMatch: 'full' },
      {
        path: 'home',
        component: HomeComponent
      },
      {
        path: 'live',
        component: LiveComponent,
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


