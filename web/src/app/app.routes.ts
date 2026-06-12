import { Routes } from '@angular/router';
import { LayoutComponent } from './components/layout/layout.component';
import { LiveComponent } from './components/live/live.component';
import { HomeComponent } from './components/home/home.component';
import { ReportComponent } from './components/report/report.component';
import { HelpComponent } from './components/help/help.component';
import { SettingsComponent } from './components/settings/settings.component';
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
        component: ReportComponent
      },
      {
        path: 'help',
        component: HelpComponent
      },
      {
        path: 'settings',
        component: SettingsComponent
      }
    ]
  },
  { path: '**', redirectTo: 'live' }
];


