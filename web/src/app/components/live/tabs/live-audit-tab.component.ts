import { ChangeDetectionStrategy, Component, input } from '@angular/core';
import { DatePipe } from '@angular/common';
import { MatButtonModule } from '@angular/material/button';
import { MatCardModule } from '@angular/material/card';
import { MatIconModule } from '@angular/material/icon';
import { MatMenuModule } from '@angular/material/menu';
import { LiveAuditTabViewModel } from '../live-tab-view-models';

@Component({
  selector: 'live-audit-tab',
  imports: [
    DatePipe,
    MatButtonModule,
    MatCardModule,
    MatIconModule,
    MatMenuModule
  ],
  templateUrl: './live-audit-tab.component.html',
  styleUrl: './live-tab-panel.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class LiveAuditTabComponent {
  readonly context = input.required<LiveAuditTabViewModel>();
}
