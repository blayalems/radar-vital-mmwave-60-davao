import { ChangeDetectionStrategy, Component, input } from '@angular/core';
import { DatePipe } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { MatButtonModule } from '@angular/material/button';
import { MatCardModule } from '@angular/material/card';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatIconModule } from '@angular/material/icon';
import { MatInputModule } from '@angular/material/input';
import { LiveSnapsTabViewModel } from '../live-tab-view-models';

@Component({
  selector: 'live-snaps-tab',
  imports: [
    DatePipe,
    FormsModule,
    MatButtonModule,
    MatCardModule,
    MatFormFieldModule,
    MatIconModule,
    MatInputModule
  ],
  templateUrl: './live-snaps-tab.component.html',
  styleUrl: './live-tab-panel.css',
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class LiveSnapsTabComponent {
  readonly context = input.required<LiveSnapsTabViewModel>();
  protected readonly Math = Math;
}
