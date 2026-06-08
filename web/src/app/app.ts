import { Component, OnInit, inject } from '@angular/core';
import { RouterOutlet } from '@angular/router';
import { MatDialogModule } from '@angular/material/dialog';
import { MatSnackBarModule } from '@angular/material/snack-bar';

import { SwUpdateService } from './services/sw-update.service';

@Component({
  selector: 'app-root',
  imports: [RouterOutlet, MatDialogModule, MatSnackBarModule],
  templateUrl: './app.html',
  styleUrl: './app.css'
})
export class App implements OnInit {
  private readonly swUpdate = inject(SwUpdateService);

  ngOnInit() {
    this.swUpdate.initialize();
  }
}
