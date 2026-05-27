import { ChangeDetectionStrategy, Component, ViewEncapsulation } from '@angular/core';

@Component({
  selector: 'mat-navigation-bar, app-navigation-bar',
  standalone: true,
  template: `
    <div class="mat-navigation-bar-container">
      <ng-content></ng-content>
    </div>
  `,
  styles: [`
    mat-navigation-bar, app-navigation-bar {
      display: flex;
      flex-direction: row;
      justify-content: space-around;
      align-items: center;
      height: calc(80px + env(safe-area-inset-bottom, 0px));
      padding-bottom: env(safe-area-inset-bottom, 0px);
      background-color: var(--md-sys-color-surface-container);
      border-top: 1px solid var(--md-sys-color-outline-variant);
      box-sizing: border-box;
      width: 100%;
      position: sticky;
      bottom: 0;
      z-index: 800;
    }
    .mat-navigation-bar-container {
      display: flex;
      flex-direction: row;
      width: 100%;
      height: 100%;
      align-items: stretch;
    }
  `],
  encapsulation: ViewEncapsulation.None,
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class MatNavigationBar {}
