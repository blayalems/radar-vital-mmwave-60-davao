import { ChangeDetectionStrategy, Component, ViewEncapsulation } from '@angular/core';

@Component({
  selector: 'a[mat-navigation-tab], button[mat-navigation-tab], mat-navigation-tab',
  standalone: true,
  template: `
    <div class="mat-navigation-tab-wrapper">
      <div class="mat-navigation-tab-icon-container">
        <div class="mat-navigation-tab-pill"></div>
        <ng-content select="[icon], .material-symbols-rounded, mat-icon"></ng-content>
      </div>
      <span class="mat-navigation-tab-label">
        <ng-content select="[label], span:not([icon]):not(.material-symbols-rounded):not(mat-icon)"></ng-content>
      </span>
    </div>
  `,
  styles: [`
    a[mat-navigation-tab], button[mat-navigation-tab], mat-navigation-tab {
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      flex: 1;
      min-width: 0;
      height: 100%;
      cursor: pointer;
      text-decoration: none;
      box-sizing: border-box;
      position: relative;
      background: transparent;
      border: none;
      outline: none;
      padding: 0;
      -webkit-tap-highlight-color: transparent;
    }

    .mat-navigation-tab-wrapper {
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      width: 100%;
      height: 100%;
      gap: 4px;
    }

    .mat-navigation-tab-icon-container {
      position: relative;
      display: inline-flex;
      align-items: center;
      justify-content: center;
      height: 32px;
      width: 64px;
      border-radius: 16px;
      color: var(--md-sys-color-on-surface-variant);
      transition: color 300ms cubic-bezier(0.2, 0, 0, 1);
    }

    .mat-navigation-tab-pill {
      position: absolute;
      inset: 0;
      border-radius: 16px;
      background-color: var(--md-sys-color-secondary-container);
      opacity: 0;
      transform: scaleX(0.4);
      z-index: 0;
      transition: opacity 350ms cubic-bezier(0.2, 0, 0, 1), transform 350ms cubic-bezier(0.2, 0, 0, 1);
    }

    /* Target the projected icons inside */
    .mat-navigation-tab-icon-container .material-symbols-rounded,
    .mat-navigation-tab-icon-container mat-icon {
      position: relative;
      z-index: 1;
      font-size: 24px;
      font-variation-settings: 'FILL' 0, 'wght' 400, 'GRAD' 0, 'opsz' 24;
      transition: font-variation-settings 300ms cubic-bezier(0.2, 0, 0, 1);
    }

    .mat-navigation-tab-label {
      font-family: var(--md-sys-typescale-label-medium-font-family, inherit);
      font-size: var(--md-sys-typescale-label-medium-font-size, 12px);
      font-weight: var(--md-sys-typescale-label-medium-font-weight, 500);
      line-height: var(--md-sys-typescale-label-medium-line-height, 16px);
      letter-spacing: var(--md-sys-typescale-label-medium-letter-spacing, 0.5px);
      color: var(--md-sys-color-on-surface-variant);
      transition: color 300ms cubic-bezier(0.2, 0, 0, 1), font-weight 300ms cubic-bezier(0.2, 0, 0, 1);
      text-align: center;
      white-space: nowrap;
      overflow: hidden;
      text-overflow: ellipsis;
      width: 100%;
      box-sizing: border-box;
      padding: 0 4px;
    }

    /* Active States */
    a[mat-navigation-tab].active .mat-navigation-tab-pill,
    button[mat-navigation-tab].active .mat-navigation-tab-pill,
    mat-navigation-tab.active .mat-navigation-tab-pill {
      opacity: 1;
      transform: scaleX(1);
    }

    a[mat-navigation-tab].active .mat-navigation-tab-icon-container,
    button[mat-navigation-tab].active .mat-navigation-tab-icon-container,
    mat-navigation-tab.active .mat-navigation-tab-icon-container {
      color: var(--md-sys-color-on-secondary-container);
    }

    a[mat-navigation-tab].active .mat-navigation-tab-icon-container .material-symbols-rounded,
    button[mat-navigation-tab].active .mat-navigation-tab-icon-container .material-symbols-rounded,
    mat-navigation-tab.active .mat-navigation-tab-icon-container .material-symbols-rounded {
      font-variation-settings: 'FILL' 1, 'wght' 400, 'GRAD' 0, 'opsz' 24;
    }

    a[mat-navigation-tab].active .mat-navigation-tab-label,
    button[mat-navigation-tab].active .mat-navigation-tab-label,
    mat-navigation-tab.active .mat-navigation-tab-label {
      color: var(--md-sys-color-on-surface);
      font-weight: 700;
    }
  `],
  encapsulation: ViewEncapsulation.None,
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class MatNavigationTab {}
