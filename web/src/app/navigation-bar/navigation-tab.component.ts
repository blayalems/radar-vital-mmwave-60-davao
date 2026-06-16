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
      transition: color var(--md-sys-motion-duration-long1, 300ms) var(--md-sys-motion-easing-emphasized, cubic-bezier(0.2, 0, 0, 1));
    }
 
    .mat-navigation-tab-pill {
      position: absolute;
      inset: 0;
      border-radius: 16px;
      background-color: var(--md-sys-color-secondary-container);
      opacity: 0;
      transform: scaleX(0.4);
      z-index: 0;
      clip-path: path('M 24,8 H 40 A 8,8 0 0,1 48,16 A 8,8 0 0,1 40,24 H 24 A 8,8 0 0,1 16,16 A 8,8 0 0,1 24,8 Z');
      transition: opacity var(--dur-spring, 500ms) var(--ease-spring, cubic-bezier(0.34, 1.56, 0.64, 1)),
                  transform var(--dur-spring, 500ms) var(--ease-spring, cubic-bezier(0.34, 1.56, 0.64, 1)),
                  clip-path var(--dur-spring, 500ms) var(--ease-spring, cubic-bezier(0.34, 1.56, 0.64, 1));
    }
 
    /* Target the projected icons inside */
    .mat-navigation-tab-icon-container .material-symbols-rounded,
    .mat-navigation-tab-icon-container mat-icon {
      position: relative;
      z-index: 1;
      font-size: 24px;
      font-variation-settings: 'FILL' 0, 'wght' 400, 'GRAD' 0, 'opsz' 24;
      transition: font-variation-settings var(--md-sys-motion-duration-long1, 300ms) var(--md-sys-motion-easing-emphasized, cubic-bezier(0.2, 0, 0, 1));
    }
 
    .mat-navigation-tab-label {
      font-family: var(--md-sys-typescale-label-medium-font-family, inherit);
      font-size: var(--md-sys-typescale-label-medium-font-size, 12px);
      font-weight: var(--md-sys-typescale-label-medium-font-weight, 500);
      line-height: var(--md-sys-typescale-label-medium-line-height, 16px);
      letter-spacing: var(--md-sys-typescale-label-medium-letter-spacing, 0.5px);
      color: var(--md-sys-color-on-surface-variant);
      transition: color var(--md-sys-motion-duration-long1, 300ms) var(--md-sys-motion-easing-emphasized, cubic-bezier(0.2, 0, 0, 1)),
                  font-weight var(--md-sys-motion-duration-long1, 300ms) var(--md-sys-motion-easing-emphasized, cubic-bezier(0.2, 0, 0, 1));
      text-align: center;
      /* Translation-safe: wrap long localized labels (up to two lines) instead
         of clipping with an ellipsis, which previously hid non-English text. */
      white-space: normal;
      overflow-wrap: anywhere;
      hyphens: auto;
      display: -webkit-box;
      -webkit-box-orient: vertical;
      -webkit-line-clamp: 2;
      line-clamp: 2;
      overflow: hidden;
      width: 100%;
      box-sizing: border-box;
      padding: 0 2px;
    }
 
    /* Active States */
    a[mat-navigation-tab].active .mat-navigation-tab-pill,
    button[mat-navigation-tab].active .mat-navigation-tab-pill,
    mat-navigation-tab.active .mat-navigation-tab-pill {
      opacity: 1;
      transform: scaleX(1);
      clip-path: path('M 16,0 H 48 A 16,16 0 0,1 64,16 A 16,16 0 0,1 48,32 H 16 A 16,16 0 0,1 0,16 A 16,16 0 0,1 16,0 Z');
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
