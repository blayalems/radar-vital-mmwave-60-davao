import { Injectable, signal } from '@angular/core';
import { Capacitor } from '@capacitor/core';
import { Haptics, ImpactStyle, NotificationType } from '@capacitor/haptics';
import { ThemeId, DensityId, PaletteId, HapticMode } from '../../models/rvt.models';

@Injectable({
  providedIn: 'root'
})
export class UiStore {
  currentView = signal<string>('live');
  activeTab = signal<string>('tab-overview');
  commandPaletteOpen = signal<boolean>(false);
  alertsOpen = signal<boolean>(false);
  
  theme = signal<ThemeId>('dark');
  palette = signal<PaletteId>('azure');
  density = signal<DensityId>('comfortable');
  fontScale = signal<number>(1);
  zenMode = signal<boolean>(false);

  demoMode = signal<boolean>(false);
  autoDemoOnDisconnect = signal<boolean>(false);
  autoDemoActive = signal<boolean>(false);

  hxMode = signal<HapticMode>('auto');

  triggerHaptic(kind: string, sessionActive: boolean, ctlStopPending: boolean) {
    if (this.hxMode() === 'off') return;

    const reducedMotion = window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    const active = sessionActive && !ctlStopPending;

    if (active && !['safety', 'confirm-stop', 'reject', 'destructiveAccept'].includes(kind)) return;
    if (reducedMotion && kind !== 'safety') return;

    if (Capacitor.isNativePlatform()) {
      try {
        switch (kind) {
          case 'tap':
            void Haptics.impact({ style: ImpactStyle.Light });
            break;
          case 'confirm':
            void Haptics.impact({ style: ImpactStyle.Light });
            break;
          case 'success':
            void Haptics.notification({ type: NotificationType.Success });
            break;
          case 'warn':
            void Haptics.notification({ type: NotificationType.Warning });
            break;
          case 'safety':
          case 'reject':
            void Haptics.notification({ type: NotificationType.Error });
            break;
          case 'destructiveAccept':
            void Haptics.impact({ style: ImpactStyle.Heavy });
            break;
          case 'sessionStart':
            void Haptics.impact({ style: ImpactStyle.Medium });
            break;
          case 'sessionStop':
            void Haptics.impact({ style: ImpactStyle.Medium });
            break;
          default:
            void Haptics.impact({ style: ImpactStyle.Light });
        }
        return;
      } catch (e) {
        console.warn('[Haptics] Capacitor haptics failed:', e);
      }
    }

    if (typeof navigator === 'undefined' || !('vibrate' in navigator)) return;

    try {
      switch (kind) {
        case 'tap':
          navigator.vibrate(10);
          break;
        case 'confirm':
          navigator.vibrate(15);
          break;
        case 'success':
          navigator.vibrate([12, 40, 12]);
          break;
        case 'warn':
          navigator.vibrate([8, 60, 8]);
          break;
        case 'safety':
        case 'reject':
          navigator.vibrate([20, 40, 20]);
          break;
        case 'destructiveAccept':
          navigator.vibrate([40, 80, 40]);
          break;
        case 'sessionStart':
          navigator.vibrate([18, 30, 18]);
          break;
        case 'sessionStop':
          navigator.vibrate(22);
          break;
        default:
          navigator.vibrate(10);
      }
    } catch (_) {}
  }
}
