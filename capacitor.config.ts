import type { CapacitorConfig } from '@capacitor/cli';

const config: CapacitorConfig = {
  appId: 'app.radarvital.trainer',
  appName: 'Radar Vital',
  webDir: 'www',
  // The dashboard reads S.apiBase from localStorage. On native, traffic to that
  // base goes through @capacitor/http (native HTTP stack), so the WebView's
  // mixed-content rules never apply and no network_security_config exemption
  // is needed. We deliberately do NOT set server.url — the bundled www/ is
  // the source of truth.
  server: {
    androidScheme: 'https'
  },
  plugins: {
    CapacitorHttp: {
      enabled: true
    },
    BluetoothLe: {
      displayStrings: {
        scanning: 'Scanning for Radar Vital devices…',
        cancel: 'Cancel',
        availableDevices: 'Available devices',
        noDeviceFound: 'No Radar Vital device found'
      }
    },
    StatusBar: {
      overlaysWebView: true
    },
    SplashScreen: {
      launchShowDuration: 0,
      launchAutoHide: false,
      backgroundColor: '#0f172a',
      androidSplashResourceName: 'splash',
      showSpinnerOnPrelaunch: false
    }
  },
  android: {
    allowMixedContent: false,
    // Service worker behavior in Capacitor is configurable; we resolve through
    // the SW so the local cache strategies in assets/sw.js apply.
    resolveServiceWorkerRequests: true
  }
};

export default config;
