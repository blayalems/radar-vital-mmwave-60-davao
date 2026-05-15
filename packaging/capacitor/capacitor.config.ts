import type { CapacitorConfig } from '@capacitor/cli';

const config: CapacitorConfig = {
  appId: 'com.blayalems.radarvital',
  appName: 'Radar Vital',
  webDir: 'www',
  bundledWebRuntime: false,
  server: {
    androidScheme: 'https',
    resolveServiceWorkerRequests: false
  },
  plugins: {
    CapacitorHttp: {
      enabled: true
    }
  },
  android: {
    allowMixedContent: false,
    captureInput: true
  }
};

export default config;
