import { ApplicationConfig, provideBrowserGlobalErrorListeners } from '@angular/core';
import { provideRouter, withComponentInputBinding, withInMemoryScrolling } from '@angular/router';
import { provideAnimations } from '@angular/platform-browser/animations';
import { provideHttpClient, withFetch, withInterceptors } from '@angular/common/http';
import { provideServiceWorker } from '@angular/service-worker';

import { routes } from './app.routes';
import { rvtAuthInterceptor } from './services/interceptors/auth.interceptor';
import { rvtTauriInterceptor } from './services/interceptors/tauri.interceptor';

export const appConfig: ApplicationConfig = {
  providers: [
    provideBrowserGlobalErrorListeners(),
    provideRouter(
      routes,
      withComponentInputBinding(),
      withInMemoryScrolling({ scrollPositionRestoration: 'enabled' })
    ),
    provideAnimations(),
    provideHttpClient(
      withFetch(),
      withInterceptors([rvtAuthInterceptor, rvtTauriInterceptor])
    ),
    provideServiceWorker('sw.js', {
      enabled: typeof navigator !== 'undefined' && 'serviceWorker' in navigator,
      registrationStrategy: 'registerWhenStable:30000'
    })
  ]
};
