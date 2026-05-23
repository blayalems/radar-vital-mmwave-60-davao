import { Component, signal, OnInit } from '@angular/core';
import { RouterOutlet } from '@angular/router';

let reloading = false;

@Component({
  selector: 'app-root',
  imports: [RouterOutlet],
  templateUrl: './app.html',
  styleUrl: './app.css'
})
export class App implements OnInit {
  protected readonly title = signal('web');

  ngOnInit() {
    this.installServiceWorker();
  }

  private installServiceWorker() {
    if (typeof window === 'undefined' || !('serviceWorker' in navigator)) return;

    // Track active controller presence on load to guarantee first install never triggers a reload
    const hasController = !!navigator.serviceWorker.controller;

    // Register service worker ./sw.js
    navigator.serviceWorker.register('./sw.js').then(reg => {
      reg.addEventListener('updatefound', () => {
        const w = reg.installing;
        if (!w) return;
        w.addEventListener('statechange', () => {
          if (w.state === 'installed' && navigator.serviceWorker.controller) {
            console.log('New service worker version available. Triggering skipWaiting.');
            w.postMessage({ type: 'SKIP_WAITING' });
          }
        });
      });
    }).catch(err => {
      console.warn('Service worker registration failed', err);
    });

    // Double-reload guard: During worker updates, the browser may fire controllerchange
    // and statechange in quick succession. Gating reloads with hasController and reloading
    // ensures we reload exactly once, and only on actual updates.
    navigator.serviceWorker.addEventListener('controllerchange', () => {
      if (!hasController) return; // First install does not reload
      if (reloading) return;
      reloading = true;
      location.reload();
    });
  }
}
