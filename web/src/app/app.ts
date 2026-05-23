import { Component, signal, OnInit } from '@angular/core';
import { RouterOutlet } from '@angular/router';

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

    // Register service worker /sw.js
    navigator.serviceWorker.register('/sw.js').then(reg => {
      reg.addEventListener('updatefound', () => {
        const w = reg.installing;
        if (!w) return;
        w.addEventListener('statechange', () => {
          if (w.state === 'installed' && navigator.serviceWorker.controller) {
            console.log('New service worker version available. Reloading.');
            location.reload();
          }
        });
      });
    }).catch(err => {
      console.warn('Service worker registration failed', err);
    });

    navigator.serviceWorker.addEventListener('controllerchange', () => {
      location.reload();
    });
  }
}
