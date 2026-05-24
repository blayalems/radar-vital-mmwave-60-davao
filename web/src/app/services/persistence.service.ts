import { Injectable } from '@angular/core';

export type StorageScope = 'demo' | 'live' | 'legacy-unclassified';
export type PersistenceStore =
  | 'snapshots'
  | 'session-notes'
  | 'snap-notes'
  | 'waveform-buffers'
  | 'settings-backup'
  | 'alert-history'
  | 'sessions';

const DB_NAME = 'radar-vital-v12';
const DB_VERSION = 1;
const STORES: PersistenceStore[] = [
  'snapshots',
  'session-notes',
  'snap-notes',
  'waveform-buffers',
  'settings-backup',
  'alert-history',
  'sessions'
];

@Injectable({ providedIn: 'root' })
export class PersistenceService {
  private dbPromise: Promise<IDBDatabase | null> | null = null;
  private readonly memoryFallback = new Map<string, unknown>();

  async get<T>(store: PersistenceStore, key: string): Promise<T | undefined> {
    const db = await this.database();
    if (!db) return this.memoryFallback.get(`${store}:${key}`) as T | undefined;
    return new Promise<T | undefined>((resolve, reject) => {
      const request = db.transaction(store, 'readonly').objectStore(store).get(key);
      request.onsuccess = () => resolve(request.result as T | undefined);
      request.onerror = () => reject(request.error);
    });
  }

  async put<T>(store: PersistenceStore, key: string, value: T): Promise<void> {
    const db = await this.database();
    if (!db) {
      this.memoryFallback.set(`${store}:${key}`, value);
      return;
    }
    return new Promise<void>((resolve, reject) => {
      const transaction = db.transaction(store, 'readwrite');
      transaction.objectStore(store).put(value, key);
      transaction.oncomplete = () => resolve();
      transaction.onerror = () => reject(transaction.error);
    });
  }

  async quarantineLegacyLocalStorage(): Promise<void> {
    const mappings: Array<[string, PersistenceStore]> = [
      ['rvt-snaps', 'snapshots'],
      ['rvt-snap-notes', 'snap-notes'],
      ['rvt-session-notes', 'session-notes'],
      ['rvt-alert-history', 'alert-history'],
      ['rvt-alert-pins', 'settings-backup'],
      ['rvt-sandbox-sessions', 'sessions']
    ];
    for (const [key, store] of mappings) {
      const raw = localStorage.getItem(key);
      if (raw === null) continue;
      try {
        await this.put(store, 'legacy-unclassified', JSON.parse(raw));
        localStorage.removeItem(key);
      } catch (_) {
        // Keep unreadable legacy data intact so it is never silently discarded.
      }
    }
  }

  private database(): Promise<IDBDatabase | null> {
    if (this.dbPromise) return this.dbPromise;
    if (typeof indexedDB === 'undefined') {
      this.dbPromise = Promise.resolve(null);
      return this.dbPromise;
    }
    this.dbPromise = new Promise((resolve) => {
      const request = indexedDB.open(DB_NAME, DB_VERSION);
      request.onupgradeneeded = () => {
        const db = request.result;
        STORES.forEach(store => {
          if (!db.objectStoreNames.contains(store)) db.createObjectStore(store);
        });
      };
      request.onsuccess = () => resolve(request.result);
      request.onerror = () => resolve(null);
    });
    return this.dbPromise;
  }
}
