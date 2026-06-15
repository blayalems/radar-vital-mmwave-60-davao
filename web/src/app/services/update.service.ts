import { computed, inject, Injectable, signal } from '@angular/core';
import { MatSnackBar } from '@angular/material/snack-bar';

import { ApiService } from './api.service';
import { StateService } from './state.service';
import { SwUpdateService } from './sw-update.service';

export type UpdatePlatform = 'tauri' | 'capacitor' | 'pwa';
export type UpdateKind = 'product' | 'build';

export interface UpdateArtifact {
  url: string;
  size?: number;
  size_bytes?: number;
  sha256: string;
  compatibility: string;
  size_mb?: string;
}

export interface RvtUpdateManifest {
  product_version: string;
  release_tag?: string;
  release_version?: string;
  build_number?: number | null;
  minimum_supported?: string;
  released_at?: string;
  release_url?: string;
  artifacts?: {
    apk?: UpdateArtifact;
    exe?: UpdateArtifact;
  };
  tauri_updater?: {
    manifest_url?: string;
    signature_configured?: boolean;
  };
}

export type UpdateState =
  | { phase: 'idle' }
  | { phase: 'checking' }
  | { phase: 'up-to-date'; version: string }
  | { phase: 'available'; manifest: RvtUpdateManifest; kind: UpdateKind }
  | { phase: 'downloading'; manifest: RvtUpdateManifest; progress: number }
  | { phase: 'verifying'; manifest: RvtUpdateManifest }
  | { phase: 'ready-to-install'; manifest: RvtUpdateManifest; platform: UpdatePlatform }
  | { phase: 'installing'; manifest: RvtUpdateManifest; platform: UpdatePlatform }
  | { phase: 'installed'; platform: UpdatePlatform; message: string }
  | { phase: 'error'; message: string; manifest?: RvtUpdateManifest };

export interface UpdateResultView {
  ok: boolean;
  updateAvailable?: boolean;
  isNewBuild?: boolean;
  product_version?: string;
  release_tag?: string;
  release_version?: string;
  build_number?: number | null;
  minimum_supported?: string;
  released_at?: string;
  released_at_formatted?: string;
  artifacts?: {
    apk?: UpdateArtifact;
    exe?: UpdateArtifact;
  };
  error?: string;
}

@Injectable({ providedIn: 'root' })
export class UpdateService {
  private readonly api = inject(ApiService);
  private readonly stateService = inject(StateService);
  private readonly snackBar = inject(MatSnackBar);
  private readonly swUpdate = inject(SwUpdateService);

  readonly state = signal<UpdateState>({ phase: 'idle' });
  readonly result = computed<UpdateResultView | null>(() => this.toResultView(this.state()));
  readonly platform = computed<UpdatePlatform>(() => this.detectPlatform());
  readonly busy = computed(() => ['checking', 'downloading', 'verifying', 'installing'].includes(this.state().phase));
  readonly installDisabledReason = computed(() => {
    const state = this.state();
    const manifest = 'manifest' in state ? state.manifest : null;
    if (!manifest || !['available', 'ready-to-install', 'error'].includes(state.phase)) return '';
    const platform = this.platform();
    if (platform === 'tauri') {
      if (!manifest.artifacts?.exe?.url) return 'No Windows installer artifact is listed in the release manifest.';
      if (!manifest.tauri_updater?.signature_configured) {
        return 'Minisign updater signature is not configured for this build.';
      }
    }
    if (platform === 'capacitor' && !manifest.artifacts?.apk?.url) {
      return 'No Android APK artifact is listed in the release manifest.';
    }
    return '';
  });

  async checkForUpdates(currentProductVersion: string): Promise<void> {
    this.state.set({ phase: 'checking' });
    this.stateService.triggerHaptic('tap');
    try {
      const manifest = await this.api.request<RvtUpdateManifest | null>('/api/update/manifest');
      if (!manifest?.product_version) {
        throw new Error('Invalid manifest format returned.');
      }

      const normalized = this.normalizeManifest(manifest);
      const productUpdate = this.isNewerVersion(currentProductVersion, normalized.product_version);
      const releaseTag = normalized.release_tag || '';
      const releaseVersion = normalized.release_version || releaseTag.replace(/^v/, '');
      const currentStableTag = `v${currentProductVersion}`;
      const buildUpdate = !productUpdate
        && normalized.product_version === currentProductVersion
        && Boolean((releaseTag && releaseTag !== currentStableTag) || (releaseVersion && releaseVersion !== currentProductVersion));

      if (productUpdate || buildUpdate) {
        this.state.set({ phase: 'available', manifest: normalized, kind: productUpdate ? 'product' : 'build' });
        this.snackBar.open(
          buildUpdate ? `New build of ${currentProductVersion} is available.` : `New update ${normalized.product_version} is available.`,
          'Dismiss',
          { duration: 5000 }
        );
        this.stateService.triggerHaptic('success');
      } else {
        this.state.set({ phase: 'up-to-date', version: currentProductVersion });
        this.snackBar.open('Your dashboard is up to date.', 'Dismiss', { duration: 4000 });
        this.stateService.triggerHaptic('success');
      }
    } catch (error: unknown) {
      const message = error instanceof Error ? error.message : 'Offline or endpoint not found.';
      this.state.set({ phase: 'error', message });
      this.snackBar.open(`Update check failed: ${message}`, 'Dismiss', { duration: 5000 });
      this.stateService.triggerHaptic('reject');
    }
  }

  async installAvailable(): Promise<void> {
    const state = this.state();
    const manifest = 'manifest' in state ? state.manifest : null;
    if (!manifest) return;
    const reason = this.installDisabledReason();
    if (reason) {
      this.snackBar.open(reason, 'Dismiss', { duration: 6000 });
      return;
    }

    const platform = this.platform();
    try {
      if (platform === 'tauri') {
        await this.installTauri(manifest);
      } else if (platform === 'capacitor') {
        await this.installAndroid(manifest);
      } else {
        await this.installPwa();
      }
    } catch (error: unknown) {
      const message = error instanceof Error ? error.message : 'Update installation failed.';
      this.state.set({ phase: 'error', message, manifest });
      this.snackBar.open(message, 'Dismiss', { duration: 7000 });
      this.stateService.triggerHaptic('reject');
    }
  }

  installButtonLabel(): string {
    const platform = this.platform();
    const state = this.state();
    if (state.phase === 'downloading') return `Downloading ${Math.round(state.progress)}%`;
    if (state.phase === 'verifying') return 'Verifying...';
    if (state.phase === 'installing') return 'Installing...';
    if (platform === 'tauri') return 'Install Windows Update';
    if (platform === 'capacitor') return 'Install Android Update';
    return 'Reload & Apply';
  }

  private async installTauri(manifest: RvtUpdateManifest): Promise<void> {
    this.state.set({ phase: 'installing', manifest, platform: 'tauri' });
    const core = this.tauriCore();
    if (!core) throw new Error('Tauri IPC is not available in this shell.');
    const status = await core.invoke<string>('check_and_install_update');
    this.state.set({ phase: 'installed', platform: 'tauri', message: status === 'up-to-date' ? 'Windows app is already up to date.' : 'Windows update installer started.' });
    this.snackBar.open(status === 'up-to-date' ? 'Windows app is already up to date.' : 'Windows update installer started.', 'Dismiss', { duration: 5000 });
  }

  private async installAndroid(manifest: RvtUpdateManifest): Promise<void> {
    const apk = manifest.artifacts?.apk;
    if (!apk?.url) throw new Error('No Android APK artifact is listed in the release manifest.');
    const expectedHash = apk.sha256?.toLowerCase();
    if (!expectedHash) throw new Error('Release manifest does not include an APK SHA-256 hash.');

    this.state.set({ phase: 'downloading', manifest, progress: 0 });
    const response = await fetch(apk.url);
    if (!response.ok) throw new Error(`APK download failed with HTTP ${response.status}.`);
    const bytes = await this.readResponseBytes(response, apk.size_bytes ?? apk.size, progress => {
      this.state.set({ phase: 'downloading', manifest, progress });
    });

    this.state.set({ phase: 'verifying', manifest });
    const actualHash = await this.sha256Hex(bytes);
    if (actualHash !== expectedHash) {
      throw new Error('SHA-256 mismatch; download may be corrupted.');
    }

    this.state.set({ phase: 'ready-to-install', manifest, platform: 'capacitor' });
    const cap = this.capacitor();
    const filesystem = cap?.Plugins?.Filesystem;
    const openFile = cap?.Plugins?.OpenFile;
    if (!filesystem?.writeFile || !openFile?.openFile) {
      throw new Error('Native Android update plugins are not available.');
    }

    await filesystem.mkdir?.({ path: 'updates', directory: 'CACHE', recursive: true }).catch(() => undefined);
    const file = await filesystem.writeFile({
      path: `updates/rvt-update-${manifest.release_version || manifest.product_version}.apk`,
      data: this.bytesToBase64(bytes),
      directory: 'CACHE',
      recursive: true
    });

    this.state.set({ phase: 'installing', manifest, platform: 'capacitor' });
    await openFile.openFile({
      path: file?.uri,
      mimeType: 'application/vnd.android.package-archive'
    });
    this.state.set({ phase: 'installed', platform: 'capacitor', message: 'Android package installer opened.' });
    this.snackBar.open('Android package installer opened.', 'Dismiss', { duration: 5000 });
  }

  private async installPwa(): Promise<void> {
    const applied = await this.swUpdate.checkNow(true);
    if (!applied) {
      this.snackBar.open('No service worker update is ready yet. Try again after deployment finishes.', 'Dismiss', { duration: 6000 });
    }
  }

  private normalizeManifest(manifest: RvtUpdateManifest): RvtUpdateManifest {
    return {
      ...manifest,
      artifacts: {
        apk: manifest.artifacts?.apk ? this.normalizeArtifact(manifest.artifacts.apk) : undefined,
        exe: manifest.artifacts?.exe ? this.normalizeArtifact(manifest.artifacts.exe) : undefined
      }
    };
  }

  private normalizeArtifact(artifact: UpdateArtifact): UpdateArtifact {
    const sizeBytes = artifact.size_bytes ?? artifact.size ?? 0;
    return {
      ...artifact,
      size: sizeBytes,
      size_bytes: sizeBytes,
      size_mb: (sizeBytes / (1024 * 1024)).toFixed(2)
    };
  }

  private toResultView(state: UpdateState): UpdateResultView | null {
    if (state.phase === 'idle' || state.phase === 'checking') return null;
    if (state.phase === 'up-to-date') {
      return { ok: true, updateAvailable: false, product_version: state.version };
    }
    if (state.phase === 'error') {
      return { ok: false, error: state.message };
    }
    if (state.phase === 'installed') {
      return { ok: true, updateAvailable: false, product_version: state.message };
    }
    const manifest = state.manifest;
    const kind = state.phase === 'available' ? state.kind : undefined;
    return {
      ok: true,
      updateAvailable: true,
      isNewBuild: kind === 'build' || (manifest.product_version && manifest.release_tag ? manifest.release_tag !== `v${manifest.product_version}` : false),
      product_version: manifest.product_version,
      release_tag: manifest.release_tag,
      release_version: manifest.release_version,
      build_number: manifest.build_number,
      minimum_supported: manifest.minimum_supported,
      released_at: manifest.released_at,
      released_at_formatted: manifest.released_at ? new Date(manifest.released_at).toLocaleString() : 'Unknown',
      artifacts: manifest.artifacts
    };
  }

  private isNewerVersion(current: string, latest: string): boolean {
    return compareSemver(latest, current) > 0;
  }

  private async readResponseBytes(response: Response, expectedSize: number | undefined, onProgress: (progress: number) => void): Promise<Uint8Array> {
    if (!response.body?.getReader) {
      const buffer = await response.arrayBuffer();
      onProgress(100);
      return new Uint8Array(buffer);
    }
    const reader = response.body.getReader();
    const chunks: Uint8Array[] = [];
    let received = 0;
    while (true) {
      const { done, value } = await reader.read();
      if (done) break;
      if (!value) continue;
      chunks.push(value);
      received += value.byteLength;
      onProgress(expectedSize ? Math.min(100, (received / expectedSize) * 100) : 0);
    }
    const out = new Uint8Array(received);
    let offset = 0;
    for (const chunk of chunks) {
      out.set(chunk, offset);
      offset += chunk.byteLength;
    }
    onProgress(100);
    return out;
  }

  private async sha256Hex(bytes: Uint8Array): Promise<string> {
    const payload = new ArrayBuffer(bytes.byteLength);
    new Uint8Array(payload).set(bytes);
    const hash = await crypto.subtle.digest('SHA-256', payload);
    return Array.from(new Uint8Array(hash)).map(byte => byte.toString(16).padStart(2, '0')).join('');
  }

  private bytesToBase64(bytes: Uint8Array): string {
    let binary = '';
    const chunkSize = 0x8000;
    for (let i = 0; i < bytes.length; i += chunkSize) {
      binary += String.fromCharCode(...bytes.subarray(i, i + chunkSize));
    }
    return btoa(binary);
  }

  private detectPlatform(): UpdatePlatform {
    const userAgent = typeof navigator === 'undefined' ? '' : navigator.userAgent || '';
    if (this.tauriCore() && !/android|iphone|ipad|ipod/i.test(userAgent)) return 'tauri';
    if (this.capacitor()?.isNativePlatform?.()) return 'capacitor';
    return 'pwa';
  }

  private tauriCore(): { invoke<T = unknown>(command: string, args?: Record<string, unknown>): Promise<T> } | null {
    if (typeof window === 'undefined') return null;
    return (window as Window & { __TAURI__?: { core?: { invoke<T = unknown>(command: string, args?: Record<string, unknown>): Promise<T> } } }).__TAURI__?.core || null;
  }

  private capacitor(): any {
    if (typeof window === 'undefined') return null;
    return (window as any).Capacitor || null;
  }
}

/**
 * Compare two version strings by SemVer 2.0.0 precedence.
 * Returns >0 if `a` is newer than `b`, <0 if older, 0 if equal precedence.
 *
 * Fixes the previous comparator, which discarded everything after the first
 * `-`/`+` (so `16.3.0` and `16.3.0-rc1` compared equal). Build metadata (`+…`)
 * is ignored per spec; a release outranks a prerelease of the same core; and
 * prerelease identifiers compare dot-by-dot (numeric < alphanumeric, numeric
 * compared numerically, a larger set of identifiers outranks a prefix subset).
 */
export function compareSemver(a: string, b: string): number {
  const parse = (v: string) => {
    const cleaned = String(v ?? '').trim().replace(/^v/i, '').split('+')[0];
    const dash = cleaned.indexOf('-');
    const core = dash === -1 ? cleaned : cleaned.slice(0, dash);
    const pre = dash === -1 ? '' : cleaned.slice(dash + 1);
    const nums = core.split('.').map(p => (Number.isFinite(Number(p)) ? Number(p) : 0));
    while (nums.length < 3) nums.push(0);
    return { nums, pre };
  };
  const pa = parse(a);
  const pb = parse(b);
  for (let i = 0; i < 3; i++) {
    if (pa.nums[i] !== pb.nums[i]) return pa.nums[i] > pb.nums[i] ? 1 : -1;
  }
  // Equal numeric core: a release (no prerelease) has higher precedence than a prerelease.
  if (!pa.pre && !pb.pre) return 0;
  if (!pa.pre) return 1;
  if (!pb.pre) return -1;
  const ida = pa.pre.split('.');
  const idb = pb.pre.split('.');
  for (let i = 0; i < Math.max(ida.length, idb.length); i++) {
    if (i >= ida.length) return -1; // fewer identifiers → lower precedence
    if (i >= idb.length) return 1;
    const x = ida[i];
    const y = idb[i];
    const xNum = /^\d+$/.test(x);
    const yNum = /^\d+$/.test(y);
    if (xNum && yNum) {
      const nx = Number(x), ny = Number(y);
      if (nx !== ny) return nx > ny ? 1 : -1;
    } else if (xNum !== yNum) {
      return xNum ? -1 : 1; // numeric identifiers rank below alphanumeric
    } else if (x !== y) {
      return x > y ? 1 : -1;
    }
  }
  return 0;
}
