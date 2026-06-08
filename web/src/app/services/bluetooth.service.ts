import { Injectable } from '@angular/core';

export interface BleDevice {
  id: string;
  name?: string;
  gatt?: any;
  nativeHandle?: any;
}

export interface BleReferenceProbeResult {
  device: BleDevice;
  notificationBytes: number | null;
}

@Injectable({
  providedIn: 'root'
})
export class BluetoothService {
  private readonly ailinkServiceUuid = '0000ffe0-0000-1000-8000-00805f9b34fb';
  private readonly ailinkNotifyUuid = '0000ffe2-0000-1000-8000-00805f9b34fb';
  private activeDevice: BleDevice | null = null;
  private notificationCleanup: (() => void) | null = null;

  isSupported(): boolean {
    if (typeof window === 'undefined') return false;
    const cap = (window as any).Capacitor;
    if (cap?.isNativePlatform?.()) return true;
    if ((window as any).__TAURI_INTERNALS__ || (window as any).__TAURI__) return true;
    return 'bluetooth' in navigator;
  }

  async requestDevice(): Promise<BleDevice> {
    if (!this.isSupported()) {
      throw new Error('Bluetooth is not supported on this platform');
    }

    const cap = (window as any).Capacitor;
    if (cap?.isNativePlatform?.()) {
      return this.requestDeviceCapacitor();
    }

    if ((window as any).__TAURI_INTERNALS__ || (window as any).__TAURI__) {
      return this.requestDeviceTauri();
    }

    return this.requestDeviceWebBluetooth();
  }

  private async requestDeviceWebBluetooth(): Promise<BleDevice> {
    const filters = [{ services: [this.ailinkServiceUuid] }];
    const dev = await (navigator as any).bluetooth.requestDevice({
      filters,
      optionalServices: [this.ailinkServiceUuid]
    });
    return {
      id: dev.id,
      name: dev.name,
      gatt: dev.gatt,
      nativeHandle: dev
    };
  }

  private async requestDeviceCapacitor(): Promise<BleDevice> {
    const ble = (window as any).Capacitor?.Plugins?.BluetoothLe || (window as any).BluetoothLe;
    if (!ble) throw new Error('Capacitor BluetoothLe plugin unavailable');
    
    await ble.initialize();
    const result = await ble.requestDevice({
      services: [this.ailinkServiceUuid]
    });
    
    return {
      id: result.deviceId,
      name: result.name,
      nativeHandle: result
    };
  }

  private async requestDeviceTauri(): Promise<BleDevice> {
    const devices = await this.tauriInvoke<Array<{ id: string; name?: string }>>('native_ble_scan', { timeoutMs: 8000 });
    const selected = devices[0];
    if (!selected) throw new Error('No Radar Vital BLE device found by the Windows native scanner. Make sure Bluetooth is enabled and the device is advertising, then scan again.');
    return { id: selected.id, name: selected.name, nativeHandle: selected };
  }

  async connect(device: BleDevice): Promise<any> {
    this.activeDevice = device;
    if (device.gatt) {
      return device.gatt.connect();
    }
    // Capacitor / Tauri BLE
    const cap = (window as any).Capacitor;
    if (cap?.isNativePlatform?.()) {
      const ble = cap.Plugins?.BluetoothLe || (window as any).BluetoothLe;
      return ble.connect({ deviceId: device.id });
    }
    if (this.isTauri()) {
      return this.tauriInvoke('native_ble_connect', { deviceId: device.id });
    }
    return Promise.resolve();
  }

  async disconnect() {
    if (!this.activeDevice) {
      this.clearNotificationListener();
      return;
    }
    const device = this.activeDevice;
    this.activeDevice = null;

    try {
      if (device.gatt) {
        device.gatt.disconnect();
        return;
      }
      const cap = (window as any).Capacitor;
      if (cap?.isNativePlatform?.()) {
        const ble = cap.Plugins?.BluetoothLe || (window as any).BluetoothLe;
        await ble.disconnect({ deviceId: device.id });
        return;
      }
      if (this.isTauri()) {
        await this.tauriInvoke('native_ble_disconnect', { deviceId: device.id });
      }
    } catch (_) {
    } finally {
      this.clearNotificationListener();
    }
  }

  async startNotifications(svcUUID: string, charUUID: string, callback: (data: DataView) => void) {
    if (!this.activeDevice) throw new Error('No active device connected');
    const profile = this.requireNotificationProfile(svcUUID, charUUID);
    const device = this.activeDevice;

    if (device.gatt) {
      const server = device.gatt.connected ? device.gatt : await device.gatt.connect();
      const service = await server.getPrimaryService(profile.service);
      const characteristic = await service.getCharacteristic(profile.characteristic);
      
      characteristic.addEventListener('characteristicvaluechanged', (ev: any) => {
        callback(ev.target.value);
      });
      await characteristic.startNotifications();
      return characteristic;
    }

    const cap = (window as any).Capacitor;
    if (cap?.isNativePlatform?.()) {
      const ble = cap.Plugins?.BluetoothLe || (window as any).BluetoothLe;
      await ble.startNotifications({
        deviceId: device.id,
        service: profile.service,
        characteristic: profile.characteristic
      }, (value: any) => {
        // value.value is base64 encoded
        if (value?.value) {
          const raw = atob(value.value);
          const bytes = new Uint8Array(raw.length);
          for (let i = 0; i < raw.length; i++) {
            bytes[i] = raw.charCodeAt(i);
          }
          callback(new DataView(bytes.buffer));
        }
      });
      return;
    }

    if (this.isTauri()) {
      const tauri = (window as any).__TAURI__;
      const unlisten = await tauri?.event?.listen?.('rvt-ble-notification', (event: any) => {
        if (event?.payload?.device_id !== device.id || !event?.payload?.data_base64) return;
        const raw = atob(event.payload.data_base64);
        const bytes = Uint8Array.from(raw, char => char.charCodeAt(0));
        callback(new DataView(bytes.buffer));
      });
      if (typeof unlisten === 'function') {
        this.notificationCleanup = unlisten;
      }
      try {
        await this.tauriInvoke('native_ble_start_notifications', {
          deviceId: device.id,
          serviceUuid: profile.service,
          characteristicUuid: profile.characteristic
        });
      } catch (error) {
        this.clearNotificationListener();
        throw error;
      }
    }
  }

  async validateReferenceNotification(timeoutMs = 5000): Promise<BleReferenceProbeResult> {
    const device = await this.requestDevice();
    await this.connect(device);
    try {
      return await new Promise<BleReferenceProbeResult>((resolve, reject) => {
        let settled = false;
        let timer: ReturnType<typeof setTimeout>;
        const complete = (notificationBytes: number | null) => {
          if (settled) return;
          settled = true;
          clearTimeout(timer);
          resolve({ device, notificationBytes });
        };
        timer = setTimeout(() => complete(null), timeoutMs);

        this.startNotifications(this.ailinkServiceUuid, this.ailinkNotifyUuid, value => {
          complete(value.byteLength);
        }).catch(error => {
          clearTimeout(timer);
          reject(error);
        });
      });
    } finally {
      await this.disconnect();
    }
  }

  private clearNotificationListener(): void {
    this.notificationCleanup?.();
    this.notificationCleanup = null;
  }

  private requireNotificationProfile(serviceUuid: string, characteristicUuid: string): { service: string; characteristic: string } {
    const service = this.canonicalBluetoothUuid(serviceUuid);
    const characteristic = this.canonicalBluetoothUuid(characteristicUuid);
    if (service !== this.ailinkServiceUuid || characteristic !== this.ailinkNotifyUuid) {
      throw new Error('Only the configured AiLink notification profile is permitted.');
    }
    return { service, characteristic };
  }

  private canonicalBluetoothUuid(value: string): string {
    const normalized = value.trim().toLowerCase();
    const shortUuid = normalized.startsWith('0x') ? normalized.slice(2) : normalized;
    return /^[0-9a-f]{4}$/.test(shortUuid)
      ? `0000${shortUuid}-0000-1000-8000-00805f9b34fb`
      : normalized;
  }

  private isTauri(): boolean {
    return !!((window as any).__TAURI_INTERNALS__ || (window as any).__TAURI__);
  }

  private tauriInvoke<T = unknown>(command: string, args: Record<string, unknown>): Promise<T> {
    const invoke = (window as any).__TAURI__?.core?.invoke;
    if (!invoke) throw new Error('Tauri native BLE bridge unavailable.');
    return invoke(command, args) as Promise<T>;
  }
}
