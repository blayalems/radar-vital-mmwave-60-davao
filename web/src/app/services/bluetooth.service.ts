import { Injectable } from '@angular/core';

export interface BleDevice {
  id: string;
  name?: string;
  gatt?: any;
  nativeHandle?: any;
}

@Injectable({
  providedIn: 'root'
})
export class BluetoothService {
  private activeDevice: BleDevice | null = null;

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
    const filters = [{ services: [0xffe0] }]; // AiLink standard Service UUID filter
    const dev = await (navigator as any).bluetooth.requestDevice({
      filters,
      optionalServices: [0xffe0]
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
      services: ['0000ffe0-0000-1000-8000-00805f9b34fb']
    });
    
    return {
      id: result.deviceId,
      name: result.name,
      nativeHandle: result
    };
  }

  private async requestDeviceTauri(): Promise<BleDevice> {
    const devices = await this.tauriInvoke<Array<{ id: string; name?: string }>>('native_ble_scan', { timeoutMs: 3000 });
    const selected = devices[0];
    if (!selected) throw new Error('No Radar Vital BLE device found.');
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
    if (!this.activeDevice) return;
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
    } catch (_) {}
  }

  async startNotifications(svcUUID: string, charUUID: string, callback: (data: DataView) => void) {
    if (!this.activeDevice) throw new Error('No active device connected');
    const device = this.activeDevice;

    if (device.gatt) {
      const server = device.gatt.connected ? device.gatt : await device.gatt.connect();
      const service = await server.getPrimaryService(svcUUID);
      const characteristic = await service.getCharacteristic(charUUID);
      
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
        service: svcUUID,
        characteristic: charUUID
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
      try {
        await this.tauriInvoke('native_ble_start_notifications', {
          deviceId: device.id,
          serviceUuid: svcUUID,
          characteristicUuid: charUUID
        });
      } catch (error) {
        if (typeof unlisten === 'function') unlisten();
        throw error;
      }
    }
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
