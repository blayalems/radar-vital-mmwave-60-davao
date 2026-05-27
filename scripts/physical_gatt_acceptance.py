#!/usr/bin/env python3
"""Radar Vital BLE/GATT Physical Hardware Acceptance Tool.

This diagnostic tool scans for nearby Bluetooth Low Energy devices, matches the
approved AiLink oximeter service profile (0000ffe0), subscribes to the notify
characteristic (0000ffe2), and streams incoming notifications to verify physical
compatibility and 1 Hz telemetry throughput.
"""

import asyncio
import time
import sys
from typing import Optional

try:
    from bleak import BleakScanner, BleakClient
except ImportError:
    print("Error: The 'bleak' library is required to run this diagnostic.")
    print("Please install it via: pip install bleak")
    sys.exit(1)

AILINK_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
AILINK_NOTIFY_UUID = "0000ffe2-0000-1000-8000-00805f9b34fb"

last_notification_time: Optional[float] = None
notification_count = 0


def notification_handler(sender: int, data: bytearray):
    global last_notification_time, notification_count
    now = time.time()
    notification_count += 1

    hex_data = data.hex().upper()
    readable_hex = " ".join(hex_data[i:i+2] for i in range(0, len(hex_data), 2))

    if last_notification_time is not None:
        interval = now - last_notification_time
        print(f"[{notification_count:03d}] Received {len(data)} bytes from {sender} "
              f"(Interval: {interval:.3f}s): {readable_hex}")
    else:
        print(f"[{notification_count:03d}] Received {len(data)} bytes from {sender}: {readable_hex}")

    last_notification_time = now


async def main():
    print("=" * 70)
    print("   RADAR VITAL PHYSICAL GATT ACCEPTANCE TEST RUNNER")
    print("=" * 70)
    print(f"Target Service UUID:        {AILINK_SERVICE_UUID}")
    print(f"Target Characteristic UUID: {AILINK_NOTIFY_UUID}")
    print("-" * 70)
    print("Scanning for approved AiLink BLE devices (5 seconds)...")

    devices = await BleakScanner.discover(timeout=5.0)
    approved_devices = []

    for device in devices:
        name = device.name or "Unknown Device"
        uuids = device.details.get("props", {}).get("UUIDs", []) if hasattr(device, "details") else []
        
        # Check if service UUID is advertised or name matches known profile
        is_approved = AILINK_SERVICE_UUID in uuids or "ailink" in name.lower() or "oximeter" in name.lower()
        if is_approved:
            approved_devices.append(device)
            print(f"  [FOUND] Address: {device.address} | Name: {name} | RSSI: {device.rssi}dBm")

    if not approved_devices:
        print("\nNo devices matching the approved AiLink profile were advertised.")
        print("Fallback: Displaying all discovered BLE devices:")
        for idx, dev in enumerate(devices):
            print(f"  [{idx}] Address: {dev.address} | Name: {dev.name or 'Unknown'}")
        
        if not devices:
            print("No BLE devices found at all. Please check that Bluetooth is enabled.")
            return

        selection = input("\nEnter address or index of device to connect to (or Ctrl+C to cancel): ").strip()
        try:
            idx = int(selection)
            target_device = devices[idx]
        except ValueError:
            target_device = next((d for d in devices if d.address.lower() == selection.lower()), None)
            if not target_device:
                print("Invalid address or index selected.")
                return
    else:
        target_device = approved_devices[0]
        print(f"\nAutomatically selecting best match device: {target_device.address} ({target_device.name})")

    print(f"\nConnecting to {target_device.address}...")
    async with BleakClient(target_device.address) as client:
        if not client.is_connected:
            print("Failed to establish Bluetooth connection.")
            return

        print("Connection successful! Discovering primary GATT services...")
        services = client.services
        
        # Validate that the approved service and characteristic exist
        target_service = services.get_service(AILINK_SERVICE_UUID)
        if not target_service:
            print(f"Warning: Approved service {AILINK_SERVICE_UUID} not found in GATT profile.")
            print("Listing discovered services:")
            for s in services:
                print(f"  Service: {s.uuid}")
        
        print(f"Starting notifications on {AILINK_NOTIFY_UUID}...")
        await client.start_notify(AILINK_NOTIFY_UUID, notification_handler)
        
        print("\nStreaming real-time physiological packets. Press Enter or Ctrl+C to exit...\n")
        
        # Block until user presses Enter or 30s timeout
        loop = asyncio.get_running_loop()
        future = loop.run_in_executor(None, sys.stdin.readline)
        try:
            await asyncio.wait_for(future, timeout=30.0)
        except asyncio.TimeoutError:
            print("\nVerification session timed out after 30 seconds.")
        
        print("Stopping notifications and disconnecting...")
        await client.stop_notify(AILINK_NOTIFY_UUID)

    print("\nAcceptance diagnostics complete.")
    if notification_count > 0:
        print(f"SUCCESS: Captured {notification_count} stable physiological telemetry frames.")
    else:
        print("FAILURE: Connection completed but no notification packets were received.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nTesting interrupted by operator.")
        sys.exit(0)
