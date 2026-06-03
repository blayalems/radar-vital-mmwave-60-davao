#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::{collections::HashMap, sync::Mutex, time::Duration};

use base64::{engine::general_purpose::STANDARD, Engine as _};
use btleplug::{
    api::{Central, Manager as _, Peripheral as _, ScanFilter},
    platform::{Manager as BleManager, Peripheral},
};
use futures_util::StreamExt;
use serde::{Deserialize, Serialize};
use tauri::{AppHandle, Emitter, State};
use url::Url;
use uuid::Uuid;

const AILINK_SERVICE_UUID: &str = "0000ffe0-0000-1000-8000-00805f9b34fb";
const AILINK_NOTIFY_UUID: &str = "0000ffe2-0000-1000-8000-00805f9b34fb";

#[derive(Default)]
struct NativeState {
    paired_origin: Mutex<Option<String>>,
    active_ble_device: Mutex<Option<String>>,
}

#[derive(Deserialize)]
struct NativeRequest {
    origin: String,
    path: String,
    method: String,
    headers: HashMap<String, String>,
    body: Option<String>,
}

#[derive(Serialize)]
struct NativeResponse {
    status: u16,
    data: serde_json::Value,
}

#[derive(Serialize)]
struct NativeDownload {
    status: u16,
    body_base64: String,
    content_type: String,
}

#[derive(Serialize)]
struct NativeBleDevice {
    id: String,
    name: String,
    address: String,
    rssi: Option<i16>,
}

#[derive(Clone, Serialize)]
struct NativeBleNotification {
    device_id: String,
    data_base64: String,
}

fn validate_origin(origin: &str) -> Result<String, String> {
    let url = Url::parse(origin).map_err(|_| "Invalid trainer origin.".to_string())?;
    if !matches!(url.scheme(), "http" | "https")
        || url.host_str().is_none()
        || !url.username().is_empty()
        || url.password().is_some()
        || url.path() != "/"
        || url.query().is_some()
        || url.fragment().is_some()
    {
        return Err("Trainer origin must be a plain HTTP(S) origin.".to_string());
    }
    Ok(url.origin().ascii_serialization())
}

fn ailink_service_uuid() -> Result<Uuid, String> {
    Uuid::parse_str(AILINK_SERVICE_UUID).map_err(|error| error.to_string())
}

fn allowed_notification_profile(service_uuid: &str, characteristic_uuid: &str) -> Result<(Uuid, Uuid), String> {
    let service = Uuid::parse_str(service_uuid).map_err(|_| "Invalid BLE service UUID.".to_string())?;
    let characteristic = Uuid::parse_str(characteristic_uuid).map_err(|_| "Invalid BLE characteristic UUID.".to_string())?;
    let approved_service = ailink_service_uuid()?;
    let approved_characteristic = Uuid::parse_str(AILINK_NOTIFY_UUID).map_err(|error| error.to_string())?;
    if service != approved_service || characteristic != approved_characteristic {
        return Err("Native BLE permits only the configured AiLink notification profile.".to_string());
    }
    Ok((service, characteristic))
}

fn require_active_ble_device(device_id: &str, state: &State<'_, NativeState>) -> Result<(), String> {
    let active = state.active_ble_device.lock().map_err(|_| "Native BLE state is unavailable.".to_string())?;
    if active.as_deref() != Some(device_id) {
        return Err("Connect to the approved BLE device before using this command.".to_string());
    }
    Ok(())
}

fn target_url(request: &NativeRequest, paired_origin: Option<&str>) -> Result<String, String> {
    let origin = validate_origin(&request.origin)?;
    if let Some(allowed) = paired_origin {
        if origin != allowed {
            return Err("Request origin does not match the paired trainer.".to_string());
        }
    }
    if !request.path.starts_with("/api/") {
        return Err("Native transport permits trainer API routes only.".to_string());
    }
    Ok(format!("{origin}{}", request.path))
}

async fn perform_request(request: &NativeRequest, paired_origin: Option<&str>) -> Result<reqwest::Response, String> {
    let url = target_url(request, paired_origin)?;
    let method = reqwest::Method::from_bytes(request.method.as_bytes()).map_err(|_| "Invalid HTTP method.".to_string())?;
    let client = reqwest::Client::builder().timeout(Duration::from_secs(15)).build().map_err(|error| error.to_string())?;
    let mut outgoing = client.request(method, url);
    for (name, value) in &request.headers {
        outgoing = outgoing.header(name, value);
    }
    if let Some(body) = &request.body {
        outgoing = outgoing.body(body.clone());
    }
    outgoing.send().await.map_err(|error| error.to_string())
}

#[tauri::command]
fn native_set_paired_origin(origin: String, state: State<'_, NativeState>) -> Result<(), String> {
    let normalized = validate_origin(&origin)?;
    *state.paired_origin.lock().map_err(|_| "Native origin state is unavailable.".to_string())? = Some(normalized);
    Ok(())
}

#[tauri::command]
async fn native_pair_request(request: NativeRequest) -> Result<NativeResponse, String> {
    let route = request.path.split('?').next().unwrap_or("");
    if !matches!(route, "/api/server-info" | "/api/auth/exchange") {
        return Err("Only pairing bootstrap routes are public to native transport.".to_string());
    }
    let response = perform_request(&request, None).await?;
    let status = response.status().as_u16();
    let data = response.json::<serde_json::Value>().await.map_err(|error| error.to_string())?;
    Ok(NativeResponse { status, data })
}

#[tauri::command]
async fn native_http_request(request: NativeRequest, state: State<'_, NativeState>) -> Result<NativeResponse, String> {
    let paired = state.paired_origin.lock().map_err(|_| "Native origin state is unavailable.".to_string())?.clone();
    let paired = paired.ok_or_else(|| "Pair with a trainer before making native API requests.".to_string())?;
    let response = perform_request(&request, Some(&paired)).await?;
    let status = response.status().as_u16();
    let data = response.json::<serde_json::Value>().await.map_err(|error| error.to_string())?;
    Ok(NativeResponse { status, data })
}

#[tauri::command]
async fn native_download(request: NativeRequest, state: State<'_, NativeState>) -> Result<NativeDownload, String> {
    let paired = state.paired_origin.lock().map_err(|_| "Native origin state is unavailable.".to_string())?.clone();
    let paired = paired.ok_or_else(|| "Pair with a trainer before downloading artifacts.".to_string())?;
    let response = perform_request(&request, Some(&paired)).await?;
    let status = response.status().as_u16();
    let content_type = response.headers().get(reqwest::header::CONTENT_TYPE)
        .and_then(|header| header.to_str().ok()).unwrap_or("application/octet-stream").to_string();
    let bytes = response.bytes().await.map_err(|error| error.to_string())?;
    Ok(NativeDownload { status, body_base64: STANDARD.encode(bytes), content_type })
}

async fn first_adapter() -> Result<btleplug::platform::Adapter, String> {
    let manager = BleManager::new().await.map_err(|error| error.to_string())?;
    manager.adapters().await.map_err(|error| error.to_string())?
        .into_iter().next().ok_or_else(|| "No Bluetooth adapter was found.".to_string())
}

async fn find_peripheral(device_id: &str) -> Result<Peripheral, String> {
    let adapter = first_adapter().await?;
    adapter.peripherals().await.map_err(|error| error.to_string())?
        .into_iter()
        .find(|peripheral| peripheral.id().to_string() == device_id)
        .ok_or_else(|| "BLE device is no longer available; scan again.".to_string())
}

#[tauri::command]
async fn native_ble_scan(timeout_ms: Option<u64>) -> Result<Vec<NativeBleDevice>, String> {
    let adapter = first_adapter().await?;
    let service_uuid = ailink_service_uuid()?;
    adapter.start_scan(ScanFilter {
        services: vec![service_uuid],
    }).await.map_err(|error| error.to_string())?;
    tokio::time::sleep(Duration::from_millis(timeout_ms.unwrap_or(3000).min(10_000))).await;
    let mut devices = Vec::new();
    for peripheral in adapter.peripherals().await.map_err(|error| error.to_string())? {
        if let Some(properties) = peripheral.properties().await.map_err(|error| error.to_string())? {
            if !properties.services.contains(&service_uuid) {
                continue;
            }
            devices.push(NativeBleDevice {
                id: peripheral.id().to_string(),
                name: properties.local_name.unwrap_or_else(|| "Radar Vital BLE device".to_string()),
                address: properties.address.to_string(),
                rssi: properties.rssi,
            });
        }
    }
    Ok(devices)
}

#[tauri::command]
async fn native_ble_connect(device_id: String, state: State<'_, NativeState>) -> Result<(), String> {
    let peripheral = find_peripheral(&device_id).await?;
    if !peripheral.is_connected().await.map_err(|error| error.to_string())? {
        peripheral.connect().await.map_err(|error| error.to_string())?;
    }
    peripheral.discover_services().await.map_err(|error| error.to_string())?;
    let service_uuid = ailink_service_uuid()?;
    if !peripheral.services().iter().any(|service| service.uuid == service_uuid) {
        let _ = peripheral.disconnect().await;
        return Err("Connected device does not expose the approved AiLink BLE service.".to_string());
    }
    *state.active_ble_device.lock().map_err(|_| "Native BLE state is unavailable.".to_string())? = Some(device_id);
    Ok(())
}

#[tauri::command]
async fn native_ble_disconnect(device_id: String, state: State<'_, NativeState>) -> Result<(), String> {
    require_active_ble_device(&device_id, &state)?;
    find_peripheral(&device_id).await?.disconnect().await.map_err(|error| error.to_string())?;
    *state.active_ble_device.lock().map_err(|_| "Native BLE state is unavailable.".to_string())? = None;
    Ok(())
}

#[tauri::command]
async fn native_ble_start_notifications(
    app: AppHandle,
    state: State<'_, NativeState>,
    device_id: String,
    service_uuid: String,
    characteristic_uuid: String,
) -> Result<(), String> {
    require_active_ble_device(&device_id, &state)?;
    let peripheral = find_peripheral(&device_id).await?;
    let (service, characteristic) = allowed_notification_profile(&service_uuid, &characteristic_uuid)?;
    let target = peripheral.characteristics().into_iter()
        .find(|item| item.service_uuid == service && item.uuid == characteristic)
        .ok_or_else(|| "Requested BLE notification characteristic was not found.".to_string())?;
    peripheral.subscribe(&target).await.map_err(|error| error.to_string())?;
    let mut notifications = peripheral.notifications().await.map_err(|error| error.to_string())?;
    tauri::async_runtime::spawn(async move {
        while let Some(notification) = notifications.next().await {
            let _ = app.emit("rvt-ble-notification", NativeBleNotification {
                device_id: device_id.clone(),
                data_base64: STANDARD.encode(notification.value),
            });
        }
    });
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn request(origin: &str, path: &str) -> NativeRequest {
        NativeRequest {
            origin: origin.to_string(),
            path: path.to_string(),
            method: "GET".to_string(),
            headers: HashMap::new(),
            body: None,
        }
    }

    #[test]
    fn permits_only_the_ailink_notification_profile() {
        assert!(allowed_notification_profile(AILINK_SERVICE_UUID, AILINK_NOTIFY_UUID).is_ok());
        assert!(allowed_notification_profile(AILINK_SERVICE_UUID, "0000ffe1-0000-1000-8000-00805f9b34fb").is_err());
        assert!(allowed_notification_profile("8b1d0000-7d4c-4a3f-9a2b-7f2d4c8b1000", AILINK_NOTIFY_UUID).is_err());
    }

    #[test]
    fn validates_plain_http_origins_only() {
        assert_eq!(
            validate_origin("http://192.168.1.50:8765").unwrap(),
            "http://192.168.1.50:8765"
        );
        assert_eq!(
            validate_origin("https://trainer.local:8765/").unwrap(),
            "https://trainer.local:8765"
        );

        for origin in [
            "file://trainer",
            "ftp://trainer.local",
            "http://user@trainer.local:8765",
            "http://trainer.local:8765/path",
            "http://trainer.local:8765?token=secret",
            "http://trainer.local:8765/#fragment",
        ] {
            assert!(validate_origin(origin).is_err(), "{origin} should be rejected");
        }
    }

    #[test]
    fn pins_native_requests_to_the_paired_origin_and_api_routes() {
        let paired = "http://192.168.1.50:8765";
        assert_eq!(
            target_url(&request(paired, "/api/status"), Some(paired)).unwrap(),
            "http://192.168.1.50:8765/api/status"
        );

        assert!(target_url(&request("http://192.168.1.51:8765", "/api/status"), Some(paired))
            .unwrap_err()
            .contains("paired trainer"));
        assert!(target_url(&request(paired, "/manifest.webmanifest"), Some(paired))
            .unwrap_err()
            .contains("API routes only"));
    }
}

fn main() {
    tauri::Builder::default()
        .manage(NativeState::default())
        .invoke_handler(tauri::generate_handler![
            native_set_paired_origin,
            native_pair_request,
            native_http_request,
            native_download,
            native_ble_scan,
            native_ble_connect,
            native_ble_disconnect,
            native_ble_start_notifications,
        ])
        .run(tauri::generate_context!())
        .expect("error while running radar-vital");
}
