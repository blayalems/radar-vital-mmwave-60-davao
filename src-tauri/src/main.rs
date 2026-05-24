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

#[derive(Default)]
struct NativeState {
    paired_origin: Mutex<Option<String>>,
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
    adapter.start_scan(ScanFilter {
        services: vec![Uuid::parse_str("0000ffe0-0000-1000-8000-00805f9b34fb").map_err(|error| error.to_string())?],
    }).await.map_err(|error| error.to_string())?;
    tokio::time::sleep(Duration::from_millis(timeout_ms.unwrap_or(3000).min(10_000))).await;
    let mut devices = Vec::new();
    for peripheral in adapter.peripherals().await.map_err(|error| error.to_string())? {
        if let Some(properties) = peripheral.properties().await.map_err(|error| error.to_string())? {
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
async fn native_ble_connect(device_id: String) -> Result<(), String> {
    let peripheral = find_peripheral(&device_id).await?;
    if !peripheral.is_connected().await.map_err(|error| error.to_string())? {
        peripheral.connect().await.map_err(|error| error.to_string())?;
    }
    peripheral.discover_services().await.map_err(|error| error.to_string())
}

#[tauri::command]
async fn native_ble_disconnect(device_id: String) -> Result<(), String> {
    find_peripheral(&device_id).await?.disconnect().await.map_err(|error| error.to_string())
}

#[tauri::command]
async fn native_ble_start_notifications(
    app: AppHandle,
    device_id: String,
    service_uuid: String,
    characteristic_uuid: String,
) -> Result<(), String> {
    let peripheral = find_peripheral(&device_id).await?;
    let service = Uuid::parse_str(&service_uuid).map_err(|error| error.to_string())?;
    let characteristic = Uuid::parse_str(&characteristic_uuid).map_err(|error| error.to_string())?;
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
