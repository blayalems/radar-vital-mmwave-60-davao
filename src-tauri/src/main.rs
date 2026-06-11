#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::{
    collections::{HashMap, VecDeque},
    fs::{create_dir_all, OpenOptions},
    io::Write,
    net::TcpListener,
    sync::Mutex,
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use base64::{engine::general_purpose::STANDARD, Engine as _};
use btleplug::{
    api::{Central, Manager as _, Peripheral as _, ScanFilter},
    platform::{Manager as BleManager, Peripheral},
};
use futures_util::StreamExt;
use serde::{Deserialize, Serialize};
use tauri::{AppHandle, Emitter, Manager, State, WindowEvent};
use tauri_plugin_shell::{process::{CommandChild, CommandEvent}, ShellExt};
use url::Url;
use uuid::Uuid;

const AILINK_SERVICE_UUID: &str = "0000ffe0-0000-1000-8000-00805f9b34fb";
const AILINK_NOTIFY_UUID: &str = "0000ffe2-0000-1000-8000-00805f9b34fb";
const LOCAL_TRAINER_HOST: &str = "127.0.0.1";
const LAN_TRAINER_PORT: u16 = 8765;
const TRAINER_LOG_TAIL_LINES: usize = 20;

#[derive(Default)]
struct NativeState {
    paired_origin: Mutex<Option<String>>,
    active_ble_device: Mutex<Option<String>>,
    trainer: Mutex<TrainerSidecarState>,
    trainer_log: Mutex<VecDeque<String>>,
}

struct TrainerSidecarState {
    origin: Option<String>,
    running: bool,
    ready: bool,
    error: Option<String>,
    child: Option<CommandChild>,
    child_pid: Option<u32>,
    stop_requested: bool,
    started_at_ms: Option<u128>,
    sessions_root: Option<String>,
    bind_mode: String,
}

impl Default for TrainerSidecarState {
    fn default() -> Self {
        Self {
            origin: None,
            running: false,
            ready: false,
            error: None,
            child: None,
            child_pid: None,
            stop_requested: false,
            started_at_ms: None,
            sessions_root: None,
            bind_mode: "local".to_string(),
        }
    }
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

#[derive(Clone, Serialize)]
struct TrainerSidecarStatus {
    enabled: bool,
    running: bool,
    ready: bool,
    origin: Option<String>,
    error: Option<String>,
    started_at_ms: Option<u128>,
    sessions_root: Option<String>,
    bind_mode: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
#[serde(tag = "state", content = "message", rename_all = "snake_case")]
enum TrainerLifecycleState {
    Starting,
    Running,
    Stopped,
    Error(String),
}

fn now_ms() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_else(|_| Duration::from_secs(0))
        .as_millis()
}

fn push_trainer_log(state: &NativeState, stream: &str, text: &str) {
    if let Ok(mut log) = state.trainer_log.lock() {
        for line in text.lines().map(str::trim).filter(|line| !line.is_empty()) {
            log.push_back(format!("[{stream}] {line}"));
            while log.len() > TRAINER_LOG_TAIL_LINES {
                log.pop_front();
            }
        }
    }
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

fn reserve_loopback_port() -> Result<u16, String> {
    let listener = TcpListener::bind((LOCAL_TRAINER_HOST, 0)).map_err(|error| format!("Could not reserve loopback port: {error}"))?;
    let port = listener.local_addr().map_err(|error| error.to_string())?.port();
    drop(listener);
    Ok(port)
}

fn trainer_bind_mode(bind_mode: Option<String>) -> Result<&'static str, String> {
    match bind_mode.as_deref().unwrap_or("local") {
        "local" => Ok("local"),
        "lan" => Ok("lan"),
        other => Err(format!("Unsupported trainer bind mode: {other}. Expected 'local' or 'lan'.")),
    }
}

fn trainer_sidecar_args<'a>(mode: &str, port_arg: &'a str, sessions_root_arg: &'a str) -> Vec<&'a str> {
    if mode == "lan" {
        vec![
            "serve",
            "--bind",
            "lan",
            "--port",
            port_arg,
            "--sessions-root",
            sessions_root_arg,
            "--no-browser",
        ]
    } else {
        vec![
            "serve",
            "--bind",
            "local",
            "--host",
            LOCAL_TRAINER_HOST,
            "--port",
            port_arg,
            "--sessions-root",
            sessions_root_arg,
            "--no-browser",
        ]
    }
}

fn ensure_lan_port_available() -> Result<(), String> {
    TcpListener::bind((LOCAL_TRAINER_HOST, LAN_TRAINER_PORT))
        .map(|listener| drop(listener))
        .map_err(|error| format!("Port {LAN_TRAINER_PORT} is already in use; stop the other trainer or choose local mode. {error}"))
}

fn local_trainer_origin(state: &State<'_, NativeState>) -> Option<String> {
    state.trainer.lock().ok().and_then(|trainer| trainer.origin.clone())
}

fn is_local_trainer_origin(origin: &str, state: &State<'_, NativeState>) -> bool {
    local_trainer_origin(state).as_deref() == Some(origin)
}

fn record_trainer_error(app: &AppHandle, message: &str) {
    let _ = app.emit("rvt-trainer-error", message.to_string());
    let state = app.state::<NativeState>();
    push_trainer_log(&state, "error", message);
    if let Ok(data_dir) = app.path().app_data_dir() {
        if create_dir_all(&data_dir).is_ok() {
            let log_path = data_dir.join("trainer-sidecar.log");
            if let Ok(mut file) = OpenOptions::new().create(true).append(true).open(log_path) {
                let _ = writeln!(file, "{} {}", now_ms(), message);
            }
        }
    }
}

fn trainer_lifecycle_from_state(trainer: &TrainerSidecarState) -> TrainerLifecycleState {
    if let Some(error) = &trainer.error {
        return TrainerLifecycleState::Error(error.clone());
    }
    if trainer.running && trainer.ready {
        TrainerLifecycleState::Running
    } else if trainer.running {
        TrainerLifecycleState::Starting
    } else {
        TrainerLifecycleState::Stopped
    }
}

fn trainer_lifecycle_status(state: &State<'_, NativeState>) -> TrainerLifecycleState {
    match state.trainer.lock() {
        Ok(trainer) => trainer_lifecycle_from_state(&trainer),
        Err(_) => TrainerLifecycleState::Error("Native trainer state is unavailable.".to_string()),
    }
}

#[cfg(target_os = "windows")]
fn terminate_process_tree(pid: u32) {
    let pid_arg = pid.to_string();
    let _ = std::process::Command::new("taskkill")
        .args(["/PID", pid_arg.as_str(), "/T", "/F"])
        .status();
}

#[cfg(not(target_os = "windows"))]
fn terminate_process_tree(_pid: u32) {}

async fn wait_for_trainer_health(origin: &str, timeout_s: u64) -> Result<(), String> {
    let client = reqwest::Client::builder()
        .timeout(Duration::from_millis(800))
        .build()
        .map_err(|error| error.to_string())?;
    let deadline = tokio::time::Instant::now() + Duration::from_secs(timeout_s);
    let health_url = format!("{origin}/api/health");
    loop {
        match client.get(&health_url).send().await {
            Ok(response) if response.status().is_success() => return Ok(()),
            _ if tokio::time::Instant::now() >= deadline => {
                return Err(format!("Bundled trainer did not answer /api/health within {timeout_s} seconds."));
            }
            _ => tokio::time::sleep(Duration::from_millis(400)).await,
        }
    }
}

async fn ensure_local_trainer_ready(state: &State<'_, NativeState>, origin: &str) -> Result<(), String> {
    if !is_local_trainer_origin(origin, state) {
        return Ok(());
    }
    let needs_wait = {
        let trainer = state.trainer.lock().map_err(|_| "Native trainer state is unavailable.".to_string())?;
        if !trainer.running {
            return Err(trainer.error.clone().unwrap_or_else(|| "Bundled trainer is not running.".to_string()));
        }
        !trainer.ready
    };
    if !needs_wait {
        return Ok(());
    }
    wait_for_trainer_health(origin, 25).await?;
    let mut trainer = state.trainer.lock().map_err(|_| "Native trainer state is unavailable.".to_string())?;
    if !trainer.running || trainer.origin.as_deref() != Some(origin) {
        return Err(trainer.error.clone().unwrap_or_else(|| "Bundled trainer terminated during startup.".to_string()));
    }
    trainer.ready = true;
    trainer.error = None;
    Ok(())
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

fn trainer_status_snapshot(state: &State<'_, NativeState>) -> TrainerSidecarStatus {
    match state.trainer.lock() {
        Ok(guard) => TrainerSidecarStatus {
            enabled: true,
            running: guard.running,
            ready: guard.ready,
            origin: guard.origin.clone(),
            error: guard.error.clone(),
            started_at_ms: guard.started_at_ms,
            sessions_root: guard.sessions_root.clone(),
            bind_mode: guard.bind_mode.clone(),
        },
        Err(_) => TrainerSidecarStatus {
            enabled: true,
            running: false,
            ready: false,
            origin: None,
            error: Some("Native trainer state is unavailable.".to_string()),
            started_at_ms: None,
            sessions_root: None,
            bind_mode: "local".to_string(),
        },
    }
}

fn shutdown_local_trainer(state: &State<'_, NativeState>) {
    if let Ok(mut trainer) = state.trainer.lock() {
        trainer.stop_requested = true;
        if let Some(pid) = trainer.child_pid.take() {
            terminate_process_tree(pid);
        }
        if let Some(child) = trainer.child.take() {
            let _ = child.kill();
        }
        trainer.running = false;
        trainer.ready = false;
        trainer.error = None;
    }
}

fn start_local_trainer_sidecar(bind_mode: Option<String>, app: &AppHandle, state: &State<'_, NativeState>) -> Result<(), String> {
    let mode = trainer_bind_mode(bind_mode)?;
    let (port, origin) = if mode == "lan" {
        ensure_lan_port_available()?;
        (LAN_TRAINER_PORT, format!("http://{LOCAL_TRAINER_HOST}:{LAN_TRAINER_PORT}"))
    } else {
        let p = reserve_loopback_port()?;
        (p, format!("http://{LOCAL_TRAINER_HOST}:{p}"))
    };
    let data_dir = app.path().app_data_dir().map_err(|error| format!("Could not resolve app data dir: {error}"))?;
    let sessions_root = data_dir.join("sessions");
    create_dir_all(&sessions_root).map_err(|error| format!("Could not create sessions dir: {error}"))?;
    let sessions_root_arg = sessions_root.to_string_lossy().to_string();
    let port_arg = port.to_string();

    let sidecar = app
        .shell()
        .sidecar("rvt-trainer")
        .map_err(|error| format!("Bundled trainer sidecar is not available: {error}"))?;

    let args = trainer_sidecar_args(mode, &port_arg, &sessions_root_arg);

    let (mut events, child) = sidecar
        .args(args)
        .spawn()
        .map_err(|error| format!("Failed to start bundled trainer: {error}"))?;
    let child_pid = child.pid();

    {
        let mut trainer = state.trainer.lock().map_err(|_| "Native trainer state is unavailable.".to_string())?;
        trainer.origin = Some(origin.clone());
        trainer.running = true;
        trainer.ready = false;
        trainer.error = None;
        trainer.stop_requested = false;
        trainer.started_at_ms = Some(now_ms());
        trainer.sessions_root = Some(sessions_root_arg);
        trainer.child_pid = Some(child_pid);
        trainer.child = Some(child);
        trainer.bind_mode = mode.to_string();
    }
    if let Ok(mut paired) = state.paired_origin.lock() {
        *paired = Some(origin.clone());
    }

    let app_handle = app.clone();
    tauri::async_runtime::spawn(async move {
        while let Some(event) = events.recv().await {
            match event {
                CommandEvent::Stdout(bytes) => {
                    let text = String::from_utf8_lossy(&bytes).to_string();
                    eprintln!("[RVT trainer stdout] {text}");
                    let state = app_handle.state::<NativeState>();
                    push_trainer_log(&state, "stdout", &text);
                    let _ = app_handle.emit("rvt-trainer-stdout", text);
                }
                CommandEvent::Stderr(bytes) => {
                    let text = String::from_utf8_lossy(&bytes).to_string();
                    eprintln!("[RVT trainer stderr] {text}");
                    let state = app_handle.state::<NativeState>();
                    push_trainer_log(&state, "stderr", &text);
                    let _ = app_handle.emit("rvt-trainer-stderr", text.clone());
                    if text.to_ascii_lowercase().contains("error") || text.to_ascii_lowercase().contains("traceback") {
                        record_trainer_error(&app_handle, &text);
                    }
                }
                CommandEvent::Terminated(payload) => {
                    let state = app_handle.state::<NativeState>();
                    let message = format!("Bundled trainer exited: {:?}", payload);
                    let mut stopped_by_request = false;
                    if let Ok(mut trainer) = state.trainer.lock() {
                        stopped_by_request = trainer.stop_requested;
                        trainer.running = false;
                        trainer.ready = false;
                        trainer.child = None;
                        trainer.child_pid = None;
                        trainer.stop_requested = false;
                        trainer.error = if stopped_by_request { None } else { Some(message.clone()) };
                    }
                    if stopped_by_request {
                        push_trainer_log(&state, "lifecycle", "Bundled trainer stopped by user request.");
                    } else {
                        record_trainer_error(&app_handle, &message);
                    }
                    let _ = app_handle.emit("rvt-trainer-terminated", payload);
                    break;
                }
                _ => {}
            }
        }
    });

    Ok(())
}

#[tauri::command]
fn native_set_paired_origin(origin: String, state: State<'_, NativeState>) -> Result<(), String> {
    let normalized = validate_origin(&origin)?;
    *state.paired_origin.lock().map_err(|_| "Native origin state is unavailable.".to_string())? = Some(normalized);
    Ok(())
}

#[tauri::command]
fn native_trainer_status(state: State<'_, NativeState>) -> TrainerSidecarStatus {
    trainer_status_snapshot(&state)
}

#[tauri::command]
fn trainer_status(state: State<'_, NativeState>) -> TrainerLifecycleState {
    trainer_lifecycle_status(&state)
}

#[tauri::command]
fn trainer_start(bind_mode: Option<String>, app: AppHandle, state: State<'_, NativeState>) -> TrainerLifecycleState {
    {
        let trainer = match state.trainer.lock() {
            Ok(trainer) => trainer,
            Err(_) => return TrainerLifecycleState::Error("Native trainer state is unavailable.".to_string()),
        };
        if trainer.running && trainer.child.is_some() {
            return trainer_lifecycle_from_state(&trainer);
        }
    }

    if let Err(error) = start_local_trainer_sidecar(bind_mode, &app, &state) {
        if let Ok(mut trainer) = state.trainer.lock() {
            trainer.running = false;
            trainer.ready = false;
            trainer.child = None;
            trainer.child_pid = None;
            trainer.stop_requested = false;
            trainer.error = Some(error.clone());
        }
        record_trainer_error(&app, &error);
        return TrainerLifecycleState::Error(error);
    }
    trainer_lifecycle_status(&state)
}

#[tauri::command]
fn trainer_stop(state: State<'_, NativeState>) -> TrainerLifecycleState {
    shutdown_local_trainer(&state);
    trainer_lifecycle_status(&state)
}

#[tauri::command]
fn trainer_log_tail(state: State<'_, NativeState>) -> Vec<String> {
    state.trainer_log.lock()
        .map(|log| log.iter().cloned().collect())
        .unwrap_or_default()
}

#[tauri::command]
async fn native_pair_request(request: NativeRequest) -> Result<NativeResponse, String> {
    let route = request.path.split('?').next().unwrap_or("");
    if !matches!(route, "/api/server-info" | "/api/native-pairing-info" | "/api/auth/exchange") {
        return Err("Only pairing bootstrap routes are public to native transport.".to_string());
    }
    let response = perform_request(&request, None).await?;
    let status = response.status().as_u16();
    let data = response.json::<serde_json::Value>().await.map_err(|error| error.to_string())?;
    Ok(NativeResponse { status, data })
}

#[tauri::command]
async fn native_http_request(request: NativeRequest, state: State<'_, NativeState>) -> Result<NativeResponse, String> {
    let origin = validate_origin(&request.origin)?;
    let paired = state.paired_origin.lock().map_err(|_| "Native origin state is unavailable.".to_string())?.clone();
    let paired = paired.ok_or_else(|| "Pair with a trainer before making native API requests.".to_string())?;
    ensure_local_trainer_ready(&state, &origin).await?;
    let response = perform_request(&request, Some(&paired)).await?;
    let status = response.status().as_u16();
    let data = response.json::<serde_json::Value>().await.map_err(|error| error.to_string())?;
    Ok(NativeResponse { status, data })
}

#[tauri::command]
async fn native_download(request: NativeRequest, state: State<'_, NativeState>) -> Result<NativeDownload, String> {
    let origin = validate_origin(&request.origin)?;
    let paired = state.paired_origin.lock().map_err(|_| "Native origin state is unavailable.".to_string())?.clone();
    let paired = paired.ok_or_else(|| "Pair with a trainer before downloading artifacts.".to_string())?;
    ensure_local_trainer_ready(&state, &origin).await?;
    let response = perform_request(&request, Some(&paired)).await?;
    let status = response.status().as_u16();
    let content_type = response.headers().get(reqwest::header::CONTENT_TYPE)
        .and_then(|header| header.to_str().ok()).unwrap_or("application/octet-stream").to_string();
    let bytes = response.bytes().await.map_err(|error| error.to_string())?;
    Ok(NativeDownload { status, body_base64: STANDARD.encode(bytes), content_type })
}

#[tauri::command]
async fn check_and_install_update(app: AppHandle) -> Result<String, String> {
    use tauri_plugin_updater::UpdaterExt;

    let update = app
        .updater()
        .map_err(|error| error.to_string())?
        .check()
        .await
        .map_err(|error| error.to_string())?;

    match update {
        Some(update) => {
            update
                .download_and_install(|_, _| {}, || {})
                .await
                .map_err(|error| error.to_string())?;
            Ok("installed".to_string())
        }
        None => Ok("up-to-date".to_string()),
    }
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
    adapter.start_scan(ScanFilter::default()).await.map_err(|error| error.to_string())?;
    tokio::time::sleep(Duration::from_millis(timeout_ms.unwrap_or(8000).min(10_000))).await;
    let mut devices = Vec::new();
    for peripheral in adapter.peripherals().await.map_err(|error| error.to_string())? {
        if let Some(properties) = peripheral.properties().await.map_err(|error| error.to_string())? {
            let name = properties.local_name.unwrap_or_else(|| "Radar Vital BLE device".to_string());
            let normalized_name = name.to_ascii_lowercase();
            let service_match = properties.services.contains(&service_uuid);
            let name_match = normalized_name.contains("radar")
                || normalized_name.contains("vital")
                || normalized_name.contains("ailink")
                || normalized_name.contains("spo2")
                || normalized_name.contains("pulse");
            if !service_match && !name_match {
                continue;
            }
            devices.push(NativeBleDevice {
                id: peripheral.id().to_string(),
                name,
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

    #[test]
    fn accepts_plain_loopback_trainer_origin() {
        assert!(validate_origin("http://127.0.0.1:49152").is_ok());
    }

    #[test]
    fn trainer_lifecycle_reports_running_after_ready() {
        let state = TrainerSidecarState {
            running: true,
            ready: true,
            ..Default::default()
        };
        assert_eq!(trainer_lifecycle_from_state(&state), TrainerLifecycleState::Running);
    }

    #[test]
    fn trainer_lifecycle_reports_stopped_after_stop_state() {
        let mut state = TrainerSidecarState {
            running: true,
            ready: true,
            ..Default::default()
        };
        state.running = false;
        state.ready = false;
        state.child = None;
        assert_eq!(trainer_lifecycle_from_state(&state), TrainerLifecycleState::Stopped);
    }

    #[test]
    fn trainer_lifecycle_reports_error_when_child_exits_badly() {
        let state = TrainerSidecarState {
            error: Some("Bundled trainer exited with code 1".to_string()),
            ..Default::default()
        };
        assert_eq!(
            trainer_lifecycle_from_state(&state),
            TrainerLifecycleState::Error("Bundled trainer exited with code 1".to_string())
        );
    }

    #[test]
    fn trainer_log_tail_is_bounded() {
        let state = NativeState::default();
        for idx in 0..25 {
            push_trainer_log(&state, "stdout", &format!("line {idx}"));
        }
        let log = state.trainer_log.lock().unwrap();
        assert_eq!(log.len(), TRAINER_LOG_TAIL_LINES);
        assert_eq!(log.front().unwrap(), "[stdout] line 5");
        assert_eq!(log.back().unwrap(), "[stdout] line 24");
    }

    #[test]
    fn trainer_sidecar_args_include_local_bind_mode() {
        let args = trainer_sidecar_args("local", "49152", "C:\\RVT\\sessions");
        assert_eq!(args, vec![
            "serve",
            "--bind",
            "local",
            "--host",
            LOCAL_TRAINER_HOST,
            "--port",
            "49152",
            "--sessions-root",
            "C:\\RVT\\sessions",
            "--no-browser",
        ]);
    }

    #[test]
    fn trainer_sidecar_args_include_lan_bind_mode_and_fixed_port() {
        let args = trainer_sidecar_args("lan", "8765", "C:\\RVT\\sessions");
        assert_eq!(args, vec![
            "serve",
            "--bind",
            "lan",
            "--port",
            "8765",
            "--sessions-root",
            "C:\\RVT\\sessions",
            "--no-browser",
        ]);
    }

    #[test]
    fn trainer_bind_mode_rejects_unknown_values() {
        assert_eq!(trainer_bind_mode(None).unwrap(), "local");
        assert_eq!(trainer_bind_mode(Some("lan".to_string())).unwrap(), "lan");
        assert!(trainer_bind_mode(Some("public".to_string())).unwrap_err().contains("Unsupported trainer bind mode"));
    }
}

fn main() {
    tauri::Builder::default()
        .plugin(tauri_plugin_shell::init())
        .plugin(tauri_plugin_updater::Builder::new().build())
        .manage(NativeState::default())
        .setup(|app| {
            let app_handle = app.handle().clone();
            let state = app.state::<NativeState>();
            if let Err(error) = start_local_trainer_sidecar(None, &app_handle, &state) {
                let app_handle = app.handle().clone();
                let state = app.state::<NativeState>();
                if let Ok(mut trainer) = state.trainer.lock() {
                    trainer.running = false;
                    trainer.ready = false;
                    trainer.error = Some(error.clone());
                }
                record_trainer_error(&app_handle, &error);
                eprintln!("[RVT] {error}");
            }
            Ok(())
        })
        .on_window_event(|window, event| {
            if matches!(event, WindowEvent::CloseRequested { .. }) {
                let state = window.app_handle().state::<NativeState>();
                shutdown_local_trainer(&state);
            }
        })
        .invoke_handler(tauri::generate_handler![
            native_set_paired_origin,
            native_trainer_status,
            trainer_start,
            trainer_stop,
            trainer_status,
            trainer_log_tail,
            native_pair_request,
            native_http_request,
            native_download,
            check_and_install_update,
            native_ble_scan,
            native_ble_connect,
            native_ble_disconnect,
            native_ble_start_notifications,
        ])
        .run(tauri::generate_context!())
        .expect("error while running radar-vital");
}
