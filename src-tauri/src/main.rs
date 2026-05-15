// Tauri entry point. The web shell (www/) does all the UI work; the Rust side
// stays minimal so we can keep the MSI small and the attack surface tiny.

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

fn main() {
    tauri::Builder::default()
        .run(tauri::generate_context!())
        .expect("error while running radar-vital");
}
