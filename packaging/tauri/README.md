# Tauri v2 Windows Packaging

Tauri uses Microsoft Edge WebView2, not a bundled fallback browser engine.

The default config uses:

```json
"webviewInstallMode": { "type": "downloadBootstrapper" }
```

This keeps the installer small and downloads WebView2 on Windows 10 when needed. For offline installs, change the type to `embedBootstrapper` and accept the larger installer.

The WebView CSP keeps `connect-src 'self'`. HTTP access to the trainer should be implemented through a Tauri HTTP plugin or by connecting to a local trainer at `localhost`, not by broadening the bundled shell CSP.
