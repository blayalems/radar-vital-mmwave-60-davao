# Capacitor APK Packaging

This package bundles the v12 dashboard shell into `www/` and uses native bridges for device access.

- LAN HTTP must go through a native HTTP plugin (`@capacitor-community/http` in this package); do not reintroduce in-WebView `fetch()` to plain HTTP LAN origins.
- BLE must go through `@capacitor-community/bluetooth-le`.
- `resolveServiceWorkerRequests` is disabled by default. If a release enables it, verify the APK on hardware and document the observed behavior.
- No broad cleartext `network_security_config.xml` is needed for v12 because LAN HTTP is handled by the native HTTP bridge.

Build outline:

```powershell
npm install
npm run prepare:www
npx cap add android
npm run sync:android
```

Open Android Studio for signing and release generation, or run the Gradle release task after keys are configured.
