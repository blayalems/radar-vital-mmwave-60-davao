# Install PWA

The hosted PWA is the static dashboard shell published by GitHub Pages. Its
stable URL is:

`https://blayalems.github.io/radar-vital-mmwave-60-davao/`

The stable legal URLs required by Play and operator review are:

- Privacy: `https://blayalems.github.io/radar-vital-mmwave-60-davao/privacy.html`
- Terms: `https://blayalems.github.io/radar-vital-mmwave-60-davao/terms.html`

## Install from browser

On Android Chrome, open the Pages URL, then choose **Install app** or **Add to
Home screen** from the browser menu. On desktop Chrome or Edge, use the install
button in the address bar when it appears.

The PWA can open in demo mode without a trainer. For live sessions it must pair
with a LAN-bound trainer running on the Windows host. Because the hosted page is
served over HTTPS and the trainer may be local HTTP, use only the pairing mode
that the current release has verified for the target browser and deployment.

## Offline behavior

The service worker caches the shell and static assets so the dashboard can load
for review when offline. API calls remain network-only. If the trainer is not
reachable, the app must show disconnected/demo states rather than silently
pretending a live session exists.

## Privacy expectations

The hosted PWA does not upload measurement data to GitHub Pages. Live session
data is stored by the trainer computer. Diagnostics are user-initiated through
the GitHub issue flow and should be reviewed by the operator before submission.
