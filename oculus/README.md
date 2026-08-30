# Oculus Quest 2 monitor (Option 0)

A read-only WebXR dashboard for `robot-edge`, built for a stock/unmodified Meta Quest 2 --
no developer mode, no sideloading. Implements Option 0 from the [Quest 2 VR-teleop
discussion](https://github.com/spinworks-tech/robotele/discussions/12) and
[issue #13](https://github.com/spinworks-tech/robotele/issues/13).

- `gateway/` -- a Rust binary (`oculus-gateway`) that speaks the real
  QUIC/mTLS/FlatBuffers protocol to `robot-edge` (reusing `roboprotocol-core`/
  `roboprotocol-proto`), decodes Channel A video (H.264 -> JPEG) and Channel B
  telemetry, and serves both over a WebSocket to any browser.
- `web/` -- the static WebXR page served by the gateway: a video quad + a
  floating telemetry HUD panel, rendered with Three.js.

## What this deliberately does not do

This gateway never sends a Channel B command, ActionTrigger, CameraControl, or
E-Stop -- it only reads. See `gateway/src/quic_client.rs`'s module doc for the
two consequences worth knowing before pointing it at a real robot:

1. `robot-edge` is v0 single-connection only, so this gateway and a live
   `operator-console` teleop session can't be attached at the same time.
2. `robot-edge`'s safety watchdog latches E-Stop ~400ms after the last Channel
   B command it saw. Since this gateway never sends one, the robot E-Stops
   itself shortly after the session reaches Operating -- expected, not a bug.

## Running it

```bash
# from the repo root, against a robot-edge already listening on 127.0.0.1:4433
cargo run -p oculus-gateway -- --connect 127.0.0.1:4433

# against a real robot on the LAN, with the gateway's own HTTPS cert covering
# the LAN IP you'll type into the Quest Browser
cargo run -p oculus-gateway -- --connect 192.168.68.64:4433 --tls-san 192.168.68.<gateway-ip>
```

Then, on the Quest 2 (same Wi-Fi network), open
`https://<gateway-host>:8443/` in the Quest Browser, accept the self-signed
certificate warning (required once per session/restart -- see
`gateway/src/main.rs`'s doc for why it's generated fresh each run rather than
persisted), and tap **Enter VR** once the HUD shows `phase: operating`.

`--web-dir` defaults to `oculus/web` relative to the working directory the
gateway is launched from; pass an absolute path if you run it from elsewhere.
