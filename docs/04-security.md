# Security

Source of truth: `DESIGN.md` §4, `REQUIREMENTS.md` §6 (Security Requirements).

## Encryption & confidentiality

- **Control datagram encryption:** all control, haptic, and telemetry packets are
  encrypted using QUIC's native TLS 1.3 (RFC 9000/9001). Cipher suites negotiate in
  strict priority order: **AES-256-GCM primary** (matches hardware AES acceleration
  present on every target platform — Intel VAES, AMD VAES/PSP, ARM Crypto Extensions,
  ESP32 HW AES engine), **ChaCha20-Poly1305 as fallback** for peers without AES
  hardware.
- **Media encryption:** video and audio are encrypted as QUIC unreliable datagrams
  under the same connection's native TLS 1.3 session — no separate SRTP session or
  keying required.
- **0-RTT early data restriction:** TLS 1.3 0-RTT session resumption is scoped to
  non-actuating early data only — in practice, the `HELLO` version/capability
  handshake. 0-RTT data carries no forward secrecy or anti-replay guarantee, so
  Channel B motion commands and any mode-change RPC are explicitly forbidden as 0-RTT
  early data and must wait for full 1-RTT handshake completion before reaching the
  joint controller.

## Key exchange & authentication

- **Key exchange:** **X25519** is the sole supported group — the cheapest
  constant-time ECDH curve available on constrained ARM edge controllers, minimizing
  handshake cost on every QUIC connection-migration re-key event, not just initial
  connect.
- **Mutual authentication:** X.509 certificates validated during QUIC session setup
  (mTLS). The spec specifies **Ed25519** signatures — smaller certificates,
  deterministic signing, faster verification on embedded edge controllers — with
  ECDSA P-256 permitted only where HSM/TPM hardware mandates a NIST curve.
- **Role-based access control:** teleoperation sessions enforce role boundaries
  separating high-privilege system configuration RPCs from low-privilege motion
  tracking streams.

## IP mobility

QUIC Connection IDs (RFC 9000 §19) let the secure control session migrate seamlessly
across IP address changes (Wi-Fi ↔ 5G handoffs) without re-keying or dropping
connection state — see
[Protocol Architecture § Session mobility](02-protocol-architecture.md#session-mobility).

## Integrity & anti-replay

- Datagrams carry incrementing 64-bit sequence numbers and HMAC-SHA256 message
  authentication codes to reject replayed control packets.
- Packets failing MAC authentication are dropped immediately and logged to a security
  audit subsystem.
- QUIC packet numbers and header fields are protected against tampering/replay via
  AES-ECB/ChaCha20 header protection masks.

## A documented deviation in the reference implementation

The spec (§4) calls for **Ed25519** certificates. The reference implementation's
certificate generator, `tools/dev-certs`, issues **ECDSA P-256** certificates instead
— a deliberate, documented workaround: `quiche` 0.22.0's vendored BoringSSL fails to
load Ed25519 PKCS8 keys. This is a known gap between spec and v0 implementation, not
a silent substitution — see [Reference Implementation](05-reference-implementation.md#tools).

## Dev certificates are not production credentials

The certificates committed under `certs/` in this repository exist purely so a fresh
checkout can run the reference implementation immediately, with no setup step:

- Self-signed, never expire (`notBefore=1975`/`notAfter=4096`), and **not treated as
  secret** — anyone with the repository can already stand up either endpoint with
  them.
- The CA's own private key (`certs/dev-ca/ca.key`) is **not** committed, so a
  checkout can verify existing connections but cannot mint new trusted certificates.
- Verification is by DNS name in the certificate's SAN (`robot-edge`, via
  `--server-name robot-edge`), not by IP — so the robot's LAN IP changing on every
  DHCP lease doesn't require regenerating anything.

**Do not reuse these certificates in a real deployment.** Regenerate a private CA and
leaf certificates with `cargo run -p dev-certs -- --out-dir <dir> --robot-san
<hostname>` and keep the CA key off any machine that doesn't need to mint new certs.

## What's still open (per `REVIEW.md`)

The security model is well-specified for an *already-paired* operator/robot pair —
mutual TLS, replay protection, scoped 0-RTT. What's explicitly out of scope of the
current spec, and flagged in [Design Review & Roadmap](09-design-review-and-roadmap.md):

- **No device discovery or pairing mechanism.** The protocol assumes the operator
  already knows the robot's address and both sides are already provisioned with
  certificates — there's no mDNS/QR-code/NFC pairing flow defined yet.
- **No cloud discovery / OAuth2 story** for remote WAN teleoperation.

These are adoption gaps for a consumer product, not weaknesses in the cryptographic
design of an already-connected session.
