# Carrot upload server

This is the HTTP(S) receiver for public Carrot dashcam, tmux, and automatic
validation uploads. Users do not create or enter tokens. Legacy dashcam/tmux
clients request a short-lived session bound to their Dongle ID and source IP.
Those legacy sessions remain compatible but can never authorize or overwrite
the isolated validation namespace.

Sessions last four hours. Streaming bodies may take up to six hours, with a
60-second per-chunk idle deadline, so slow tethering remains usable while a
stalled sender cannot hold a worker forever. Small JSON bodies have separate
10-second idle and 30-second total deadlines.

The default policy is 1 GiB per Dongle ID per UTC day with no bandwidth
throttle. Abuse and storage safeguards are enforced independently: 8 GiB per
source IP/day, three concurrent legacy uploads per device, sixteen legacy
uploads globally, two automatic-validation operations per device, eight
validation operations globally, 512 MiB per file, a 10 GiB free-space floor,
and safe path validation. Legacy and authenticated validation traffic have
separate device/IP daily-quota namespaces, rate buckets, and bounded global and
per-IP request-admission pools that apply before body parsing or authentication
and remain held through the final database operation. The independent pools
prevent a slow or malformed unauthenticated legacy request from consuming
validation capacity. Legacy storage reservations must also leave a dedicated
1 GiB validation headroom above the common 10 GiB floor. Zero-byte
legacy files are rejected. Files that have been published are never removed
automatically. The quota measures bytes actually received, including failed
integrity checks and idempotent/hard-link race retries; stored bytes are tracked
separately. Validation completion HTTP bodies count as
network usage even when JSON/schema validation fails or an immutable receipt is
retried. A newly completed capture reserves and records two canonical-manifest
copies (the file plus its SQLite recovery BLOB); an idempotent retry records no
second storage delta. Persistent upload leases reserve quota and free space
atomically so concurrent requests cannot overcommit either resource. Streaming
progress is retained in memory immediately and durably coalesced at 1 MiB or
one-second boundaries, bounding SQLite sync amplification while keeping
crash-time network undercount below those thresholds and disk reservations
conservative.

## Authenticated automatic validation protocol

Automatic validation uses a separate, fail-closed device authentication flow:

1. `POST /api/v1/validation/challenge` with `{"deviceId":"..."}` returns a
   one-time, five-minute `challengeId`, `nonce`, and server audience. The
   challenge is persisted in SQLite before it is returned. A device/source pair
   has one active challenge at a time.
2. The comma device signs a short-lived identity JWT containing its normal
   `identity`, `iat`, `nbf`, and `exp` claims plus the exact
   `carrotUploadChallenge`, `carrotUploadNonce`,
   `carrotUploadPurpose="validation"`, and `carrotUploadAudience` values.
   `POST /api/v1/validation/session` exchanges that JWT and challenge for a
   purpose-scoped bearer session.
3. Before issuing the session, the receiver sends the same JWT as
   `Authorization: JWT ...` to the configured official comma device endpoint.
   Redirects are disabled, the URL origin is fixed by configuration, response
   identity (when present) must match the requested Dongle ID, and
   connect/total/read timeouts are bounded. Verification is single-flight and
   globally concurrency-limited. Definitive authentication rejection returns
   HTTP 401 and counts toward a five-attempt limit; upstream timeouts, HTTP 429,
   HTTP 5xx, and malformed upstream responses return retryable HTTP 503 without
   consuming an authentication attempt. Both outcomes impose a short cooldown.
4. `PUT /api/v1/validation/upload/{captureId}/{segment}/{filename}` accepts
   only `rlog`, `rlog.bz2`, or `rlog.zst`. It requires `X-File-Size` and
   `X-Content-SHA256`, hashes while streaming, fsyncs the file and directory,
   and records the receipt in SQLite. An identical retry is storage-idempotent
   (but its received network bytes still count toward quota); a different file
   at the same logical path returns HTTP 409.
5. `POST /api/v1/validation/complete` accepts exactly the four top-level keys
   `deviceId`, `captureId`, `files: [{segment,name,size,sha256}]`, and the
   protocol-v1 `validationCapture` object emitted by the device. A capture
   contains one to three unique segments and exactly one full `rlog` file per
   segment. Unknown fields, nested shapes, wrong JSON types, out-of-range values, and oversized
   completion/manifest bodies are rejected. The file list and metadata segment
   set must match the SQLite receipts exactly. The server writes one canonical,
   immutable manifest and returns deterministic `receiptId`,
   `manifestSha256`, `files`, `verifiedDeviceId`, and `captureId` values. Every
   idempotent completion retry rechecks the exact current DB set plus each
   rlog's existence, size, and SHA-256 before returning that receipt.

Validation rlogs and manifests live below
`/volume1/openpilot/.carrot-validation-v1/<DongleID>/<captureId>`. No public
download endpoint is provided. SQLite uses WAL mode with `synchronous=FULL`;
published files and their parent directories are fsynced before a receipt is
acknowledged.

Because the deployment is a single receiver process, startup treats every
persisted upload lease as crash residue: it immediately charges its last
durable received/stored counts, releases its quota/free-space reservation, and
removes every receiver-named `.part` file in managed directories. Published and
non-receiver-owned files remain untouched. The 15-minute cleanup uses the
configured lease/stale-age cutoffs for the running process instead. Both paths
also repair a missing completed manifest from its immutable SQLite copy and
drop missing-file DB rows only for captures that have not been completed.
Hashing and official identity verification use separate bounded concurrency
limits.

## DSM deployment

1. Copy this directory to `/volume1/docker/carrot-upload`.
2. Create a Container Manager project from `compose.dsm.yml`.
3. The compose file runs as DSM UID 1026/GID 100 (with the DSM administrators
   ACL group, GID 101) and bind-mounts the existing `/volume1/openpilot`
   folder so tmux diagnostics retain their original branch-based path. No DSM
   password or FTP login is passed to the container. Adjust the numeric UID
   only if this DSM account changes.
4. Add DSM reverse proxy `https://upload.shind0.synology.me:443` to
   `http://127.0.0.1:18080` and assign a trusted certificate for that hostname.
5. Keep port 18080 bound to loopback. Do not publish the upload directory.

Set `CARROT_VALIDATION_AUDIENCE` to the public HTTPS validation API URL (or
another stable deployment-specific audience identifier). The default verifier
endpoint is `https://api.commadotai.com/v1.1/devices/{device_id}/`; override
`CARROT_VALIDATION_VERIFY_URL_TEMPLATE` only with a fixed HTTPS comma API
endpoint that returns the authenticated `dongle_id` or `id`.

Operational hardening knobs are `CARROT_CONCURRENT_PER_DEVICE` and
`CARROT_CONCURRENT_GLOBAL` (legacy defaults 3/16),
`CARROT_VALIDATION_CONCURRENT_PER_DEVICE` and
`CARROT_VALIDATION_CONCURRENT_GLOBAL` (validation defaults 2/8),
`CARROT_UPLOAD_IDLE_TIMEOUT_SECONDS` (60),
`CARROT_UPLOAD_TOTAL_TIMEOUT_SECONDS` (21600),
`CARROT_JSON_BODY_IDLE_TIMEOUT_SECONDS` (10),
`CARROT_JSON_BODY_TOTAL_TIMEOUT_SECONDS` (30),
`CARROT_REQUEST_CONCURRENT_PER_IP` and `CARROT_REQUEST_CONCURRENT_GLOBAL`
(legacy defaults 4/16),
`CARROT_VALIDATION_REQUEST_CONCURRENT_PER_IP` and
`CARROT_VALIDATION_REQUEST_CONCURRENT_GLOBAL` (validation defaults 4/16),
`CARROT_VALIDATION_FREE_SPACE_RESERVE_BYTES` (1073741824),
`CARROT_PROGRESS_COMMIT_BYTES` (1048576),
`CARROT_PROGRESS_COMMIT_INTERVAL_SECONDS` (1),
`CARROT_SESSION_RATE_BUCKET_LIMIT` (4096 per legacy/validation namespace),
`CARROT_VALIDATION_VERIFY_CONCURRENT` (4),
`CARROT_VALIDATION_VERIFY_ATTEMPT_LIMIT` (5),
`CARROT_VALIDATION_VERIFY_COOLDOWN_SECONDS` (2),
`CARROT_VALIDATION_HASH_CONCURRENT` (2), `CARROT_RESERVATION_LEASE_SECONDS`
(7200), `CARROT_STALE_PART_SECONDS` (7200), and
`CARROT_CLEANUP_INTERVAL_SECONDS` (900).

After every image change, explicitly rebuild and recreate the Container Manager
project; do not reuse a cached older image. Before enabling collection, verify
`GET /api/v1/health` and confirm that
`POST /api/v1/validation/challenge` returns a challenge rather than HTTP 404,
then run an authenticated validation session/test segment/completion and a tmux
upload. The public endpoint does not provide automatic validation until that
rebuild and smoke test succeed. Only then disable the old transfer service and
remove its account.

Run the public challenge check with a real Dongle ID after the reverse proxy is
updated:

```sh
curl --fail-with-body -X POST \
  -H 'Content-Type: application/json' \
  --data '{"deviceId":"REAL_DONGLE_ID"}' \
  https://upload.shind0.synology.me/api/v1/validation/challenge
```

Success is HTTP 200 JSON containing `challengeId`, `nonce`, `audience`, and
`expiresAt`. HTTP 404 means the old receiver is still deployed.

Dashcam files retain the original FTP-era layout at
`/volume1/openpilot/routes/<CarName> <DongleID>/<segment>`. Tmux files use
`/volume1/openpilot/<GitBranch>/<CarName> <DongleID>/<reason>-<time>-<branch>.txt`.
Strict server-side path validation confines web writes to the expected route
and branch/device layouts, and there is no public download API. Completion
manifests and quota/session state stay in the hidden
`/volume1/openpilot/tmux/.state` directory. The receiver never scans or deletes
unmanaged files in the existing Openpilot tree.
