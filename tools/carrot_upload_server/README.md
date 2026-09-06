# Carrot upload server

This is the HTTP(S) receiver for Carrot dashcam, tmux, and authenticated
automatic-validation uploads. The production DSM configuration is
validation-only: it accepts one privately configured vehicle and keeps the
unauthenticated legacy dashcam/tmux protocol disabled.
`CARROT_ALLOWED_DEVICE_IDS` is a required allowlist separated by commas or
whitespace. `CARROT_ALLOWED_DEVICE_PUBLIC_KEY_SHA256` is a matching list of
`deviceId=lowercase-sha256` pins. The SHA-256 value is calculated over the DER
SubjectPublicKeyInfo bytes of that device's registered public key. An empty or
incomplete allowlist/pin pair fails closed, makes the health endpoint return
HTTP 503, and prevents validation authentication.

Legacy dashcam/tmux clients can request a short-lived session bound to their
Dongle ID and source IP only when `CARROT_LEGACY_UPLOADS_ENABLED=true` is set
deliberately. A Dongle ID is not a secret, so do not enable this protocol on a
public endpoint until it gains device authentication equivalent to the
automatic-validation protocol. Legacy sessions can never authorize or
overwrite the isolated validation namespace.

Authenticated validation sessions last 30 minutes; deliberately enabled legacy
sessions last four hours. Streaming bodies may take up to six hours, with a
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

1. The receiver first requires `deviceId` to be present in
   `CARROT_ALLOWED_DEVICE_IDS` and to have an exact pin in
   `CARROT_ALLOWED_DEVICE_PUBLIC_KEY_SHA256`. `POST
   /api/v1/validation/challenge` with `{"deviceId":"..."}` then returns a
   one-time, five-minute `challengeId`, `nonce`, and server audience. The
   challenge is persisted in SQLite before it is returned. A device/source pair
   has one active challenge at a time.
2. The device constructs these exact bytes (where `+` means concatenation):

   ```text
   b"dk-carrot-validation-device-proof-v2\0"
   + deviceId + b"\0" + challengeId + b"\0" + nonce + b"\0" + audience
   ```

   Every text field is UTF-8 encoded. It signs those bytes with its registered
   key using RS256 (RSA PKCS#1 v1.5 with SHA-256) or ES256 (P-256 ECDSA with
   SHA-256; ASN.1 DER signature). The signature is unpadded base64url.
3. `POST /api/v1/validation/session` sends `deviceId`, `challengeId`,
   `deviceKeyAlgorithm` (`RS256` or `ES256`), `devicePublicKey` (PEM
   SubjectPublicKeyInfo), and `deviceProof`, plus non-security display metadata.
   The receiver converts the public key to canonical DER SubjectPublicKeyInfo,
   compares its lowercase SHA-256 fingerprint to the device-specific pin, and
   verifies the signature locally. It never receives a generic comma JWT and
   never calls a comma API. Verification is one-time, source-IP-bound,
   single-flight, and globally concurrency-limited. A rejected key/signature
   returns HTTP 401 and counts toward a five-attempt limit; an unexpected local
   verifier failure returns retryable HTTP 503 without consuming an attempt.
   Both outcomes impose a short cooldown. A valid proof is exchanged for a
   purpose-scoped bearer session.
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
Hashing and local device-proof verification use separate bounded concurrency
limits.

The private deployment input is the 64-character lowercase `sha256` value from
the device's local `validation_device_key_fingerprint()` result. The fingerprint
and device ID are not secret credentials, but they are private deployment
policy and must not be committed to this repository. The private signing key
never leaves the device; only its public PEM is included in the one-time
session request and checked against the pre-enrolled fingerprint.
Configure the receiver mapping exactly as:

```text
CARROT_ALLOWED_DEVICE_PUBLIC_KEY_SHA256=<device-id>=<64-lowercase-hex-spki-fingerprint>
```

## DSM deployment

Use exactly one of these deployment methods; do not run the Container Manager
Compose project and `deploy_dsm.sh` together because both claim the same
container name and loopback port.

For a manual Container Manager deployment:

1. Copy this directory to `/volume1/docker/dk-upload`.
2. Create a private, untracked Compose environment file containing exactly one
   `DK_UPLOAD_ALLOWED_DEVICE_ID` and its 64-character lowercase
   `DK_UPLOAD_DEVICE_PUBLIC_KEY_SHA256`. Never commit this file or paste its
   values into the public Compose YAML.
3. Create `/volume1/openpilot/.carrot-validation-v1` and
   `/volume1/docker/dk-upload/state`, and grant numeric UID/GID `10001:10001`
   recursive read/write access only to those two dedicated directories.
4. Create a Container Manager project named `dk-upload` from
   `compose.dsm.yml`. The image and container run as `10001:10001`, with no
   DSM administrators group. Only the validation directory and private SQLite
   state directory are mounted; the rest of `/volume1/openpilot` is not visible
   inside the container. No DSM password or FTP login is passed to it.
5. Confirm the project reports healthy and that the container is named exactly
   `dk-upload` before continuing.
6. Add a DSM reverse-proxy rule named `dk-upload` from
   `https://adot.synology.me:443` to
   `http://127.0.0.1:18080` and assign a trusted certificate for that hostname.
7. Forward only TCP 443 from the router to DSM. Keep port 18080 bound to
   loopback, and do not expose DSM management, FTP, SMB, or the upload
   directory. Test from a genuinely external connection because router NAT
   loopback behavior varies.

For the preferred repeatable deployment, run `deploy_dsm.sh` as a root DSM Task
Scheduler job with all three private deployment values:

```sh
DK_UPLOAD_GITHUB_REF=<reviewed-exact-40-character-commit-sha> \
DK_UPLOAD_ALLOWED_DEVICE_ID=<private-dongle-id> \
DK_UPLOAD_DEVICE_PUBLIC_KEY_SHA256=<64-lowercase-hex-spki-fingerprint> \
/volume1/docker/dk-upload/deploy_dsm.sh
```

The wrapper variables intentionally differ from the internal container
variables. The script rejects an absent, multiple, or malformed device ID and
any fingerprint that is not exactly 64 lowercase hexadecimal characters. It
downloads only this receiver's build files from the immutable revision, builds
`dk-upload`, binds it to loopback, verifies the writable validation store and
fail-closed API policy, and restores the previous container after any
pre-commit failure, cancellation, or signal. A process lock prevents concurrent
deployments, and the next run recovers an interrupted rollback left by a power
loss. Branch names such as `dkcarrot-wip` are refused so a later force-push
cannot silently change deployed code.

The production script always keeps `CARROT_LEGACY_UPLOADS_ENABLED=false`, fixes
`CARROT_VALIDATION_AUDIENCE` to the public DK HTTPS validation API, and passes
the one private device ID/key pin only through container environment. The
Python base image is digest-pinned and the Python dependency closure is
version-locked; updates require a new reviewed Git commit.

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

The DSM reverse proxy must allow a request body larger than the configured
512 MiB file limit, stream request bodies instead of buffering them, preserve
the real client in `X-Forwarded-For`, and use read/send timeouts longer than the
six-hour upload deadline. No outbound identity service is required by the NAS;
the receiver verifies the challenge proof locally against the private SPKI pin.

After every image change, explicitly rebuild and recreate the Container Manager
project; do not reuse a cached older image. Before enabling collection, verify
`GET /api/v1/health` and confirm that `deviceAllowlistConfigured`,
`deviceKeyPinsConfigured`, and `storageWritable` are true and
`legacyUploadsEnabled` is false. The endpoint exposes only pin readiness, never
the fingerprint or public key. Confirm that
`POST /api/v1/validation/challenge` returns a challenge rather than HTTP 404,
that a non-allowlisted ID is rejected, and that `POST /api/v1/session` is
rejected with HTTP 403. Then run an authenticated validation
session/test-segment/completion from the allowed vehicle and verify its
canonical manifest and hashes on the NAS. The public endpoint does not provide
automatic validation until that rebuild and end-to-end receipt test succeeds.
Only then disable the old transfer service and remove its account.

Run the public challenge check with a real Dongle ID after the reverse proxy is
updated:

```sh
DK_UPLOAD_ALLOWED_DEVICE_ID=<private-dongle-id> \
curl --fail-with-body -X POST \
  -H 'Content-Type: application/json' \
  --data "{\"deviceId\":\"$DK_UPLOAD_ALLOWED_DEVICE_ID\"}" \
  https://adot.synology.me/api/v1/validation/challenge
```

Success is HTTP 200 JSON containing `challengeId`, `nonce`, `audience`, and
`expiresAt`. HTTP 403 for this ID means the deployed allowlist is wrong;
HTTP 404 means the old receiver is still deployed.

When legacy uploads are deliberately enabled on a protected deployment,
dashcam files retain the original FTP-era layout at
`/volume1/openpilot/routes/<CarName> <DongleID>/<segment>`. Tmux files use
`/volume1/openpilot/<GitBranch>/<CarName> <DongleID>/<reason>-<time>-<branch>.txt`.
Strict server-side path validation confines web writes to the expected route
and branch/device layouts, and there is no public download API. Completion
manifests and quota/session state stay in `/volume1/docker/dk-upload/state`. A
validation-only deployment cannot see, scan, or delete unmanaged files
elsewhere in the existing Openpilot tree.
