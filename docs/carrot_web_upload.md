# DK private log receiver

The `dkcarrot-wip` production receiver is the owner's private, authenticated
KA4 automatic-validation endpoint. It uses HTTP(S) only; no FTP, DSM account,
password, WebDAV, or shared storage credential is present on the vehicle.

## Production behavior

- The built-in destination is `https://adot.synology.me`.
- `CarrotValidationAutoUpload` is an independent, default-off consent. It is
  restricted on both client and server to the owner's allowlisted DK device and the
  KA4 stock radar-SCC/no-openpilot-longitudinal topology.
- The receiver exchanges a one-time challenge and verifies a domain-separated
  signature made by the device registration key against a privately pinned
  public-key fingerprint. No reusable comma API bearer leaves the device.
  Every upload session is also bound to its source IP.
- The production DSM configuration sets
  `CARROT_LEGACY_UPLOADS_ENABLED=false`. Therefore the unauthenticated legacy
  dashcam/tmux session API is deliberately unavailable on the public endpoint.
- `CarrotCommunityDataSharing` is a separate default-off switch for Carrot
  community heartbeat, settings statistics, CWP, automatic tmux diagnostics,
  and bundled Discord destinations. It does not authorize or redirect the
  private KA4 validation path.

The normal Carrot Web dashcam/tmux client remains useful with an explicitly
configured private receiver that implements its session API. The public DK
receiver does not accept that legacy protocol: a manual upload to its default
URL fails closed with HTTP 403. Local log browsing, download, tmux capture, and
KA4 automatic validation are unaffected. Do not enable the legacy protocol on
the Internet merely to make manual upload work; a Dongle ID is not a secret.

## Validation API and protection policy

- `GET /api/v1/health` is public and reports readiness plus non-sensitive
  policy flags. It returns HTTP 503 when the device allowlist is empty.
- `POST /api/v1/validation/challenge` issues a persisted one-time nonce only
  for the allowed Dongle ID.
- `POST /api/v1/validation/session` verifies the challenge-bound signature
  locally against the device's pinned public-key fingerprint and returns a
  random validation-only bearer session.
- `PUT /api/v1/validation/upload/{captureId}/{segment}/{filename}` accepts only
  immutable `rlog`, `rlog.zst`, or `rlog.bz2` objects with exact size and SHA-256.
- `POST /api/v1/validation/complete` re-hashes all files and atomically writes
  the canonical receipt manifest.
- `/api/v1/session`, ordinary dashcam upload/completion, and tmux upload return
  HTTP 403 in the production validation-only policy.

The deployed limits include a 1 GiB per-device UTC-day network quota, 512 MiB
per file, two validation uploads per device, eight globally, and a 10 GiB
free-space floor. Partial files and reservations are reconciled after restart;
published files and unrelated NAS content are never overwritten or deleted.

## DSM deployment and exposure

The hardened receiver lives in `tools/carrot_upload_server`. Deploy the exact
reviewed commit with `deploy_dsm.sh` from DSM Task Scheduler. The script builds
`dk-upload`, starts it as a dedicated numeric UID/GID `10001:10001` without
Linux capabilities or a DSM administrators group, checks the fail-closed
health response, and restores the previous container if the new one fails or
the deployment is interrupted.

The container binds only `127.0.0.1:18080` and can see only these two writable
host paths; the rest of `/volume1/openpilot` is not mounted at all:

- `/volume1/openpilot/.carrot-validation-v1` — completed captures and manifests
- `/volume1/docker/dk-upload/state` — sessions, quotas, and receiver state

DSM should expose the service through one reverse-proxy rule from
`https://adot.synology.me:443` to `http://127.0.0.1:18080`, using the trusted
certificate for that hostname. Forward only router TCP 443 to NAS TCP 443. Do
not expose port 18080, DSM management, SMB, FTP, WebDAV, or a file-download
route.

Before enabling collection, verify all of the following from a genuinely
external network:

1. health is HTTP 200 with `service=dk-upload`, allowlist and writable storage
   confirmed, and legacy uploads disabled;
2. the allowed device receives a challenge and another device gets HTTP 403;
3. the legacy session endpoint gets HTTP 403;
4. one authenticated vehicle capture completes with matching hashes and a
   canonical `manifest.json` under the exact Dongle-ID directory.
