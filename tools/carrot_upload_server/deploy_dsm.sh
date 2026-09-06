#!/bin/sh

# Install the validation-only DK upload receiver on Synology DSM from one
# immutable Git commit. Run this script as root from DSM Task Scheduler.

set -eu

# DSM Task Scheduler uses a minimal, non-login PATH. Container Manager normally
# installs Docker in /usr/local/bin, while standard BusyBox tools live below it.
PATH=/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin
export PATH

REPOSITORY="${DK_UPLOAD_GITHUB_REPOSITORY:-powtrix/openpilot}"
REVISION="${DK_UPLOAD_GITHUB_REF:-}"
DEVICE_ID="${DK_UPLOAD_ALLOWED_DEVICE_ID:-}"
DEVICE_PUBLIC_KEY_SHA256="${DK_UPLOAD_DEVICE_PUBLIC_KEY_SHA256:-}"
DEPLOY_ROOT="${DK_UPLOAD_DEPLOY_ROOT:-/volume1/docker/dk-upload}"
OPENPILOT_ROOT="${DK_UPLOAD_OPENPILOT_ROOT:-/volume1/openpilot}"
RUN_UID="${DK_UPLOAD_UID:-10001}"
RUN_GID="${DK_UPLOAD_GID:-10001}"

case "$REVISION" in
  *[!0-9a-f]*|"") echo "DK_UPLOAD_GITHUB_REF must be an exact 40-character lowercase commit SHA" >&2; exit 2 ;;
esac
if [ "${#REVISION}" -ne 40 ]; then
  echo "DK_UPLOAD_GITHUB_REF must be an exact 40-character lowercase commit SHA" >&2
  exit 2
fi

case "$REPOSITORY" in
  *[!A-Za-z0-9._/-]*|*..*|/*|*/|"") echo "invalid GitHub repository" >&2; exit 2 ;;
esac

# This deployment is intentionally private to one owner device. Requiring one
# syntactically exact value at invocation time makes a missing Task Scheduler
# secret fail closed without embedding the private identifier in source.
case "$DEVICE_ID" in
  *[!A-Za-z0-9_-]*|"")
    echo "DK_UPLOAD_ALLOWED_DEVICE_ID must be one valid device ID" >&2
    exit 2
    ;;
esac
if [ "${#DEVICE_ID}" -lt 8 ] || [ "${#DEVICE_ID}" -gt 64 ]; then
  echo "DK_UPLOAD_ALLOWED_DEVICE_ID must be 8 to 64 characters" >&2
  exit 2
fi
case "$(printf '%s' "$DEVICE_ID" | tr '[:upper:]' '[:lower:]')" in
  unknown|none)
    echo "DK_UPLOAD_ALLOWED_DEVICE_ID must identify the private device" >&2
    exit 2
    ;;
esac
if printf '%s' "$DEVICE_ID" | grep -q '[,[:space:]=]'; then
  echo "DK_UPLOAD_ALLOWED_DEVICE_ID must contain exactly one device ID" >&2
  exit 2
fi
case "$DEVICE_PUBLIC_KEY_SHA256" in
  *[!0-9a-f]*|"")
    echo "DK_UPLOAD_DEVICE_PUBLIC_KEY_SHA256 must be exactly 64 lowercase hexadecimal characters" >&2
    exit 2
    ;;
esac
if [ "${#DEVICE_PUBLIC_KEY_SHA256}" -ne 64 ]; then
  echo "DK_UPLOAD_DEVICE_PUBLIC_KEY_SHA256 must be exactly 64 lowercase hexadecimal characters" >&2
  exit 2
fi

case "$DEPLOY_ROOT" in
  /volume[0-9]*/docker/dk-upload) ;;
  *) echo "DK_UPLOAD_DEPLOY_ROOT must be a dedicated /volumeN/docker/dk-upload path" >&2; exit 2 ;;
esac
case "$OPENPILOT_ROOT" in
  /volume[0-9]*/openpilot) ;;
  *) echo "DK_UPLOAD_OPENPILOT_ROOT must be a dedicated /volumeN/openpilot path" >&2; exit 2 ;;
esac
case "$RUN_UID:$RUN_GID" in
  *[!0-9:]*|:*|*:) echo "DK upload UID and GID must be numeric" >&2; exit 2 ;;
esac

if [ -x /usr/local/bin/docker ]; then
  DOCKER=/usr/local/bin/docker
elif [ -x /usr/bin/docker ]; then
  DOCKER=/usr/bin/docker
else
  echo "Docker is not installed or is unavailable to DSM Task Scheduler" >&2
  exit 1
fi
if [ -x /usr/bin/curl ]; then
  CURL=/usr/bin/curl
elif [ -x /bin/curl ]; then
  CURL=/bin/curl
else
  echo "curl is unavailable to DSM Task Scheduler" >&2
  exit 1
fi

mkdir -p "$DEPLOY_ROOT"
LOCK_DIR="$DEPLOY_ROOT/.deploy.lock"
if ! mkdir "$LOCK_DIR" 2>/dev/null; then
  LOCK_PID=""
  if [ -f "$LOCK_DIR/pid" ]; then
    LOCK_PID="$(sed -n '1p' "$LOCK_DIR/pid" 2>/dev/null || true)"
  fi
  case "$LOCK_PID" in
    *[!0-9]*|"") LOCK_PID="" ;;
  esac
  if [ -n "$LOCK_PID" ] && kill -0 "$LOCK_PID" 2>/dev/null; then
    echo "another dk-upload deployment is already running (PID $LOCK_PID)" >&2
    exit 1
  fi
  rm -f "$LOCK_DIR/pid"
  rmdir "$LOCK_DIR" 2>/dev/null || {
    echo "stale deployment lock could not be recovered: $LOCK_DIR" >&2
    exit 1
  }
  mkdir "$LOCK_DIR" || {
    echo "could not acquire deployment lock" >&2
    exit 1
  }
fi
printf '%s\n' "$$" > "$LOCK_DIR/pid"

STAGE=""
HAD_CURRENT=0
OLD_RENAMED=0
NEW_ATTEMPTED=0
COMMITTED=0

container_exists() {
  "$DOCKER" container inspect "$1" >/dev/null 2>&1
}

container_healthy() {
  "$DOCKER" exec "$1" python -c \
    'import json,urllib.request; d=json.load(urllib.request.urlopen("http://127.0.0.1:8080/api/v1/health", timeout=2)); assert d.get("ok") is True and d.get("service") == "dk-upload" and d.get("deviceAllowlistConfigured") is True and d.get("deviceKeyPinsConfigured") is True and d.get("legacyUploadsEnabled") is False and d.get("storageWritable") is True' \
    >/dev/null 2>&1
}

cleanup() {
  STATUS=$?
  trap - 0 HUP INT TERM

  if [ "$COMMITTED" -ne 1 ]; then
    if [ "$NEW_ATTEMPTED" -eq 1 ] && container_exists dk-upload; then
      "$DOCKER" rm -f dk-upload >/dev/null 2>&1 || true
    fi
    if [ "$OLD_RENAMED" -eq 1 ] && container_exists dk-upload-rollback; then
      "$DOCKER" rename dk-upload-rollback dk-upload >/dev/null 2>&1 || true
      "$DOCKER" start dk-upload >/dev/null 2>&1 || true
    elif [ "$HAD_CURRENT" -eq 1 ] && container_exists dk-upload; then
      "$DOCKER" start dk-upload >/dev/null 2>&1 || true
    fi
  fi

  case "$STAGE" in
    "$DEPLOY_ROOT"/.stage.*) rm -rf "$STAGE" ;;
  esac
  rm -f "$LOCK_DIR/pid"
  rmdir "$LOCK_DIR" 2>/dev/null || true
  exit "$STATUS"
}
trap cleanup 0
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

# Recover a previous deployment interrupted after the old container was
# renamed. If both names exist, keep a healthy current container; otherwise
# restore the last-known-good rollback before doing any new work.
if container_exists dk-upload-rollback; then
  if container_exists dk-upload && container_healthy dk-upload; then
    "$DOCKER" rm -f dk-upload-rollback >/dev/null
  else
    if container_exists dk-upload; then
      "$DOCKER" rm -f dk-upload >/dev/null
    fi
    "$DOCKER" rename dk-upload-rollback dk-upload
    "$DOCKER" start dk-upload >/dev/null
  fi
fi

VALIDATION_ROOT="$OPENPILOT_ROOT/.carrot-validation-v1"
STATE_ROOT="$DEPLOY_ROOT/state"
mkdir -p "$VALIDATION_ROOT" "$STATE_ROOT"
# These are the only two host directories visible to the unprivileged
# container, so a narrow recursive ownership repair cannot touch other NAS data.
chown -R "$RUN_UID:$RUN_GID" "$VALIDATION_ROOT" "$STATE_ROOT"
chmod 0750 "$VALIDATION_ROOT" "$STATE_ROOT"

STAGE="$(mktemp -d "$DEPLOY_ROOT/.stage.XXXXXX")"
RAW_BASE="https://raw.githubusercontent.com/$REPOSITORY/$REVISION/tools/carrot_upload_server"
for name in Dockerfile requirements.txt server.py __init__.py .dockerignore; do
  "$CURL" -fL --retry 3 --connect-timeout 15 --max-time 180 \
    "$RAW_BASE/$name" -o "$STAGE/$name"
done

IMAGE="dk-upload:$REVISION"
"$DOCKER" build --pull \
  --label "dk.openpilot.receiver=true" \
  --label "dk.openpilot.commit=$REVISION" \
  --tag "$IMAGE" \
  "$STAGE"

OLD_IMAGE=""
if container_exists dk-upload; then
  HAD_CURRENT=1
  OLD_IMAGE="$("$DOCKER" inspect --format '{{.Image}}' dk-upload)"
  "$DOCKER" stop -t 30 dk-upload >/dev/null
  "$DOCKER" rename dk-upload dk-upload-rollback
  OLD_RENAMED=1
fi

NEW_ATTEMPTED=1
"$DOCKER" run -d \
  --name dk-upload \
  --restart unless-stopped \
  --init \
  --user "$RUN_UID:$RUN_GID" \
  --read-only \
  --tmpfs /tmp:size=64m \
  --cap-drop ALL \
  --security-opt no-new-privileges:true \
  --memory 512m \
  --memory-swap 512m \
  --cpus 1.5 \
  --pids-limit 128 \
  --ulimit nofile=1024:4096 \
  --log-driver json-file \
  --log-opt max-size=10m \
  --log-opt max-file=3 \
  --publish 127.0.0.1:18080:8080 \
  --volume "$VALIDATION_ROOT:/data/openpilot/.carrot-validation-v1:rw" \
  --volume "$STATE_ROOT:/data/state:rw" \
  --env CARROT_UPLOAD_ROOT=/data/openpilot \
  --env CARROT_UPLOAD_DB=/data/state/uploads.sqlite3 \
  --env "CARROT_ALLOWED_DEVICE_IDS=$DEVICE_ID" \
  --env "CARROT_ALLOWED_DEVICE_PUBLIC_KEY_SHA256=$DEVICE_ID=$DEVICE_PUBLIC_KEY_SHA256" \
  --env CARROT_LEGACY_UPLOADS_ENABLED=false \
  --env CARROT_DAILY_DEVICE_QUOTA_BYTES=1073741824 \
  --env CARROT_DAILY_IP_QUOTA_BYTES=8589934592 \
  --env CARROT_MAX_FILE_BYTES=536870912 \
  --env CARROT_MIN_FREE_BYTES=10737418240 \
  --env CARROT_VALIDATION_FREE_SPACE_RESERVE_BYTES=1073741824 \
  --env CARROT_SESSION_TTL_SECONDS=14400 \
  --env CARROT_SESSION_RATE_BUCKET_LIMIT=4096 \
  --env CARROT_REQUEST_CONCURRENT_PER_IP=4 \
  --env CARROT_REQUEST_CONCURRENT_GLOBAL=16 \
  --env CARROT_VALIDATION_REQUEST_CONCURRENT_PER_IP=4 \
  --env CARROT_VALIDATION_REQUEST_CONCURRENT_GLOBAL=16 \
  --env CARROT_CONCURRENT_PER_DEVICE=3 \
  --env CARROT_CONCURRENT_GLOBAL=16 \
  --env CARROT_VALIDATION_CONCURRENT_PER_DEVICE=2 \
  --env CARROT_VALIDATION_CONCURRENT_GLOBAL=8 \
  --env CARROT_UPLOAD_IDLE_TIMEOUT_SECONDS=60 \
  --env CARROT_UPLOAD_TOTAL_TIMEOUT_SECONDS=21600 \
  --env CARROT_JSON_BODY_IDLE_TIMEOUT_SECONDS=10 \
  --env CARROT_JSON_BODY_TOTAL_TIMEOUT_SECONDS=30 \
  --env CARROT_PROGRESS_COMMIT_BYTES=1048576 \
  --env CARROT_PROGRESS_COMMIT_INTERVAL_SECONDS=1 \
  --env CARROT_VALIDATION_CHALLENGE_TTL_SECONDS=300 \
  --env CARROT_VALIDATION_SESSION_TTL_SECONDS=1800 \
  --env CARROT_VALIDATION_AUDIENCE=https://adot.synology.me/api/v1/validation \
  --env CARROT_VALIDATION_VERIFY_TIMEOUT_SECONDS=5 \
  --env CARROT_VALIDATION_VERIFY_CONCURRENT=4 \
  --env CARROT_VALIDATION_VERIFY_ATTEMPT_LIMIT=5 \
  --env CARROT_VALIDATION_VERIFY_COOLDOWN_SECONDS=2 \
  --env CARROT_VALIDATION_HASH_CONCURRENT=2 \
  --env CARROT_RESERVATION_LEASE_SECONDS=7200 \
  --env CARROT_STALE_PART_SECONDS=7200 \
  --env CARROT_CLEANUP_INTERVAL_SECONDS=900 \
  "$IMAGE" >/dev/null

READY=0
ATTEMPT=0
while [ "$ATTEMPT" -lt 60 ]; do
  if container_healthy dk-upload; then
    READY=1
    break
  fi
  ATTEMPT=$((ATTEMPT + 1))
  sleep 2
done
if [ "$READY" -ne 1 ]; then
  "$DOCKER" logs --tail 100 dk-upload >&2 || true
  echo "dk-upload failed its internal storage/readiness check" >&2
  exit 1
fi

# Confirm that the host port, immutable image label, narrow mounts, and public
# protocol policy match the reviewed deployment rather than merely trusting a
# process-local health response.
"$CURL" -fsS --connect-timeout 3 --max-time 5 \
  http://127.0.0.1:18080/api/v1/health >/dev/null

LABEL_REVISION="$("$DOCKER" inspect --format '{{index .Config.Labels "dk.openpilot.commit"}}' dk-upload)"
PORT_BINDING="$("$DOCKER" inspect --format '{{(index (index .HostConfig.PortBindings "8080/tcp") 0).HostIp}}:{{(index (index .HostConfig.PortBindings "8080/tcp") 0).HostPort}}' dk-upload)"
ROOT_READ_ONLY="$("$DOCKER" inspect --format '{{.HostConfig.ReadonlyRootfs}}' dk-upload)"
RUN_AS="$("$DOCKER" inspect --format '{{.Config.User}}' dk-upload)"
MOUNTS="$("$DOCKER" inspect --format '{{range .Mounts}}{{printf "%s|%s|%t\n" .Source .Destination .RW}}{{end}}' dk-upload)"
CONTAINER_ENV="$("$DOCKER" inspect --format '{{range .Config.Env}}{{println .}}{{end}}' dk-upload)"
if [ "$LABEL_REVISION" != "$REVISION" ] \
  || [ "$PORT_BINDING" != "127.0.0.1:18080" ] \
  || [ "$ROOT_READ_ONLY" != "true" ] \
  || [ "$RUN_AS" != "$RUN_UID:$RUN_GID" ]; then
  echo "dk-upload container identity or isolation policy mismatch" >&2
  exit 1
fi
if [ "$(printf '%s\n' "$MOUNTS" | sed '/^$/d' | wc -l | tr -d ' ')" -ne 2 ] \
  || ! printf '%s\n' "$MOUNTS" | grep -Fqx "$VALIDATION_ROOT|/data/openpilot/.carrot-validation-v1|true" \
  || ! printf '%s\n' "$MOUNTS" | grep -Fqx "$STATE_ROOT|/data/state|true"; then
  echo "dk-upload mount policy mismatch" >&2
  exit 1
fi
if ! printf '%s\n' "$CONTAINER_ENV" | grep -Fqx "CARROT_ALLOWED_DEVICE_IDS=$DEVICE_ID" \
  || ! printf '%s\n' "$CONTAINER_ENV" | grep -Fqx \
    "CARROT_ALLOWED_DEVICE_PUBLIC_KEY_SHA256=$DEVICE_ID=$DEVICE_PUBLIC_KEY_SHA256" \
  || ! printf '%s\n' "$CONTAINER_ENV" | grep -Fqx "CARROT_LEGACY_UPLOADS_ENABLED=false"; then
  echo "dk-upload private device authentication policy mismatch" >&2
  exit 1
fi

CHALLENGE_BODY="$("$CURL" -fsS --connect-timeout 3 --max-time 10 \
  -H 'Content-Type: application/json' \
  --data "{\"deviceId\":\"$DEVICE_ID\"}" \
  http://127.0.0.1:18080/api/v1/validation/challenge)"
printf '%s' "$CHALLENGE_BODY" | grep -Fq '"challengeId"' || {
  echo "allowlisted validation challenge smoke test failed" >&2
  exit 1
}

DISALLOWED_STATUS="$("$CURL" -sS -o /dev/null -w '%{http_code}' --connect-timeout 3 --max-time 10 \
  -H 'Content-Type: application/json' \
  --data '{"deviceId":"not-this-dk-device"}' \
  http://127.0.0.1:18080/api/v1/validation/challenge)"
LEGACY_STATUS="$("$CURL" -sS -o /dev/null -w '%{http_code}' --connect-timeout 3 --max-time 10 \
  -H 'Content-Type: application/json' \
  --data "{\"deviceId\":\"$DEVICE_ID\",\"purpose\":\"test\"}" \
  http://127.0.0.1:18080/api/v1/session)"
if [ "$DISALLOWED_STATUS" != "403" ] || [ "$LEGACY_STATUS" != "403" ]; then
  echo "dk-upload fail-closed protocol smoke test failed" >&2
  exit 1
fi

# From this point the new container is the last-known-good deployment. A
# signal after COMMITTED leaves it running instead of rolling back unnecessarily.
COMMITTED=1
if container_exists dk-upload-rollback; then
  "$DOCKER" rm dk-upload-rollback >/dev/null
fi
if [ -n "$OLD_IMAGE" ] && [ "$OLD_IMAGE" != "$("$DOCKER" inspect --format '{{.Image}}' dk-upload)" ]; then
  "$DOCKER" image rm "$OLD_IMAGE" >/dev/null 2>&1 || true
fi

printf 'dk-upload is healthy at commit %s on 127.0.0.1:18080\n' "$REVISION"
