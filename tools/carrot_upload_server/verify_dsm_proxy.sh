#!/bin/sh

# Verify the dedicated DSM reverse-proxy entry without following redirects.
# LAN mode proves the 18443 SNI/certificate/backend path. External mode must be
# run from a genuinely separate network and also checks that auxiliary ports
# were not forwarded from the WAN.

set -eu

PATH=/usr/bin:/bin:/usr/sbin:/sbin
export PATH

HOST=adot.synology.me
SOURCE_PORT=18443
MODE="${DK_UPLOAD_VERIFY_MODE:-}"
DEVICE_ID="${DK_UPLOAD_ALLOWED_DEVICE_ID:-}"
NAS_IP="${DK_UPLOAD_NAS_IP:-}"

if [ -x /usr/bin/curl ]; then
  CURL=/usr/bin/curl
elif [ -x /bin/curl ]; then
  CURL=/bin/curl
else
  echo "curl is required" >&2
  exit 1
fi

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

case "$MODE" in
  lan)
    case "$NAS_IP" in
      *[!0-9.]*|"")
        echo "LAN mode requires DK_UPLOAD_NAS_IP as an IPv4 address" >&2
        exit 2
        ;;
    esac
    BASE_URL="https://$HOST:$SOURCE_PORT"
    ;;
  external)
    if [ "${DK_UPLOAD_CONFIRM_EXTERNAL:-}" != "yes" ]; then
      echo "external mode must run off the NAS LAN with DK_UPLOAD_CONFIRM_EXTERNAL=yes" >&2
      exit 2
    fi
    BASE_URL="https://$HOST"
    ;;
  *)
    echo "DK_UPLOAD_VERIFY_MODE must be lan or external" >&2
    exit 2
    ;;
esac

VERIFY_ROOT="$(mktemp -d /tmp/dk-upload-verify.XXXXXX)"
cleanup() {
  case "$VERIFY_ROOT" in
    /tmp/dk-upload-verify.*) rm -rf "$VERIFY_ROOT" ;;
  esac
}
trap cleanup 0
trap 'exit 129' HUP
trap 'exit 130' INT
trap 'exit 143' TERM

run_curl() {
  if [ "$MODE" = "lan" ]; then
    "$CURL" --resolve "$HOST:$SOURCE_PORT:$NAS_IP" "$@"
  else
    "$CURL" "$@"
  fi
}

request() {
  REQUEST_NAME="$1"
  EXPECTED_STATUS="$2"
  METHOD="$3"
  PATH_SUFFIX="$4"
  JSON_BODY="${5:-}"
  BODY_FILE="$VERIFY_ROOT/$REQUEST_NAME.json"

  if [ "$METHOD" = "GET" ]; then
    STATUS_CODE="$(run_curl -sS -o "$BODY_FILE" -w '%{http_code}' \
      --connect-timeout 5 --max-time 20 "$BASE_URL$PATH_SUFFIX")" || {
        echo "$REQUEST_NAME request could not reach the proxy with valid TLS" >&2
        exit 1
      }
  else
    STATUS_CODE="$(run_curl -sS -o "$BODY_FILE" -w '%{http_code}' \
      --connect-timeout 5 --max-time 20 -X "$METHOD" \
      -H 'Content-Type: application/json' --data "$JSON_BODY" \
      "$BASE_URL$PATH_SUFFIX")" || {
        echo "$REQUEST_NAME request could not reach the proxy with valid TLS" >&2
        exit 1
      }
  fi
  if [ "$STATUS_CODE" != "$EXPECTED_STATUS" ]; then
    echo "$REQUEST_NAME returned HTTP $STATUS_CODE; expected $EXPECTED_STATUS" >&2
    exit 1
  fi
}

request health 200 GET /api/v1/health
for REQUIRED_FIELD in \
  '"ok"[[:space:]]*:[[:space:]]*true' \
  '"service"[[:space:]]*:[[:space:]]*"dk-upload"' \
  '"deviceAllowlistConfigured"[[:space:]]*:[[:space:]]*true' \
  '"deviceKeyPinsConfigured"[[:space:]]*:[[:space:]]*true' \
  '"legacyUploadsEnabled"[[:space:]]*:[[:space:]]*false' \
  '"storageWritable"[[:space:]]*:[[:space:]]*true'
do
  if ! grep -Eq "$REQUIRED_FIELD" "$VERIFY_ROOT/health.json"; then
    echo "health response does not match the private validation-only policy" >&2
    exit 1
  fi
done

request challenge 200 POST /api/v1/validation/challenge \
  "{\"deviceId\":\"$DEVICE_ID\"}"
for REQUIRED_FIELD in challengeId nonce audience expiresAt; do
  if ! grep -Eq "\"$REQUIRED_FIELD\"[[:space:]]*:" "$VERIFY_ROOT/challenge.json"; then
    echo "challenge response is missing $REQUIRED_FIELD" >&2
    exit 1
  fi
done

request disallowed 403 POST /api/v1/validation/challenge \
  '{"deviceId":"not-this-dk-device"}'
request legacy 403 POST /api/v1/session \
  "{\"deviceId\":\"$DEVICE_ID\",\"purpose\":\"test\"}"

if [ "$MODE" = "lan" ]; then
  if "$CURL" -sS --connect-timeout 2 --max-time 3 \
    "http://$NAS_IP:18080/api/v1/health" >/dev/null 2>&1; then
    echo "port 18080 is reachable beyond NAS loopback; stop and fix the container binding" >&2
    exit 1
  fi
else
  # These are reachability probes only and carry no credentials. Ignore a
  # certificate mismatch so an accidentally exposed DSM listener cannot hide
  # behind its default certificate.
  for EXTERNAL_TARGET in \
    "https://$HOST:18443/" \
    "http://$HOST:18080/" \
    "http://$HOST:5000/" \
    "https://$HOST:5001/"
  do
    if "$CURL" -k -sS --connect-timeout 2 --max-time 3 \
      "$EXTERNAL_TARGET" >/dev/null 2>&1; then
      echo "unexpected WAN service exposure at $EXTERNAL_TARGET" >&2
      exit 1
    fi
  done
fi

printf 'dk-upload %s reverse-proxy checks passed at %s\n' "$MODE" "$BASE_URL"
