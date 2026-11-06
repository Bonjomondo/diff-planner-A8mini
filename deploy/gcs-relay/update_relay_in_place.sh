#!/usr/bin/env bash
# Update only the existing relay executable; keep the deployed service and HTTP split.
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
    echo "Run with sudo: sudo bash $0" >&2
    exit 1
fi

RELAY_SOURCE_DIR="$(cd "$(dirname "$0")" && pwd)"
RELAY_SOURCE="$RELAY_SOURCE_DIR/siyi_gimbal_relay.py"
RELAY_TARGET=/usr/local/bin/siyi_gimbal_relay.py
RELAY_SERVICE=siyi-gimbal-relay.service

python3 - "$RELAY_SOURCE" <<'PY'
import ast
import pathlib
import sys
source = pathlib.Path(sys.argv[1]).read_text(encoding="utf-8")
ast.parse(source)
if 'class RecentClients:' not in source or 'def run_udp_relay(' not in source:
    raise SystemExit("Expected the validated multi-client relay version")
PY

test -f "$RELAY_TARGET"
systemctl is-active --quiet "$RELAY_SERVICE"
RELAY_BACKUP_DIR="$(mktemp -d /var/backups/agri-gimbal-relay.XXXXXX)"
cp -p "$RELAY_TARGET" "$RELAY_BACKUP_DIR/siyi_gimbal_relay.py"
echo "Backup: $RELAY_BACKUP_DIR/siyi_gimbal_relay.py"
install -m 755 "$RELAY_SOURCE" "$RELAY_TARGET"

if ! systemctl restart "$RELAY_SERVICE" || ! systemctl is-active --quiet "$RELAY_SERVICE"; then
    install -m 755 "$RELAY_BACKUP_DIR/siyi_gimbal_relay.py" "$RELAY_TARGET"
    systemctl restart "$RELAY_SERVICE"
    echo "Relay update failed; restored previous executable." >&2
    exit 1
fi

cmp "$RELAY_SOURCE" "$RELAY_TARGET"
echo "Relay v4 installed; existing service options and media proxy were preserved."
systemctl --no-pager --full status "$RELAY_SERVICE"
