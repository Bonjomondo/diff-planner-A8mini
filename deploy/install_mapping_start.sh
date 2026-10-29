#!/usr/bin/env bash
# Portable, per-user deployment. No login name or repository path is baked in.
set -euo pipefail
WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
profile_args=()
if (( $# )); then
    [[ $# == 2 && "$1" == --profile ]] || { echo 'Usage: install_mapping_start.sh [--profile NAME_OR_ABSOLUTE_YAML]' >&2; exit 1; }
    profile_args=(--profile "$2")
fi
STATE_DIR="${XDG_STATE_HOME:-$HOME/.local/state}/agri-gcs/mapping-start"
mkdir -p "$STATE_DIR"
PROFILE_DIR=$(python3 "$WORKSPACE/tools/prepare_mapping_profile.py" "${profile_args[@]}" --output-dir "$STATE_DIR/profiles")
PROFILE_FILE=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["profile"])' "$PROFILE_DIR/profile.json")
python3 -B -m unittest discover -s "$WORKSPACE/tests" -p test_mapping_start_service.py
python3 -B -m unittest discover -s "$WORKSPACE/tests" -p test_mapping_profiles.py
bash -n "$WORKSPACE/sh_files/start_mapping_only.sh"
systemctl --user show-environment >/dev/null
if systemctl is-active --quiet agri-mapping-start.service; then
    echo 'ERROR: A system-wide mapping API is running. Migrate it before installing a user API on port 37263.' >&2
    exit 1
fi
set +u
source /opt/ros/noetic/setup.bash
set -u
export ROS_MASTER_URI=http://127.0.0.1:11311
if timeout 3 rosnode list 2>/dev/null | grep -Fxq /laserMapping; then
    echo 'ERROR: Mapping is running. Stop it on the ground before replacing the startup service.' >&2
    exit 1
fi
python3 - "$WORKSPACE" "$PROFILE_FILE" <<'PY'
from pathlib import Path
import shutil
import sys

root=Path(sys.argv[1])
profile=sys.argv[2]
# systemd interprets percent specifiers even within quotes.
def quoted(value):
    value=str(value)
    if any(c in value for c in '\r\n\x00'):
        raise SystemExit('Invalid newline in deployment path/profile')
    return '"'+value.replace('%','%%').replace('\\','\\\\').replace('"','\\"')+'"'
unit=Path.home()/'.config/systemd/user/agri-mapping-start.service'
unit.parent.mkdir(parents=True,exist_ok=True)
if unit.exists():
    shutil.copy2(unit,unit.with_suffix('.service.before-profile'))
setting=('Environment='+quoted('AGRI_MAPPING_PROFILE='+profile)+'\n') if profile else ''
unit.write_text('''[Unit]
Description=Agri GCS sensor mapping startup API (aircraft profiles)
After=network.target
[Service]
Type=simple
Environment=PYTHONUNBUFFERED=1
Environment=ROS_MASTER_URI=http://127.0.0.1:11311
Environment=ROS_IP=127.0.0.1
'''+setting+'ExecStart=/usr/bin/python3 '+quoted(root/'tools/mapping_start_service.py')+' --script '+quoted(root/'sh_files/start_mapping_only.sh')+'''
Restart=on-failure
RestartSec=3
TimeoutStopSec=10
KillMode=control-group
NoNewPrivileges=true
[Install]
WantedBy=default.target
''')
print('Installed user service:',unit)
PY
systemctl --user daemon-reload
systemctl --user enable agri-mapping-start.service
systemctl --user restart agri-mapping-start.service
systemctl --user is-active agri-mapping-start.service
# Runs for this logged-in user. For service availability before login, an
# administrator can enable user lingering separately.
