#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
[[ "$(id -un)" == ly && "$WORKSPACE" == /home/ly/Documents/diff-planner-A8mini-gcs-mission ]] || {
    echo 'This deployment template targets ly and the independent GCS mission workspace.' >&2; exit 1;
}
install -d -m 700 "$HOME/.local/state/agri-gcs/mapping-start"
python3 -B -m unittest discover -s "$WORKSPACE/tests" -p test_mapping_start_service.py
bash -n "$WORKSPACE/sh_files/start_mapping_only.sh"
sudo install -m 644 "$WORKSPACE/deploy/agri-mapping-start.service" /etc/systemd/system/agri-mapping-start.service
sudo systemctl daemon-reload
sudo systemctl enable --now agri-mapping-start.service
systemctl is-active agri-mapping-start.service
# The API starts at boot but sensors wait for an explicit ground-station request.
