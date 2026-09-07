#!/bin/bash
# One-time setup: sudo bash sh_files/install_kernel_log_access.sh q
# Authorize only these two read-only commands, never an editable project script.
set -euo pipefail
account="${1:-${SUDO_USER:-}}"
if [[ "$EUID" != 0 ]]; then
    echo "Run with sudo; this installs a root-owned sudoers rule." >&2
    exit 1
fi
if [[ ! "$account" =~ ^[a-z_][a-z0-9_-]*\$?$ ]] || [[ "$account" == root ]]; then
    echo "Supply a non-root local username." >&2
    exit 1
fi
id "$account" >/dev/null
test -x /usr/bin/dmesg
script_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
install -d -o root -g root -m 0755 /usr/local/libexec
install -o root -g root -m 0755 "$script_dir/a8mini_kernel_log_reader.py" /usr/local/libexec/a8mini-kernel-log-reader
rule="/etc/sudoers.d/a8mini-kernel-log-${account}"
temporary="$(mktemp /etc/sudoers.d/.a8mini-kernel-log.XXXXXX)"
trap 'rm -f "$temporary"' EXIT
printf '%s ALL=(root) NOPASSWD: /usr/bin/dmesg --time-format iso, /usr/local/libexec/a8mini-kernel-log-reader ""\n' "$account" > "$temporary"
chmod 0440 "$temporary"
chown root:root "$temporary"
/usr/sbin/visudo -cf "$temporary"
mv -f "$temporary" "$rule"
echo "Installed $rule (read-only dmesg; no password stored)."
