#!/usr/bin/env bash
# Install the heatsoak Klipper plugin by symlinking it into klippy/extras.
#
# Usage:
#   ./install.sh
#
# Environment overrides:
#   KLIPPER_PATH=~/klipper        Klipper install root (default: ~/klipper)
#   KLIPPER_SERVICE=klipper       systemd service name (default: klipper)

set -eu

KLIPPER_PATH="${KLIPPER_PATH:-${HOME}/klipper}"
SERVICE_NAME="${KLIPPER_SERVICE:-klipper}"
EXTRAS_PATH="${KLIPPER_PATH}/klippy/extras"
PLUGIN_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PLUGIN_FILE="heatsoak.py"

if [ "$EUID" -eq 0 ]; then
    echo "error: do not run as root; run as your normal user." >&2
    exit 1
fi

if [ ! -d "$EXTRAS_PATH" ]; then
    echo "error: Klipper extras directory not found at $EXTRAS_PATH" >&2
    echo "       Set KLIPPER_PATH=... if Klipper is installed elsewhere." >&2
    exit 1
fi

if [ ! -f "$PLUGIN_DIR/$PLUGIN_FILE" ]; then
    echo "error: $PLUGIN_FILE not found next to install.sh" >&2
    exit 1
fi

echo "linking $PLUGIN_FILE -> $EXTRAS_PATH/"
ln -sf "$PLUGIN_DIR/$PLUGIN_FILE" "$EXTRAS_PATH/$PLUGIN_FILE"

if command -v systemctl >/dev/null 2>&1; then
    echo "restarting $SERVICE_NAME ..."
    sudo systemctl restart "$SERVICE_NAME"
else
    echo "systemctl not found; please restart Klipper manually."
fi

echo
echo "Installed. Add a [heatsoak] section to printer.cfg and reload."
