#!/usr/bin/env bash
# installs udev rules so the Luxonis OAK camera (Movidius MyriadX, USB VID 03e7)
# plug in camera, run script as sudo, unplug replug camera.
# Usage: sudo ./scripts/setup_udev_rules.sh

set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo "This script must be run as root, e.g.: sudo $0" >&2
    exit 1
fi

RULES_FILE="/etc/udev/rules.d/80-movidius.rules"

cat > "$RULES_FILE" <<'EOF'
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"
EOF

udevadm control --reload-rules
udevadm trigger

echo "Installed $RULES_FILE"
echo "Unplug and replug the camera's USB cable for the new permissions to apply."
