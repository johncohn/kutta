#!/usr/bin/env bash
# Installs a desktop-autostart entry that launches kutta on login, for a
# Raspberry Pi OS Desktop exhibit build. Run this from inside the kutta repo
# on the Pi itself, after `go build`.
set -euo pipefail

KUTTA_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

if [ ! -x "$KUTTA_DIR/kutta" ]; then
	echo "error: $KUTTA_DIR/kutta not found or not executable -- run 'go build' first" >&2
	exit 1
fi

mkdir -p ~/.config/autostart
sed "s|__KUTTA_DIR__|$KUTTA_DIR|g" "$KUTTA_DIR/deploy/pi-autostart/kutta.desktop" \
	> ~/.config/autostart/kutta.desktop

echo "Installed ~/.config/autostart/kutta.desktop (launches on next desktop login)."
echo
echo "To boot straight to the exhibit with no login screen, also enable desktop"
echo "auto-login:  sudo raspi-config  ->  System Options  ->  Boot / Auto Login  ->  Desktop Autologin"
