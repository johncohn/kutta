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

if [ -d ~/.config/labwc ]; then
	# labwc (Raspberry Pi OS Bookworm's default compositor) has its own
	# autostart file, and runs it in addition to /etc/xdg/labwc/autostart --
	# whose last line is lxsession-xdg-autostart, i.e. the XDG
	# ~/.config/autostart path runs on every boot, not rarely. Installing
	# the XDG entry as well would therefore start a second kutta every
	# time, two of them racing for the same UDP port (observed on the
	# exhibit Pi: two full simulations at once, and the instance that won
	# the port was not the one on screen). So this is the only autostart
	# path installed here, and any stale XDG entry is removed.
	rm -f ~/.config/autostart/kutta.desktop
	mkdir -p ~/.config/labwc
	sed "s|__KUTTA_DIR__|$KUTTA_DIR|g" "$KUTTA_DIR/deploy/pi-autostart/labwc-autostart" \
		> ~/.config/labwc/autostart
	chmod +x ~/.config/labwc/autostart
	echo "Installed ~/.config/labwc/autostart (launches on next labwc login)."
else
	mkdir -p ~/.config/autostart
	sed "s|__KUTTA_DIR__|$KUTTA_DIR|g" "$KUTTA_DIR/deploy/pi-autostart/kutta.desktop" \
		> ~/.config/autostart/kutta.desktop
	echo "Installed ~/.config/autostart/kutta.desktop (launches on next desktop login)."
fi

echo
echo "To boot straight to the exhibit with no login screen, also enable desktop"
echo "auto-login:  sudo raspi-config  ->  System Options  ->  Boot / Auto Login  ->  Desktop Autologin"
