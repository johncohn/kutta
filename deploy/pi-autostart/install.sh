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
	# autostart file and does not reliably run the XDG ~/.config/autostart
	# convention's lxsession-xdg-autostart step at real boot -- see
	# labwc-autostart's own comment for how that was confirmed. Installing
	# the XDG entry too, on top of this, risks two kutta processes racing
	# for the same UDP port on the rare boot where lxsession-xdg-autostart
	# does happen to run -- so this is the only autostart path installed
	# here, not an addition to it.
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
