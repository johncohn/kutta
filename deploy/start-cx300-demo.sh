#!/usr/bin/env bash
# Standard demo config: CX300 wing, speed field display, particles off (clean
# streamlines-only look), streamlines on, warp 2 (faster-reading flow, same
# stable physics), UDP control on multicast (239.192.1.1, RFC 2365
# admin-scoped range) for the knob box. Works unmodified on the Mac or the Pi
# -- DISPLAY only matters on Linux and is left alone if already set (e.g. from
# a desktop login session), falling back to :0 for a bare SSH launch.
set -euo pipefail

KUTTA_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$KUTTA_DIR"

if [ ! -x "$KUTTA_DIR/kutta" ]; then
	echo "error: $KUTTA_DIR/kutta not found or not executable -- run 'go build' first" >&2
	exit 1
fi

export DISPLAY="${DISPLAY:-:0}"

exec ./kutta \
	-scene cx300wing.afoil \
	-kiosk \
	-mode speed \
	-particles=false \
	-streamlines \
	-warp 2 \
	-udp 239.192.1.1:9000
