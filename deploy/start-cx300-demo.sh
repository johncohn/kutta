#!/usr/bin/env bash
# Standard demo config: CX300 wing, speed field display, particles off (clean
# streamlines-only look), streamlines on, warp 2 (faster-reading flow, same
# stable physics), UDP control for the knob box. Works unmodified on the Mac or
# the Pi -- DISPLAY only matters on Linux and is left alone if already set
# (e.g. from a desktop login session), falling back to :0 for a bare SSH
# launch.
set -euo pipefail

# UDP control listen address. The multicast group (239.192.1.1, from RFC 2365's
# admin-scoped range) is the default because it needs no per-host
# configuration at all. Override it with a wildcard unicast bind on a network
# that drops multicast between its clients, as the exhibit's does:
#
#   KUTTA_UDP=:9000 ./deploy/start-cx300-demo.sh
#
# The knob box sends to both a multicast and a unicast destination, so only
# this end ever needs changing.
KUTTA_UDP="${KUTTA_UDP:-239.192.1.1:9000}"

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
	-udp "$KUTTA_UDP"
