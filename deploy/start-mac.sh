#!/usr/bin/env bash
# Launch kutta on a Mac with the CX300 exhibit's tuned settings: the same look
# the Pi runs, plus the performance flags that were measured (not guessed) to
# matter, and UDP control so the knob box drives it.
#
# Every setting below is overridable from the environment, so the common
# variations need no editing:
#
#   KUTTA_UDP=:9000 ./deploy/start-mac.sh        # unicast, for a multicast-less LAN
#   KUTTA_TPS=60 KUTTA_SUBSTEPS=3 ...            # full fidelity (a Mac can afford it)
#   KUTTA_KIOSK=0 ./deploy/start-mac.sh          # windowed, for poking at the UI
#   KUTTA_DEBUG=1 ./deploy/start-mac.sh          # per-second fps/draw timings
set -euo pipefail

KUTTA_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$KUTTA_DIR"

# ---------------------------------------------------------------- settings --

# UDP control listen address. Multicast is the default here because it needs no
# per-host configuration and macOS passes it fine; the knob box always sends to
# this group. kutta binds either multicast or unicast, never both, so this has
# to match what the network actually carries:
#
#   239.192.1.1:9000  most networks (RFC 2365 admin-scoped group)
#   :9000             networks that drop multicast between clients, as the
#                     exhibit's enterprise WLAN does -- there the knob box's
#                     unicast copy is the one that arrives
KUTTA_UDP="${KUTTA_UDP:-239.192.1.1:9000}"

KUTTA_SCENE="${KUTTA_SCENE:-cx300wing.afoil}"
KUTTA_MODE="${KUTTA_MODE:-speed}"

# Knots shown at full knob/slider travel. 150 is the CX300's real max speed;
# the default of 25 is a generic placeholder.
KUTTA_MAX_KN="${KUTTA_MAX_KN:-150}"

# Seconds idle before the sim wanders on its own. 0 disables it.
KUTTA_DEMO="${KUTTA_DEMO:-30}"

# --- the performance flags, and why these values ---
#
# -particles=false is the single biggest saving, and it is also what makes
# -glow free: glow is only ever applied inside drawSmoke, which doesn't run
# with particles off, so there is nothing to gain by passing -glow=false.
# It also happens to be the look the exhibit wants -- clean streamlines
# against the field rather than smoke.
KUTTA_PARTICLES="${KUTTA_PARTICLES:-false}"

# Solver steps per displayed frame; default 3. Dropping to 2 cuts solver work
# by a third for no visible difference at these speeds.
KUTTA_SUBSTEPS="${KUTTA_SUBSTEPS:-2}"

# Simulation ticks per second; default 60. Halving it frees the CPU to draw
# more frames -- the flow speed is unchanged, since each tick just advances
# proportionally further. On a Pi this is what stops draw spikes overrunning
# the vsync budget; a Mac has headroom to spare and can run 60 if you'd
# rather have the finer temporal resolution.
KUTTA_TPS="${KUTTA_TPS:-30}"

# Kiosk hides every panel, toolbar and slider, which skips all of that drawing
# as well as looking right for a demo.
KUTTA_KIOSK="${KUTTA_KIOSK:-1}"

# Note on -warp: deliberately left at its default of 1. It multiplies
# simulation time so the flow reads quicker, but costs CPU linearly, so it is
# a visual-pacing choice rather than a performance one -- raise it only if the
# flow looks too slow AND there are frames to spare.

# ------------------------------------------------------------------ checks --

# kutta rejects a tps that doesn't divide evenly into substeps*60 (validTPS in
# tickrate.go), which otherwise shows up as a bare startup error. Catch it here
# with a message that says what the legal values actually are.
if (( (KUTTA_SUBSTEPS * 60) % KUTTA_TPS != 0 )) || (( KUTTA_TPS < 10 )) || (( KUTTA_TPS > 60 )); then
	echo "error: KUTTA_TPS=$KUTTA_TPS is invalid with KUTTA_SUBSTEPS=$KUTTA_SUBSTEPS" >&2
	printf 'valid tps for substeps=%s:' "$KUTTA_SUBSTEPS" >&2
	for t in 10 12 15 20 24 30 40 60; do
		(( (KUTTA_SUBSTEPS * 60) % t == 0 )) && printf ' %s' "$t" >&2
	done
	echo >&2
	exit 1
fi

if [ ! -f "$KUTTA_SCENE" ]; then
	echo "error: scene '$KUTTA_SCENE' not found in $KUTTA_DIR" >&2
	exit 1
fi

# Rebuild when any source is newer than the binary. Go's own build cache makes
# this near-instant when nothing changed, and it removes the standing risk of
# demoing a stale binary after a branch switch (which rewrites file mtimes, so
# "it looks recent" is not evidence of anything).
if [ ! -x ./kutta ] || [ -n "$(find . -name '*.go' -newer ./kutta -print -quit 2>/dev/null)" ]; then
	echo "building kutta..."
	go build -o kutta .
fi

# ------------------------------------------------------------------ launch --

args=(
	-scene "$KUTTA_SCENE"
	-mode "$KUTTA_MODE"
	-particles="$KUTTA_PARTICLES"
	-streamlines
	-label
	-max-kn "$KUTTA_MAX_KN"
	-demo "$KUTTA_DEMO"
	-tps "$KUTTA_TPS"
	-substeps "$KUTTA_SUBSTEPS"
	-udp "$KUTTA_UDP"
)
[ "$KUTTA_KIOSK" != "0" ] && args+=(-kiosk)
[ "${KUTTA_DEBUG:-0}" != "0" ] && args+=(-debug)

echo "kutta: scene=$KUTTA_SCENE tps=$KUTTA_TPS substeps=$KUTTA_SUBSTEPS particles=$KUTTA_PARTICLES udp=$KUTTA_UDP"
exec ./kutta "${args[@]}"
