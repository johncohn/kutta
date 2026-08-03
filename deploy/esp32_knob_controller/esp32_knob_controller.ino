/*
 * kutta UDP knob controller -- Adafruit QT Py ESP32-S3 + 3x Arduino Modulino
 * Knob rotary encoders (STEMMA QT / Qwiic I2C, addresses 0x3A/0x3B/0x3C).
 *
 * Sends kutta's UDP control protocol (see udpcontrol.go) over multicast, so
 * no configuration is needed on the kutta side beyond -udp 239.192.1.1:9000
 * -- this is a pure sender, it never listens for anything back.
 *
 * Wiring: all three Knobs share the QT Py's single STEMMA QT bus (I2C), each
 * pre-configured to its own fixed address (0x3A/0x3B/0x3C) so they can
 * coexist -- see the Arduino Modulino address-setting sketch if yours are
 * still at the factory default. The I2C scan in setup() below will show
 * exactly which addresses are actually present on the bus, which is the
 * fastest way to tell whether that reconfiguration actually took.
 *
 * Mapping:
 *   Knob A (0x3A) rotate -> SPD   (inlet speed,      0.02 .. 0.15, matching
 *                                  kutta's own solver-stability range)
 *          press        -> cycles the field display mode locally, sends
 *                           MODE speed / MODE vorticity / MODE pressure
 *   Knob B (0x3B) rotate -> AOA   (angle of attack, -20 .. +20 degrees)
 *          press        -> toggles streamlines locally, sends STREAMLINES 0/1
 *   Knob C (0x3C) rotate -> CTRL  (control-surface deflection, -40 .. +40
 *                                  degrees -- inert if the loaded scene has
 *                                  no object marked Control; kutta ignores
 *                                  it safely either way)
 *          press        -> toggles particles locally, sends PARTICLES 0/1
 *
 * kutta's UDP protocol only takes explicit values (MODE <name>, STREAMLINES
 * 0|1, PARTICLES 0|1) -- it has no "cycle" or "toggle" channel, since a
 * receive-only protocol has no way to tell the button what kutta's current
 * state actually is. So each button's next state is tracked locally here,
 * the same way a physical toggle switch would.
 *
 * Each knob's raw encoder count is clamped (and written back with .set())
 * whenever it would carry the mapped value past its min/max, so the knob
 * gets a real physical "end stop" instead of silently continuing to count
 * while the value it drives sits pinned at the limit.
 */

#include <Arduino_Modulino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>

#include "wifi_secrets.h"

// -- Network: matches the Pi's kiosk autostart config and everything else
// this project has used all along. Sending to a multicast address needs no
// group join on the sender's side -- only receivers (kutta) join the group.
const IPAddress KUTTA_MULTICAST_ADDR(239, 192, 1, 1);
const uint16_t KUTTA_PORT = 9000;

const uint8_t ADDR_SPEED = 0x3A;
const uint8_t ADDR_AOA = 0x3B;
const uint8_t ADDR_CTRL = 0x3C;

ModulinoKnob knobSpeed(ADDR_SPEED);
ModulinoKnob knobAoa(ADDR_AOA);
ModulinoKnob knobCtrl(ADDR_CTRL);

WiFiUDP udp;

// -- Per-knob mapping: physical units per encoder detent, and the min/max
// each knob enforces on itself before kutta ever sees the value.
struct KnobRange {
  float step;
  float vmin;
  float vmax;
};

const KnobRange RANGE_SPEED = {0.005f, 0.02f, 0.15f};  // matches spdMin/spdMax in game.go
const KnobRange RANGE_AOA = {0.5f, -20.0f, 20.0f};
const KnobRange RANGE_CTRL = {1.0f, -40.0f, 40.0f};  // matches controlLimit in game.go

// Debounce: a real mechanical push-button bounces on contact, and
// isPressed()'s own edge detection isn't enough to fully absorb that -- one
// physical click can report as several separate presses in quick succession.
//
// A fixed cooldown after the last ACCEPTED press (the previous approach here)
// gets this wrong both ways: if a bounce burst happens to span longer than
// the cooldown, more than one of its edges gets accepted (a "skip" to the
// state two presses later); and it can also reject a genuinely fast
// deliberate second press from the user.
//
// This instead waits for quiet: every isPressed() edge resets a timer, and
// the press is only accepted once QUIET_MS has passed with no further edges
// -- so an entire bounce burst, however long or however many edges it has,
// always collapses into exactly one accepted press, and a real second press
// is only ever blocked by the (short) quiet window, not an arbitrary cooldown.
//
// Declared up here, right after KnobRange rather than down near loop() where
// it's used: the Arduino IDE auto-generates function prototypes and inserts
// them right after the includes, before any other code -- a prototype
// referencing a type declared later in the file fails to compile even though
// the function itself comes after the type's real definition.
struct Debounce {
  bool pending = false;
  unsigned long lastEdge = 0;
};

const unsigned long QUIET_MS = 40;

Debounce dbSpeed, dbAoa, dbCtrl;

// last value actually sent per knob, so we only send on a real change
// instead of flooding the network every loop iteration.
float lastSpeed = NAN;
float lastAoa = NAN;
float lastCtrl = NAN;

// Local state for the button actions -- see the file header for why this
// can't just be a stateless CYCLE/TOGGLE message.
const char *MODE_NAMES[] = {"speed", "vorticity", "pressure"};
const int MODE_COUNT = 3;
int modeIdx = 0;
bool streamlinesOn = false;
bool particlesOn = true;  // matches kutta's own default

void setup() {
  Serial.begin(115200);
  // The QT Py's Serial is native USB (not a UART bridge), so a fixed delay
  // isn't reliable for waiting until the host has actually reattached after
  // upload -- wait on Serial itself, with a timeout so it still boots fine
  // when nothing's listening (e.g. once this is unplugged from a computer
  // and just running on USB power at the exhibit).
  unsigned long waitStart = millis();
  while (!Serial && millis() - waitStart < 5000) {
    delay(10);
  }
  delay(200);  // brief settle so the monitor doesn't miss the very first line

  // The QT Py's STEMMA QT connector is on the second I2C peripheral (Wire1),
  // not the default Wire. ModulinoClass::begin() only defaults to Wire1
  // automatically on Arduino's own UNO R4 WiFi/Nano R4 boards -- on every
  // other board, including this one, it defaults to plain Wire, which is the
  // wrong bus here and the reason begin() reported success while get()/
  // isPressed() never actually saw the hardware. Passing Wire1 explicitly
  // routes every ModulinoKnob call (not just this diagnostic scan) onto the
  // bus the Knobs are actually wired to.
  Modulino.begin(Wire1);

  bool okSpeed = knobSpeed.begin();
  bool okAoa = knobAoa.begin();
  bool okCtrl = knobCtrl.begin();

  // Repeat the whole diagnostic block a few times with real gaps between
  // lines: a single fast burst (the I2C scan alone is ~127 lines with no
  // delay between them) is exactly what a freshly-reattached USB serial
  // monitor can drop before it's ready to capture. Spacing it out and
  // repeating it removes any doubt about whether it printed at all.
  for (int rep = 0; rep < 3; rep++) {
    Serial.println("--- diagnostics ---");
    scanI2C();
    Serial.print("knobSpeed.begin() (0x3A) = ");
    Serial.println(okSpeed);
    delay(50);
    Serial.print("knobAoa.begin()   (0x3B) = ");
    Serial.println(okAoa);
    delay(50);
    Serial.print("knobCtrl.begin()  (0x3C) = ");
    Serial.println(okCtrl);
    delay(1000);
  }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected, IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(0);  // ephemeral local port; we only ever send
}

// last raw encoder counts actually printed, so the debug line below only
// appears when something really changed -- easier to read than a constant
// stream, and just as conclusive: if it never prints again after the first
// line no matter how much you turn or click, nothing is being read at all.
int16_t lastRawSpeed = INT16_MIN;
int16_t lastRawAoa = INT16_MIN;
int16_t lastRawCtrl = INT16_MIN;

bool debouncedPress(ModulinoKnob &knob, Debounce &db) {
  if (knob.isPressed()) {
    db.pending = true;
    db.lastEdge = millis();
    return false; // wait for quiet before accepting
  }
  if (db.pending && millis() - db.lastEdge >= QUIET_MS) {
    db.pending = false;
    return true;
  }
  return false;
}

void loop() {
  int16_t rawSpeed = knobSpeed.get();
  int16_t rawAoa = knobAoa.get();
  int16_t rawCtrl = knobCtrl.get();
  if (rawSpeed != lastRawSpeed || rawAoa != lastRawAoa || rawCtrl != lastRawCtrl) {
    lastRawSpeed = rawSpeed;
    lastRawAoa = rawAoa;
    lastRawCtrl = rawCtrl;
    Serial.print("raw changed: speed=");
    Serial.print(rawSpeed);
    Serial.print(" aoa=");
    Serial.print(rawAoa);
    Serial.print(" ctrl=");
    Serial.println(rawCtrl);
  }

  float speed = knobToValue(knobSpeed, RANGE_SPEED);
  if (speed != lastSpeed) {
    sendMessage("SPD", speed);
    lastSpeed = speed;
  }
  if (debouncedPress(knobSpeed, dbSpeed)) {
    modeIdx = (modeIdx + 1) % MODE_COUNT;
    char buf[24];
    snprintf(buf, sizeof(buf), "MODE %s", MODE_NAMES[modeIdx]);
    sendRaw(buf);
  }

  float aoa = knobToValue(knobAoa, RANGE_AOA);
  if (aoa != lastAoa) {
    sendMessage("AOA", aoa);
    lastAoa = aoa;
  }
  if (debouncedPress(knobAoa, dbAoa)) {
    streamlinesOn = !streamlinesOn;
    char buf[24];
    snprintf(buf, sizeof(buf), "STREAMLINES %d", streamlinesOn ? 1 : 0);
    sendRaw(buf);
  }

  float ctrl = knobToValue(knobCtrl, RANGE_CTRL);
  if (ctrl != lastCtrl) {
    sendMessage("CTRL", ctrl);
    lastCtrl = ctrl;
  }
  if (debouncedPress(knobCtrl, dbCtrl)) {
    particlesOn = !particlesOn;
    char buf[24];
    snprintf(buf, sizeof(buf), "PARTICLES %d", particlesOn ? 1 : 0);
    sendRaw(buf);
  }

  // Tight enough that a brief isPressed() edge is very unlikely to land
  // entirely between two reads and go unseen -- the debounce above is what
  // actually collapses a bounce burst into one press, not this delay.
  delay(2);
}

// scanI2C lists every address that acks on the bus, so a misconfigured Knob
// (still at its factory-default address, or not wired at all) shows up
// immediately instead of silently doing nothing.
void scanI2C() {
  Serial.println("Scanning I2C bus...");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire1.beginTransmission(addr);
    if (Wire1.endTransmission() == 0) {
      Serial.print("  found device at 0x");
      Serial.println(addr, HEX);
      found++;
    }
    delay(2);  // spread the scan out so it can't burst past the monitor
  }
  Serial.print(found);
  Serial.println(" device(s) found.");
}

// knobToValue reads one knob's raw encoder count, scales it to physical
// units, and clamps both the returned value and (via .set()) the encoder's
// own raw count whenever it would go out of range -- so turning past a limit
// stops changing anything immediately, rather than requiring an equal amount
// of backward turning before the value becomes live again.
float knobToValue(ModulinoKnob &knob, const KnobRange &r) {
  int16_t raw = knob.get();
  float v = raw * r.step;
  if (v < r.vmin) {
    v = r.vmin;
    knob.set((int16_t)round(v / r.step));
  } else if (v > r.vmax) {
    v = r.vmax;
    knob.set((int16_t)round(v / r.step));
  }
  return v;
}

void sendMessage(const char *channel, float value) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%s %.4f", channel, value);
  sendRaw(buf);
}

void sendRaw(const char *line) {
  udp.beginPacket(KUTTA_MULTICAST_ADDR, KUTTA_PORT);
  udp.print(line);
  udp.print("\n");
  udp.endPacket();
  Serial.println(line);
}
