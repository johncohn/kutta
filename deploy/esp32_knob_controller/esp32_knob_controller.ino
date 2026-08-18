/*
 * kutta UDP knob controller -- Adafruit QT Py ESP32-S3 + 3x Arduino Modulino
 * Knob rotary encoders (STEMMA QT / Qwiic I2C, addresses 0x3A/0x3B/0x3C).
 *
 * Sends kutta's UDP control protocol (see udpcontrol.go) over multicast, so
 * no configuration is needed on the kutta side beyond -udp 239.192.1.1:9000
 * -- this is a pure sender, it never listens for anything back.
 *
 * WiFi: copy wifi_secrets.h.example to wifi_secrets.h (gitignored, so real
 * credentials are never committed) and list one to three networks there. They
 * are tried strictly in order, so the exhibit's own network goes first and a
 * phone hotspot can sit behind it as a fallback -- see WIFI_NETWORKS below.
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
#include <esp_mac.h>

#include "wifi_secrets.h"

// -- WiFi networks, tried strictly in order until one associates: the
// exhibit's own network first, a phone hotspot as the fallback for setting up
// somewhere that network doesn't reach yet (or bench-debugging away from it).
// Whatever the box lands on, the Pi running kutta has to be on the same
// network too -- this is multicast on a single L2 segment, nothing routes.
//
// The SSID/password literals live in wifi_secrets.h, which is gitignored, so
// only the shape of this table is ever committed. They're #defines rather
// than const char* specifically so the #ifdefs below can compile out the
// fallback slots for a secrets file that only lists one network.
struct WiFiNetwork {
  const char *ssid;
  const char *password;
};

const WiFiNetwork WIFI_NETWORKS[] = {
    {WIFI_SSID_1, WIFI_PASSWORD_1},
#ifdef WIFI_SSID_2
    {WIFI_SSID_2, WIFI_PASSWORD_2},
#endif
#ifdef WIFI_SSID_3
    {WIFI_SSID_3, WIFI_PASSWORD_3},
#endif
};
const int WIFI_NETWORK_COUNT = sizeof(WIFI_NETWORKS) / sizeof(WIFI_NETWORKS[0]);

// Per-network association timeout. WiFi.begin() is asynchronous and
// WiFi.status() isn't a dependable "that SSID isn't here" signal across
// builds, so a timeout is what actually decides to move on to the next
// network -- long enough not to give up on a slow-but-present AP, short
// enough that falling through to the hotspot isn't a visible hang at boot.
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 12000;
const unsigned long WIFI_RECONNECT_TIMEOUT_MS = 8000;

// -- Network: matches the Pi's kiosk autostart config and everything else
// this project has used all along. Sending to a multicast address needs no
// group join on the sender's side -- only receivers (kutta) join the group.
const IPAddress KUTTA_MULTICAST_ADDR(239, 192, 1, 1);
const uint16_t KUTTA_PORT = 9000;

// Optional unicast destination, for networks that filter client-to-client
// multicast -- enterprise WLANs routinely do, which is the whole reason this
// exists. It holds the address of the machine running kutta, which needs a
// DHCP reservation first: a unicast target that moves on lease renewal is
// worse than no unicast target at all.
//
// Define it in wifi_secrets.h, not here. It's a per-installation address
// rather than anything about how this controller works, and wifi_secrets.h is
// already the gitignored home for site-specific settings -- so the committed
// sketch stays identical everywhere and one uncommitted file holds everything
// that differs between sites. Uncommenting it below works too, if you'd
// rather keep it all in one place:
//
//   #define KUTTA_UNICAST_IP "10.0.0.42"
//
// When it's set, every message goes out twice, once multicast and once
// unicast. The duplicate costs one extra small packet per knob change --
// nothing, since sends only happen on change -- and in exchange the firmware
// works whichever mode kutta is in, so moving the exhibit between a
// multicast-filtering network and a hotspot means changing kutta's -udp flag
// only, never reflashing this box:
//
//   kutta unicast:   -udp :9000                (wildcard bind, any of its IPs)
//   kutta multicast: -udp 239.192.1.1:9000
//
// kutta can only be in one of those modes at a time -- a multicast bind will
// not pick up unicast packets, or vice versa -- so sending to both is what
// decouples the two ends. Leave it undefined for multicast only.

const uint8_t ADDR_SPEED = 0x3A;
const uint8_t ADDR_AOA = 0x3B;
const uint8_t ADDR_CTRL = 0x3C;

ModulinoKnob knobSpeed(ADDR_SPEED);
ModulinoKnob knobAoa(ADDR_AOA);
ModulinoKnob knobCtrl(ADDR_CTRL);

WiFiUDP udp;

#ifdef KUTTA_UNICAST_IP
// Parsed once in setup() rather than on every send. unicastOk guards it so a
// typo'd KUTTA_UNICAST_IP degrades to multicast-only (with a loud serial
// complaint) instead of quietly firing every packet at 0.0.0.0, which is what
// a failed fromString() would otherwise leave behind.
IPAddress kuttaUnicastAddr;
bool unicastOk = false;
#endif

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

// staMacAddress returns this box's station MAC, formatted for a DHCP
// reservation or MAC allowlist.
//
// Deliberately not WiFi.macAddress(): that reads the MAC out of the WiFi
// driver, which reports all zeros until the driver has actually started, and
// WiFi.mode(WIFI_STA) alone doesn't get it there on the ESP32 Arduino core --
// so printing it before the first association attempt (the whole point, so
// it's legible when association is what's failing) yielded a useless
// "00:00:00:00:00:00". esp_read_mac() derives the same address straight from
// efuse and needs no driver at all, so it's correct at any point in setup().
String staMacAddress() {
  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1],
           mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

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

  WiFi.mode(WIFI_STA);
  // Printed before any association attempt, so the box's MAC is visible on
  // the serial monitor even if it never manages to associate to anything at
  // all -- which is exactly when you need it, e.g. to check it against a
  // router's DHCP reservation or MAC allowlist.
  Serial.print("MAC address: ");
  Serial.println(staMacAddress());

  // Keep retrying the whole list rather than giving up: at the exhibit
  // there's no console to intervene at, and an AP that isn't up yet when the
  // box powers on is the normal case, not an error.
  while (!connectWiFi(WIFI_CONNECT_TIMEOUT_MS)) {
    Serial.println("WiFi: no configured network reachable, starting over");
    delay(1000);
  }

#ifdef KUTTA_UNICAST_IP
  unicastOk = kuttaUnicastAddr.fromString(KUTTA_UNICAST_IP);
  if (!unicastOk) {
    Serial.print("KUTTA_UNICAST_IP is not a valid address, multicast only: ");
    Serial.println(KUTTA_UNICAST_IP);
  }
#endif

  udp.begin(0);  // ephemeral local port; we only ever send

  // Print the first status line here rather than waiting out the first
  // interval, so the monitor shows a complete picture immediately at boot.
  printStatus();
}

// last raw encoder counts actually printed, so the debug line below only
// appears when something really changed -- easier to read than a constant
// stream, and just as conclusive: if it never prints again after the first
// line no matter how much you turn or click, nothing is being read at all.
int16_t lastRawSpeed = INT16_MIN;
int16_t lastRawAoa = INT16_MIN;
int16_t lastRawCtrl = INT16_MIN;

// connectToNetwork makes one association attempt at one network and reports
// whether it took. A failed attempt disconnects before returning so the radio
// is left idle rather than half-associated going into the next network's
// begin().
bool connectToNetwork(const WiFiNetwork &n, unsigned long timeoutMs) {
  Serial.print("WiFi: trying \"");
  Serial.print(n.ssid);
  Serial.print("\"");
  WiFi.begin(n.ssid, n.password);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeoutMs) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    return false;
  }
  Serial.print("WiFi: connected to \"");
  Serial.print(n.ssid);
  Serial.print("\", IP: ");
  Serial.println(WiFi.localIP());

  // ESP32 WiFi modem-sleep (on by default) lets the radio doze between
  // beacons to save power -- for a device that's USB-powered and otherwise
  // idle except a steady trickle of small UDP sends, that's the single most
  // common cause of an ESP32 going quiet after working fine for a while:
  // the association degrades or drops with nothing visible on this side.
  // There's no battery to protect here, so it's simplest to just disable it.
  // Reasserted on every successful association rather than once in setup(),
  // so a reconnect can't come back up with power-save silently re-enabled.
  WiFi.setSleep(false);
  return true;
}

// connectWiFi walks WIFI_NETWORKS in order and returns on the first one that
// associates. Always restarting from the top is deliberate: if the box came up
// on the hotspot fallback, the first drop after the exhibit network appears
// moves it back onto the preferred network on its own.
bool connectWiFi(unsigned long perNetworkTimeoutMs) {
  for (int i = 0; i < WIFI_NETWORK_COUNT; i++) {
    if (connectToNetwork(WIFI_NETWORKS[i], perNetworkTimeoutMs)) return true;
  }
  return false;
}

// -- WiFi health: this sketch is a pure sender, so it never joins the
// multicast group and has no membership to lose (only a receiver, like
// kutta itself, needs to rejoin -- see udpControlSupervisor in
// udpcontrol.go). What it can lose is the WiFi association underneath it,
// and sendRaw() below has no way to notice that (endPacket()'s failure
// return is intentionally ignored, matching kutta's fire-and-forget UDP
// model) -- so the only reliable recovery is periodically checking
// WiFi.status() and, on a drop, redoing the connect and UDP socket setup
// from scratch, the same "full rebind rather than patch partial failure"
// approach udpControlSupervisor takes on the receiving end.
const unsigned long WIFI_CHECK_INTERVAL_MS = 2000;
unsigned long lastWifiCheck = 0;

void ensureWiFiConnected() {
  if (millis() - lastWifiCheck < WIFI_CHECK_INTERVAL_MS) return;
  if (WiFi.status() == WL_CONNECTED) {
    lastWifiCheck = millis();
    return;
  }

  // Only reached once the association is already down, so blocking the
  // knob-reading loop here costs nothing that wasn't already lost --
  // sendRaw() can't deliver anywhere until this succeeds anyway.
  Serial.println("WiFi: disconnected, reconnecting...");
  WiFi.disconnect();
  bool ok = connectWiFi(WIFI_RECONNECT_TIMEOUT_MS);
  // Stamped after the attempt, not before it: a full failed pass over the
  // list already takes far longer than the check interval, so stamping first
  // would turn "check every 2s" into a continuous back-to-back retry loop
  // whenever nothing at all is reachable.
  lastWifiCheck = millis();
  if (!ok) {
    Serial.println("WiFi: reconnect attempt failed, will retry");
    return;
  }
  // The old UDP socket was bound under the previous association; rebind it
  // too rather than assume it's still good after the interface flapped.
  udp.stop();
  udp.begin(0);
}

// -- Periodic status line. The boot banner scrolls off the serial monitor
// within seconds once the knobs start reporting, and what it says is exactly
// what you need while chasing a network problem: which address this box
// currently holds (it can change on any reconnect) and its MAC, for a DHCP
// reservation. Reprinting on an interval is cheaper than building a way to
// ask for it on demand, and works with any serial monitor.
//
// The mask and gateway are here for a specific reason: whether this box and
// the Pi are actually on the same segment is answerable from either end, and
// reading the mask off the sender is the half that doesn't need an ssh
// session -- matching first three octets prove nothing on their own.
const unsigned long STATUS_INTERVAL_MS = 10000;
unsigned long lastStatus = 0;

void printStatus() {
  bool up = WiFi.status() == WL_CONNECTED;
  Serial.print("STATUS  link=");
  Serial.print(up ? "up" : "DOWN");
  Serial.print("  ssid=");
  Serial.print(up ? WiFi.SSID() : String("-"));
  Serial.print("  ip=");
  Serial.print(WiFi.localIP());
  Serial.print("  mask=");
  Serial.print(WiFi.subnetMask());
  Serial.print("  gw=");
  Serial.print(WiFi.gatewayIP());
  Serial.print("  mac=");
  Serial.print(staMacAddress());
  Serial.print("  rssi=");
  Serial.print(WiFi.RSSI());
  Serial.print("dBm  sending_to=239.192.1.1");
#ifdef KUTTA_UNICAST_IP
  if (unicastOk) {
    Serial.print("+");
    Serial.print(kuttaUnicastAddr);
  }
#endif
  Serial.print(":9000  up=");
  Serial.print(millis() / 1000);
  Serial.println("s");
}

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
  ensureWiFiConnected();

  if (millis() - lastStatus >= STATUS_INTERVAL_MS) {
    lastStatus = millis();
    printStatus();
  }

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
  sendTo(KUTTA_MULTICAST_ADDR, line);
#ifdef KUTTA_UNICAST_IP
  if (unicastOk) sendTo(kuttaUnicastAddr, line);
#endif
  // Printed once, not once per destination: the serial log is there to show
  // what the knobs decided, and duplicating every line would just make a
  // change look like two changes.
  Serial.println(line);
}

// sendTo fires one message at one destination. endPacket()'s failure return
// stays ignored, matching kutta's fire-and-forget UDP model -- a dropped
// control message is corrected by the next one, and ensureWiFiConnected() is
// what handles the case where they're all dropping.
void sendTo(const IPAddress &dst, const char *line) {
  udp.beginPacket(dst, KUTTA_PORT);
  udp.print(line);
  udp.print("\n");
  udp.endPacket();
}
