#include "touch_input.h"

#if BN_ENABLE_TOUCH

#include <Arduino.h>
#include <Preferences.h>

// The touch wires are display bus lines. Take them from the panel's own pin
// defines so the two definitions can never disagree -- V2_Setup.h is the single
// place the Method B pinout lives.
#include "TFT_eSPI/V2_Setup.h"

#ifndef TFT_DC
#error "touch_input.cpp needs the TFT_* pin defines from src/TFT_eSPI/V2_Setup.h"
#endif

// See the header for why each of these is the pin it is.
static const int kYP = TFT_DC;   // gpio47, drive only
static const int kXM = TFT_CS;   // gpio42, drive only
static const int kYM = TFT_D0;   // gpio1,  ADC1_CH0 -- X sense
static const int kXP = TFT_D1;   // gpio3,  ADC1_CH2 -- Y sense

static const int kSenseX = kYM;
static const int kSenseY = kXP;

static const int kAdcBits = 12;
static const int kAdcMax = 4095;
static const int kSamples = 9;     // must be odd, we take the median
static const int kRxPlate = 300;   // X plate resistance in ohms
static const int kMinR = 20;

static const int kScrW = 320;
static const int kScrH = 480;

// Settle and drain are variables rather than constants so a bad panel can be
// swept from the console without a reflash.
static int g_settleUs = 100;
static bool g_drainOn = true;

// z1 floor is the real contact gate. Measured: 5000 samples with no finger were
// all 0, and the very lightest real contact produced 139. 80 sits below the
// lightest touch and far above the noise. The r ceiling only rejects nonsense;
// it deliberately does NOT try to reject false presses, because z1 already has.
static int g_z1Floor = 80;
static int g_maxR = 6000;

// Calibration measured on the Method B wiring. MIN > MAX on an axis is normal
// and means that axis is inverted -- map() handles it, so there is no separate
// invert flag.
static const int kCalVer = 2;
static bool g_swapXY = true;
static int g_minx = 360, g_maxx = 3435;
static int g_miny = 3901, g_maxy = 439;

static bool g_ready = false;
static int g_lastZ1 = 0, g_lastZ2 = 0;

static Preferences g_prefs;

static bool isAdc1(int gpio) { return gpio >= 1 && gpio <= 10; }

// ------------------------------------------------------------------ sampling

// Median, not mean. These plates share the LCD data bus, so an occasional
// sample comes back complete garbage; a mean lets that one sample drag the
// whole reading, a median throws it away. The first read after switching a pin
// is also discarded -- the sample-and-hold cap has not charged yet.
static int tpMedian(int pin) {
  int buf[kSamples];
  analogRead(pin);
  for (int i = 0; i < kSamples; i++) buf[i] = analogRead(pin);
  for (int i = 1; i < kSamples; i++) {
    int v = buf[i], j = i - 1;
    while (j >= 0 && buf[j] > v) { buf[j + 1] = buf[j]; j--; }
    buf[j + 1] = v;
  }
  return buf[kSamples / 2];
}

// Dump the plate capacitance. Without this the previous axis's charge is still
// sitting on the node and consecutive samples oscillate.
static void tpDrain(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(60);
  pinMode(pin, INPUT);
}

// THE ONE THAT MATTERS. Reading touch turns these four pins into inputs, but
// TFT_eSPI writes them as outputs straight into the GPIO registers. If they are
// not put back immediately the next draw call goes out on floating lines and
// the panel garbles or freezes.
void touchReleaseBus() {
  pinMode(kXP, OUTPUT);
  pinMode(kXM, OUTPUT);
  pinMode(kYP, OUTPUT);
  pinMode(kYM, OUTPUT);
  digitalWrite(kXM, HIGH);   // TFT_CS idle high
  digitalWrite(kYP, HIGH);   // TFT_DC idle high (data mode)
  digitalWrite(kXP, LOW);    // D1
  digitalWrite(kYM, LOW);    // D0
}

static bool tpReadRaw(int* rx, int* ry, int* rz) {
  // ---- X position: drive the X plate, listen on the Y plate
  pinMode(kYP, INPUT);
  pinMode(kYM, INPUT);
  pinMode(kXP, OUTPUT); digitalWrite(kXP, HIGH);
  pinMode(kXM, OUTPUT); digitalWrite(kXM, LOW);
  if (g_drainOn) tpDrain(kSenseX);
  delayMicroseconds(g_settleUs);
  const int x = tpMedian(kSenseX);

  // ---- Y position: drive the Y plate, listen on the X plate
  pinMode(kXP, INPUT);
  pinMode(kXM, INPUT);
  pinMode(kYP, OUTPUT); digitalWrite(kYP, HIGH);
  pinMode(kYM, OUTPUT); digitalWrite(kYM, LOW);
  if (g_drainOn) tpDrain(kSenseY);
  delayMicroseconds(g_settleUs);
  const int y = tpMedian(kSenseY);

  // ---- Z (pressure): X- low, Y+ high, read both free ends.
  // Adafruit's formula, but taken from the opposite ends, because one of the
  // ends it normally uses is TFT_DC and a DC pin always analogReads as 0.
  pinMode(kXM, OUTPUT); digitalWrite(kXM, LOW);
  pinMode(kYP, OUTPUT); digitalWrite(kYP, HIGH);
  pinMode(kXP, INPUT);
  pinMode(kYM, INPUT);
  delayMicroseconds(g_settleUs);
  const int z1 = tpMedian(kXP);
  const int z2 = tpMedian(kYM);

  touchReleaseBus();                  // non-negotiable

  g_lastZ1 = z1;
  g_lastZ2 = z2;

  if (rx) *rx = x;
  if (ry) *ry = y;
  if (z1 < g_z1Floor || z2 <= z1) { if (rz) *rz = 0; return false; }

  float r = (float)z2 / (float)z1 - 1.0f;
  r *= (float)x;
  r *= (float)kRxPlate;
  r /= (float)kAdcMax;
  if (r < 0) r = 0;
  const int ri = (int)r;
  if (rz) *rz = ri;
  return (ri >= kMinR && ri <= g_maxR);
}

bool touchRaw(int* rx, int* ry, int* rz) {
  if (!g_ready) return false;
  int x = 0, y = 0, z = 0;
  const bool hit = tpReadRaw(&x, &y, &z);
  if (rx) *rx = x;
  if (ry) *ry = y;
  if (rz) *rz = z;
  return hit;
}

void touchLastZ(int* z1, int* z2) {
  if (z1) *z1 = g_lastZ1;
  if (z2) *z2 = g_lastZ2;
}

// Raw sample -> screen pixel, honouring the axis swap and either inversion.
static bool tpGetPoint(int* sx, int* sy) {
  int rx = 0, ry = 0, rz = 0;
  if (!tpReadRaw(&rx, &ry, &rz)) return false;
  const int ax = g_swapXY ? ry : rx;
  const int ay = g_swapXY ? rx : ry;
  int x = map(ax, g_minx, g_maxx, 0, kScrW);
  int y = map(ay, g_miny, g_maxy, 0, kScrH);
  if (sx) *sx = constrain(x, 0, kScrW - 1);
  if (sy) *sy = constrain(y, 0, kScrH - 1);
  return true;
}

// ------------------------------------------------------------------- NVS

static void tpCalLoad() {
  if (!g_prefs.begin("touch", true)) return;
  const int ver = g_prefs.getInt("ver", 1);
  if (ver != kCalVer) {
    Serial.printf("TOUCH cal in NVS is v%d -- discarded, using Method B defaults\n", ver);
    g_prefs.end();
    return;
  }
  if (g_prefs.isKey("minx")) {
    g_minx = g_prefs.getInt("minx", g_minx);
    g_maxx = g_prefs.getInt("maxx", g_maxx);
    g_miny = g_prefs.getInt("miny", g_miny);
    g_maxy = g_prefs.getInt("maxy", g_maxy);
    g_swapXY = g_prefs.getBool("swap", g_swapXY);
    Serial.printf("TOUCH cal from NVS x=%d..%d y=%d..%d swap=%d\n", g_minx, g_maxx, g_miny,
                  g_maxy, (int)g_swapXY);
  }
  g_prefs.end();
}

void touchSaveCalibration() {
  if (!g_prefs.begin("touch", false)) return;
  g_prefs.putInt("ver", kCalVer);
  g_prefs.putInt("minx", g_minx);
  g_prefs.putInt("maxx", g_maxx);
  g_prefs.putInt("miny", g_miny);
  g_prefs.putInt("maxy", g_maxy);
  g_prefs.putBool("swap", g_swapXY);
  g_prefs.end();
  Serial.println("TOUCH cal saved to NVS");
}

void touchSetCalibration(int minx, int maxx, int miny, int maxy) {
  g_minx = minx; g_maxx = maxx; g_miny = miny; g_maxy = maxy;
  Serial.printf("TOUCH cal x=%d..%d y=%d..%d swap=%d\n", g_minx, g_maxx, g_miny, g_maxy,
                (int)g_swapXY);
}

void touchGetCalibration(int* minx, int* maxx, int* miny, int* maxy, bool* swap) {
  if (minx) *minx = g_minx;
  if (maxx) *maxx = g_maxx;
  if (miny) *miny = g_miny;
  if (maxy) *maxy = g_maxy;
  if (swap) *swap = g_swapXY;
}

void touchSetSwap(bool swap) { g_swapXY = swap; }

void touchSetThresholds(int z1floor, int maxr) {
  if (z1floor > 0) g_z1Floor = z1floor;
  if (maxr > 0) g_maxR = maxr;
  Serial.printf("TOUCH thresholds z1floor=%d maxr=%d\n", g_z1Floor, g_maxR);
}

void touchGetThresholds(int* z1floor, int* maxr) {
  if (z1floor) *z1floor = g_z1Floor;
  if (maxr) *maxr = g_maxR;
}

// ------------------------------------------------------------------- probe

static void tpProbePin(const char* name, int pin) {
  pinMode(pin, INPUT_PULLUP);
  delay(5);
  const int upd = digitalRead(pin);   // analogRead() moves the pin to the ADC,
  const int up = analogRead(pin);     // so the digital read has to come first
  pinMode(pin, INPUT_PULLDOWN);
  delay(5);
  const int dnd = digitalRead(pin);
  const int dn = analogRead(pin);
  pinMode(pin, INPUT);
  Serial.printf("  %-4s gpio%-2d  pullup adc=%4d dig=%d   pulldown adc=%4d dig=%d  %s\n", name,
                pin, up, upd, dn, dnd, (up > 3000 && dn < 1000) ? "OK" : "<-- SUSPECT");
}

void touchAdcProbe(const char* tag) {
  // All four hi-Z first: otherwise the plate's ~300 ohm shunts the internal
  // ~45k pull-up down and the test fails for no reason.
  pinMode(kXP, INPUT);
  pinMode(kXM, INPUT);
  pinMode(kYP, INPUT);
  pinMode(kYM, INPUT);
  delay(2);
  Serial.printf("ADCPROBE [%s]\n", tag);
  tpProbePin("YM", kYM);
  tpProbePin("XP", kXP);
  Serial.flush();
  pinMode(kYP, INPUT);
  pinMode(kXM, INPUT);
}

// -------------------------------------------------------------------- init

void touchPreInit() {
  analogReadResolution(kAdcBits);
  // Full 0..3.3 V. Without it the top of the range is clipped and touch dies
  // along one edge of the screen.
  //
  // Do NOT add analogSetPinAttenuation() here. On Arduino-ESP32 3.x, setting a
  // per-pin attenuation after the global one kills that channel outright and
  // analogRead() returns 0 for ever. That is what made a correctly wired pin
  // look dead: probe before this call reads 4095, probe after reads 0.
  analogSetAttenuation(ADC_11db);
}

bool touchBegin(bool wifiActive) {
  analogReadResolution(kAdcBits);

  const bool xOk = isAdc1(kSenseX);
  const bool yOk = isAdc1(kSenseY);
  Serial.printf("TOUCH pins yp=%d xm=%d ym=%d xp=%d | sense x=%d(%s) y=%d(%s)\n", kYP, kXM, kYM,
                kXP, kSenseX, xOk ? "ADC1" : "ADC2", kSenseY, yOk ? "ADC1" : "ADC2");

  if (wifiActive && !(xOk && yOk)) {
    // Arming here would just feed the UI zeros.
    Serial.println("TOUCH DISABLED: a sense pin is on ADC2 and WiFi is up.");
    Serial.println("TOUCH   ADC2 (GPIO11-20) cannot be read while WiFi runs on the S3.");
    Serial.println("TOUCH   Only GPIO1 and GPIO3 are free on ADC1 on this board.");
    g_ready = false;
    touchReleaseBus();
    return false;
  }

  touchReleaseBus();
  g_ready = true;
  tpCalLoad();
  Serial.printf("TOUCH ok cal x=%d..%d y=%d..%d swap=%d z1floor=%d maxr=%d\n", g_minx, g_maxx,
                g_miny, g_maxy, (int)g_swapXY, g_z1Floor, g_maxR);
  return true;
}

bool touchReady() { return g_ready; }

// ------------------------------------------------------------------ polling

bool touchUpdate(TouchState* st) {
  if (!st) return false;
  st->pressed = false;
  st->released = false;
  if (!g_ready) { st->down = false; return false; }

  // A full read is ~5 ms of ADC work; there is no point doing it faster than
  // the panel can be pressed, and the HTTP server needs the cycles.
  static uint32_t lastPollMs = 0;
  const uint32_t now = millis();
  if (now - lastPollMs < 8) return false;
  lastPollMs = now;

  static bool wasDown = false;
  static int8_t conf = 0;
  static int lastGoodX = 0, lastGoodY = 0;
  static int recX[3], recY[3];
  static uint8_t recN = 0;
  static uint32_t downSinceMs = 0;
  static int pressX = 0, pressY = 0;

  int x = 0, y = 0;
  const bool raw = tpGetPoint(&x, &y);
  if (raw) {
    lastGoodX = x;
    lastGoodY = y;
    if (recN < 3) { recX[recN] = x; recY[recN] = y; recN++; }
  }

  // CONFIDENCE INTEGRATOR, not a run of consecutive samples.
  //
  // "three empty samples in a row = released" does not work: lifting a finger
  // makes the panel chatter, so one stray valid sample resets the counter, the
  // release is never seen and `down` latches on for ever. After that every
  // later tap looks like a continuation rather than a new press, and the UI
  // goes dead. Integrating instead (+2 valid, -1 empty, clamped 0..8) survives
  // a dropped sample mid-press AND still decays to a release through chatter.
  //
  // At an 8 ms poll: press = 2 valid samples (~16 ms), release = 6 empty
  // samples (~48 ms).
  if (raw) { conf += 2; if (conf > 8) conf = 8; }
  else     { conf -= 1; if (conf < 0) conf = 0; }

  bool down = wasDown ? (conf > 2) : (conf >= 4);

  // Last-ditch unstick. If `down` ever latches, the whole UI is gone.
  if (down && wasDown && downSinceMs && now - downSinceMs > 5000) {
    Serial.println("TOUCH: stuck down for 5 s -- forcing release");
    conf = 0;
    down = false;
  }
  if (down) { x = lastGoodX; y = lastGoodY; }

  bool changed = false;
  if (down && !wasDown) {
    // Median of the first three valid samples. The very first sample is taken
    // while the finger is still settling and is the one most likely to land on
    // the wrong button -- or on none, losing the tap entirely.
    int sx = lastGoodX, sy = lastGoodY;
    if (recN >= 3) {
      int a = recX[0], b = recX[1], c = recX[2];
      sx = (a > b) ? ((b > c) ? b : ((a > c) ? c : a)) : ((a > c) ? a : ((b > c) ? c : b));
      a = recY[0]; b = recY[1]; c = recY[2];
      sy = (a > b) ? ((b > c) ? b : ((a > c) ? c : a)) : ((a > c) ? a : ((b > c) ? c : b));
    } else if (recN == 2) {
      sx = (recX[0] + recX[1]) / 2;
      sy = (recY[0] + recY[1]) / 2;
    }
    pressX = sx;
    pressY = sy;
    x = sx;
    y = sy;
    downSinceMs = now;
    st->pressed = true;
    changed = true;
  } else if (!down && wasDown) {
    downSinceMs = 0;
    st->released = true;
    changed = true;
  } else if (down) {
    changed = true;                 // moving finger: let the caller drag
  }

  if (!down) recN = 0;
  wasDown = down;

  st->down = down;
  st->x = down ? x : lastGoodX;
  st->y = down ? y : lastGoodY;
  st->pressX = pressX;
  st->pressY = pressY;
  return changed;
}

// ------------------------------------------------------------- calibration

// Wait for a still tap on one crosshair and return the raw pair behind it.
static bool tpCalPoint(int px, int py, void (*drawTarget)(int, int, const char*), int* orx,
                       int* ory) {
  drawTarget(px, py, "tap the cross");

  // First wait for the finger to come off whatever it was on.
  int clear = 0;
  uint32_t t0 = millis();
  while (clear < 20 && millis() - t0 < 5000) {
    int a, b, c;
    clear = tpReadRaw(&a, &b, &c) ? 0 : clear + 1;
    delay(5);
  }

  // Then wait for a run of steady contact.
  const int need = 6;
  int xs[need], ys[need];
  int n = 0;
  t0 = millis();
  while (n < need && millis() - t0 < 25000) {
    int a, b, c;
    if (tpReadRaw(&a, &b, &c)) { xs[n] = a; ys[n] = b; n++; }
    else n = 0;
    delay(5);
  }
  if (n < need) { Serial.println("TPCAL timeout -- no tap"); return false; }

  for (int i = 1; i < n; i++) {
    int v = xs[i], j = i - 1;
    while (j >= 0 && xs[j] > v) { xs[j + 1] = xs[j]; j--; }
    xs[j + 1] = v;
    v = ys[i]; j = i - 1;
    while (j >= 0 && ys[j] > v) { ys[j + 1] = ys[j]; j--; }
    ys[j + 1] = v;
  }
  *orx = xs[n / 2];
  *ory = ys[n / 2];
  Serial.printf("TPCAL point (%d,%d) -> raw %d,%d\n", px, py, *orx, *ory);
  return true;
}

bool touchCalibrate(void (*drawTarget)(int x, int y, const char* msg)) {
  if (!g_ready || !drawTarget) return false;
  const int m = 40;
  int p0x, p0y, p1x, p1y, p2x, p2y;

  Serial.println("TPCAL start -- three crosshairs, tap each one");
  if (!tpCalPoint(m, m, drawTarget, &p0x, &p0y) ||                     // top-left
      !tpCalPoint(kScrW - m, m, drawTarget, &p1x, &p1y) ||             // top-right
      !tpCalPoint(m, kScrH - m, drawTarget, &p2x, &p2y)) {             // bottom-left
    return false;
  }

  // Only screen X changes between P0 and P1, so whichever raw axis moved more
  // is the one tracking screen X.
  const int dRawX = abs(p1x - p0x), dRawY = abs(p1y - p0y);
  g_swapXY = (dRawY > dRawX);
  Serial.printf("TPCAL swap=%d (P0->P1 moved rawX %d, rawY %d)\n", (int)g_swapXY, dRawX, dRawY);

  const int a0 = g_swapXY ? p0y : p0x, a1 = g_swapXY ? p1y : p1x;
  const int b0 = g_swapXY ? p0x : p0y, b2 = g_swapXY ? p2x : p2y;
  if (a1 == a0 || b2 == b0) {
    Serial.println("TPCAL: points are not distinct, ignoring");
    return false;
  }

  const float ax = (float)(a1 - a0) / (float)(kScrW - 2 * m);
  g_minx = (int)(a0 - ax * m);
  g_maxx = (int)(a0 + ax * (kScrW - m));

  const float ay = (float)(b2 - b0) / (float)(kScrH - 2 * m);
  g_miny = (int)(b0 - ay * m);
  g_maxy = (int)(b0 + ay * (kScrH - m));

  Serial.printf("TPCAL done x=%d..%d y=%d..%d swap=%d\n", g_minx, g_maxx, g_miny, g_maxy,
                (int)g_swapXY);
  touchSaveCalibration();
  return true;
}

#else   // BN_ENABLE_TOUCH == 0

void touchPreInit() {}
bool touchBegin(bool) { return false; }
bool touchReady() { return false; }
bool touchUpdate(TouchState* st) {
  if (st) { st->down = st->pressed = st->released = false; }
  return false;
}
bool touchRaw(int*, int*, int*) { return false; }
void touchLastZ(int* a, int* b) { if (a) *a = 0; if (b) *b = 0; }
void touchSetCalibration(int, int, int, int) {}
void touchGetCalibration(int* a, int* b, int* c, int* d, bool* e) {
  if (a) *a = 0; if (b) *b = 0; if (c) *c = 0; if (d) *d = 0; if (e) *e = false;
}
void touchSetSwap(bool) {}
void touchSaveCalibration() {}
void touchSetThresholds(int, int) {}
void touchGetThresholds(int* a, int* b) { if (a) *a = 0; if (b) *b = 0; }
bool touchCalibrate(void (*)(int, int, const char*)) { return false; }
void touchReleaseBus() {}
void touchAdcProbe(const char*) {}

#endif
