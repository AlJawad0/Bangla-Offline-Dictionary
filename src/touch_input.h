// 4-wire resistive touch for the 3.5" Method B panel.
//
// Ported from ../Touch Display/Touch Display.ino, which is where all of this
// was actually measured. Adafruit_TouchScreen is gone: it assumes a 10-bit AVR
// ADC, never sets ESP32 attenuation, and leaves its pins as inputs afterwards
// -- and these pins are the LCD's data bus, so leaving them floating corrupts
// the next blit.
//
// ---------------------------------------------------------------------------
// WHICH WIRE IS WHICH -- measured with !tpscan, not guessed
// ---------------------------------------------------------------------------
// The four touch wires ARE display bus lines, but NOT the ones the common
// mcufriend Uno-shield layout uses. On this panel they are D0/D1, not D6/D7:
//
//     Y+ = LCD_RS/DC = GPIO47      drive only, never read
//     X- = LCD_CS    = GPIO42      drive only, never read
//     Y- = LCD_D0    = GPIO1       ADC1_CH0  <- X position is sensed here
//     X+ = LCD_D1    = GPIO3       ADC1_CH2  <- Y position is sensed here
//
// v5 previously assumed D6/D7, which meant the X plate was never driven at all.
//
// ---------------------------------------------------------------------------
// WHY THESE PINS AND NO OTHERS
// ---------------------------------------------------------------------------
//   * WiFi runs permanently, so ADC2 (GPIO11..20) is dead -- reads return 0.
//     Both sense pins must be on ADC1 (GPIO1..10).
//   * The camera board hardwires GPIO4..18, so of ADC1 only 1, 2 and 3 are
//     free. GPIO2 drives the onboard LED, whose load clamps the top of the
//     range, so it is out. That leaves exactly GPIO1 and GPIO3.
//   * Never sense on the DC pin. Measured on two different GPIOs: after
//     tft.init(), analogRead() of whichever pin is TFT_DC returns 0 forever,
//     while the other end of the same floating plate reads correctly. It is a
//     property of being DC, not of the pin number.
//   * Y+ and X- are only ever driven, so they can be any pin.
//
// ---------------------------------------------------------------------------
// PRESENCE IS PRESSURE, NOT POSITION
// ---------------------------------------------------------------------------
// With no finger the sense nodes float up to ~3100-4095 on the LCD's pull-ups,
// and with a finger the position reading can be anywhere in 0..4095. So a
// threshold on position cannot detect contact. Contact is detected from the
// Adafruit pressure formula instead, read from the opposite ends of the plates:
//
//     no finger : z1 = 0..197      z2 = 4095    r = 4748
//     finger    : z1 = 139..1590   z2 = 3120    r = 157..206
//
// z1 is the real gate; the r window only rejects nonsense.
#pragma once

#include <stdint.h>

#ifndef BN_ENABLE_TOUCH
#define BN_ENABLE_TOUCH 1
#endif

// One debounced poll of the panel. `down` is the filtered contact state;
// `pressed`/`released` are its edges. `x,y` follow the finger; `pressX,pressY`
// stay at the position the press started from, taken as the median of the first
// three valid samples (the first sample alone is the noisiest, and using it
// picks the wrong button or none at all).
struct TouchState {
  bool down;
  bool pressed;
  bool released;
  int x, y;
  int pressX, pressY;
};

// ADC resolution and attenuation, which must be set BEFORE the panel is
// initialised. Call this ahead of lcdBegin().
void touchPreInit();

// Arm the panel. `wifiActive` tells it whether ADC2 is off limits. Call AFTER
// lcdBegin(): tft.init() drives these pins, and the idle levels are restored on
// top of that. Also loads any calibration saved in NVS.
bool touchBegin(bool wifiActive);

// True when touchBegin() armed successfully.
bool touchReady();

// Poll once and update `st`. Returns true when anything changed. Safe (and
// cheap) to call every loop; internally rate-limited.
bool touchUpdate(TouchState* st);

// Raw sample for calibration: returns false if nothing is pressed.
bool touchRaw(int* rx, int* ry, int* rz);

// The last z pair the driver measured, for the console's pressure readout.
void touchLastZ(int* z1, int* z2);

// Calibration bounds and the axis swap, settable at runtime from the serial
// console so the panel can be calibrated without a reflash.
void touchSetCalibration(int minx, int maxx, int miny, int maxy);
void touchGetCalibration(int* minx, int* maxx, int* miny, int* maxy, bool* swap);
void touchSetSwap(bool swap);
void touchSaveCalibration();

// Contact thresholds, also runtime-settable.
void touchSetThresholds(int z1floor, int maxr);
void touchGetThresholds(int* z1floor, int* maxr);

// Three-point on-screen calibration. `drawTarget` is called with the screen
// position of each crosshair and must render it and get it onto the panel --
// this module has no framebuffer of its own. Works out the axis swap and both
// inversions by itself and saves the result to NVS.
bool touchCalibrate(void (*drawTarget)(int x, int y, const char* msg));

// Put the four wires back to their idle output levels. Every read does this
// already; it is exported because anything else that borrows the bus (a pin
// test, say) has to do it too.
void touchReleaseBus();

// Print whether each touch pin's ADC responds to internal pull-up/pull-down.
// A pin that reads the same both ways is held by something external.
void touchAdcProbe(const char* tag);
