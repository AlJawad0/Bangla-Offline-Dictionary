// Push v5's framebuffer to the physical 3.5" 320x480 panel.
//
// Nothing about v5 changes to make this work. The UI already renders a complete
// 320x480 RGB565 framebuffer in PSRAM -- exactly the format and exactly the
// geometry TFT_eSPI's pushImage() wants -- so the whole panel driver is: init
// once, then blit `g_fb` after every repaint. The web preview at /api/lcd keeps
// reading the same buffer, so the browser and the panel cannot disagree.
//
// ---------------------------------------------------------------------------
// METHOD B WIRING -- the pinout no longer comes from build_opt.h
// ---------------------------------------------------------------------------
// The panel is wired to the right-hand GPIO bank of the Freenove ESP32-S3 cam
// board, because GPIO4..18 on the left are hardwired to the camera:
//
//     CS 42   DC/RS 47   WR 41   RST 48   RD tied to RST (never read back)
//     D0 1    D1 3       D2 19   D3 20    D4 21   D5 38   D6 39   D7 40
//
// Two consequences, both load-bearing:
//
//   1. The data bus straddles TWO GPIO banks (D0..D4 in GPIO_OUT, D5..D7 in
//      GPIO_OUT1). Stock TFT_eSPI 2.5.43 picks one bank from D0 and assumes all
//      eight pins live there, so D5..D7 never move, the init commands arrive
//      corrupt and the panel stays white. This sketch therefore compiles a
//      SKETCH-LOCAL TFT_eSPI (src/TFT_eSPI) whose parallel driver writes the
//      two banks as separate 32-bit masks and only then pulses WR. The pinout
//      is that copy's src/TFT_eSPI/V2_Setup.h -- NOT build_opt.h any more.
//
//   2. GPIO19/20 are the USB-Serial-JTAG pads. While the pad is owned by the
//      USB peripheral, pinMode()/digitalWrite() on those two pins do nothing,
//      so D2/D3 stick and the panel is white again. lcdBegin() releases the pad
//      before tft.init(); the board's console runs over the CH34x UART bridge,
//      so nothing is lost. Build with USB CDC On Boot DISABLED.
#pragma once

#include <stdint.h>

// Set to 0 to build the headless device again (web preview + serial only).
#ifndef BN_ENABLE_LCD
#define BN_ENABLE_LCD 1
#endif

// Bring the panel up. Safe to call when BN_ENABLE_LCD is 0 -- it does nothing.
// Returns false if the panel was not built in.
bool lcdBegin();

// True once lcdBegin() has run with the panel compiled in.
bool lcdReady();

// Blit a w*h RGB565 buffer to the panel at 0,0. Sizes must match the panel;
// a mismatch is ignored rather than drawn wrong.
void lcdPush(const uint16_t* fb, int w, int h);

// Microseconds the last lcdPush() took, for the per-stage latency line.
uint32_t lcdLastPushUs();

// Walk the panel through solid red/green/blue/white/black. A white screen that
// never changes means the controller received no command at all -- i.e. the bus
// is broken -- which is worth being able to see without a logic analyser.
void lcdColourTest();
