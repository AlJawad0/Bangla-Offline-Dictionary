#include "lcd_panel.h"

#if BN_ENABLE_LCD

#include <Arduino.h>
#include <SPI.h>

// SKETCH-LOCAL TFT_eSPI, not the installed library. See lcd_panel.h: the Method
// B data bus straddles two GPIO banks and the stock 2.5.43 parallel driver only
// writes one of them. This copy also carries the pinout, in V2_Setup.h.
#include "TFT_eSPI/TFT_eSPI.h"

#if defined(ARDUINO_ARCH_ESP32)
#include "soc/soc.h"
#if __has_include("soc/usb_serial_jtag_reg.h")
#include "soc/usb_serial_jtag_reg.h"
#endif
#endif

static TFT_eSPI tft = TFT_eSPI();
static bool g_ready = false;
static uint32_t g_pushUs = 0;

// D2/D3 are GPIO19/20, which boot owned by the USB-Serial-JTAG peripheral. A
// pad the USB block holds does not respond to pinMode()/digitalWrite() at all,
// so those two bus bits freeze and every command byte the panel receives is
// wrong. Has to run BEFORE tft.init(), or the library configures two pins that
// are still not ours.
static void releaseUsbJtagPads() {
#if defined(USB_SERIAL_JTAG_CONF0_REG) && defined(USB_SERIAL_JTAG_USB_PAD_ENABLE)
  CLEAR_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_USB_PAD_ENABLE);
  Serial.println("LCD usb-jtag pad released -> gpio19/20 are plain GPIO");
#else
  Serial.println("LCD WARN usb-jtag pad macros missing -- gpio19/20 may stay stuck");
#endif
  pinMode(19, OUTPUT);
  digitalWrite(19, LOW);
  pinMode(20, OUTPUT);
  digitalWrite(20, LOW);
}

bool lcdBegin() {
  releaseUsbJtagPads();

  tft.init();
  tft.setRotation(0);        // 0 = portrait 320x480
  tft.fillScreen(TFT_BLACK);

  // MUST be true on this 8-bit parallel path. Measured on hardware, not
  // reasoned about: with it false the panel rendered the tan CAM pane as
  // turquoise, amber Bangla as cyan and the dark navy selected row as yellow --
  // each one exactly what byte-swapped RGB565 predicts. Pure white and pure
  // black are swap-invariant, which is why the English text looked correct and
  // hid the fault.
  tft.setSwapBytes(true);

  // Splash, drawn with TFT primitives. This is a diagnostic: if this appears
  // but the UI does not, the panel and pinout are fine and the fault is in the
  // framebuffer blit. If the screen stays white, the panel never initialised --
  // wrong pins, a stale library build, or the USB-JTAG pad still held.
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2);
  tft.drawString("Bangla lookup", tft.width() / 2, tft.height() / 2 - 20);
  tft.drawString("starting...", tft.width() / 2, tft.height() / 2 + 10);

  g_ready = true;
  Serial.printf("LCD ok %dx%d rot=0 swapbytes=1 cs=%d dc=%d wr=%d rst=%d\n", tft.width(),
                tft.height(), TFT_CS, TFT_DC, TFT_WR, TFT_RST);
  return true;
}

bool lcdReady() { return g_ready; }

void lcdPush(const uint16_t* fb, int w, int h) {
  if (!g_ready || !fb) return;
  // A geometry mismatch means the panel setup and the UI disagree; draw nothing
  // rather than something misaligned, and say so once.
  if (w != tft.width() || h != tft.height()) {
    static bool warned = false;
    if (!warned) {
      warned = true;
      Serial.printf("LCD SIZE MISMATCH fb=%dx%d panel=%dx%d -- not drawing\n", w, h,
                    tft.width(), tft.height());
    }
    return;
  }
  uint32_t t0 = micros();
  tft.pushImage(0, 0, w, h, (uint16_t*)fb);
  g_pushUs = micros() - t0;
}

uint32_t lcdLastPushUs() { return g_pushUs; }

void lcdColourTest() {
  if (!g_ready) return;
  struct {
    uint16_t c;
    const char* n;
  } steps[] = {{TFT_RED, "RED"},   {TFT_GREEN, "GREEN"}, {TFT_BLUE, "BLUE"},
               {TFT_WHITE, "WHITE"}, {TFT_BLACK, "BLACK"}};
  for (auto& s : steps) {
    tft.fillScreen(s.c);
    Serial.printf("LCDTEST %s\n", s.n);
    delay(350);
  }
}

#else   // BN_ENABLE_LCD == 0 -- headless, web preview and serial only

bool lcdBegin() { return false; }
bool lcdReady() { return false; }
void lcdPush(const uint16_t*, int, int) {}
uint32_t lcdLastPushUs() { return 0; }
void lcdColourTest() {}

#endif
