// Bangla text shaping + rendering for ESP32-S3.
//
// Everything here is a direct port of the Python model in
// tools/gen_bangla_font.py, which was verified glyph-for-glyph against
// HarfBuzz over 119,637 clusters with zero mismatches. If you change a rule
// here, change it there too and re-run the generator -- the generator's
// verification pass is what proves the two agree.
//
// No network, no font file, no shaping engine: the tables in bn_shape.h plus
// the complete 586-glyph atlas in bn_font.h are self-contained, so any Bangla
// word -- baked in or learned from the internet at runtime -- renders with no
// further downloads.
#pragma once

#include <stdint.h>
#include <stddef.h>

#include "bn_font.h"
#include "bn_shape.h"

// A long compound word shapes to well under this; calls clamp rather than overflow.
#define BN_MAX_RUN 192

// Shape UTF-8 (mixed English/Bangla) into a glyph run.
// Returns the number of glyphs written.
int bn_shape_text(const char* utf8, bn_run_t* out, int maxout);

// Total advance width in pixels.
int bn_run_width(const bn_run_t* run, int n);

// Blit a shaped run into an RGB565 framebuffer at pen position x and the given
// baseline, alpha-blending the 4bpp coverage against whatever is already there.
void bn_draw_run(uint16_t* fb, int fbw, int fbh, const bn_run_t* run, int n,
                 int x, int baseline, uint16_t color);

// Convenience: shape and draw in one call. Returns advance width.
int bn_draw_text(uint16_t* fb, int fbw, int fbh, const char* utf8,
                 int x, int baseline, uint16_t color);

// Verification path: blit into an 8-bit grayscale buffer (255 = white) using
// exactly the arithmetic tools/mirror.py uses, so device and PC output can be
// compared pixel for pixel rather than merely "looking right".
void bn_draw_run_gray(uint8_t* buf, int w, int h, const bn_run_t* run, int n,
                      int x, int baseline);
