// Drawing primitives for the 320x480 portrait UI.
//
// This file exists so `bangla_text.{h,cpp}` can stay byte-for-byte the file v1
// shipped. Word rendering still goes through `bn_shape_text()`/`bn_draw_run()`
// unchanged; everything here is either a plain shape or a half-scale blit of
// the SAME 24 px atlas.
//
// Why half scale: the atlas has exactly one size (24 px), and the screen needs
// small chrome labels ("CAM", "TODAY", "1/7", button captions) next to
// full-size words. Rather than ship a second font, `ui_text_half()` box-filters
// each glyph 2x2 down to ~12 px. The 4bpp coverage in the atlas is already
// antialiased, so averaging four samples gives a legible grey rather than a
// broken-up bitmap -- and it works for the Bangla header label too, which a
// small ASCII font could not.
#pragma once

#include <stdint.h>

#include "bangla_text.h"

// ---- shapes
void ui_fill(uint16_t* fb, int fbw, int fbh, uint16_t c);
void ui_rect(uint16_t* fb, int fbw, int fbh, int x, int y, int w, int h, uint16_t c);
void ui_frame(uint16_t* fb, int fbw, int fbh, int x, int y, int w, int h, uint16_t c);
// Rounded-looking 1px frame: same as ui_frame with the four corner pixels left out.
void ui_frame_soft(uint16_t* fb, int fbw, int fbh, int x, int y, int w, int h, uint16_t c);
void ui_dashed(uint16_t* fb, int fbw, int fbh, int x, int y, int w, int h, uint16_t c,
               int dash, int gap);
void ui_hline(uint16_t* fb, int fbw, int fbh, int x, int y, int w, uint16_t c);
void ui_vline(uint16_t* fb, int fbw, int fbh, int x, int y, int h, uint16_t c);

// Small solid triangles for the nav arrows -- the font has no glyph for them,
// and an unmapped codepoint is dropped silently rather than boxed.
void ui_tri_up(uint16_t* fb, int fbw, int fbh, int cx, int cy, int r, uint16_t c);
void ui_tri_down(uint16_t* fb, int fbw, int fbh, int cx, int cy, int r, uint16_t c);
// Double chevron, used for "jump to newest".
void ui_chev_up2(uint16_t* fb, int fbw, int fbh, int cx, int cy, int r, uint16_t c);
// An X, for "clear history".
void ui_cross(uint16_t* fb, int fbw, int fbh, int cx, int cy, int r, uint16_t c);

// ---- mockup chrome
// Rounded-rectangle outline, `t` pixels thick, corner radius `r`. The mockup's
// buttons are 2px rounded outlines; on this panel a 1px saturated edge is the
// first thing the controller loses, and the button stops reading as a button
// from arm's length. The corners are stepped rather than swept -- at r=6 and
// t=2 that is what a real arc quantises to anyway.
void ui_frame_round(uint16_t* fb, int fbw, int fbh, int x, int y, int w, int h, int r, int t,
                    uint16_t c);

// Double chevron pointing down: the mockup's jump-to-newest glyph.
void ui_chev_down2(uint16_t* fb, int fbw, int fbh, int cx, int cy, int r, uint16_t c);

// Left arrow (BACK): a stem with a triangular head.
void ui_arrow_left(uint16_t* fb, int fbw, int fbh, int cx, int cy, int r, uint16_t c);

// Three stacked bars (HIST).
void ui_icon_menu(uint16_t* fb, int fbw, int fbh, int cx, int cy, int r, uint16_t c);

// A ring with a filled centre (CAPTURE).
void ui_icon_capture(uint16_t* fb, int fbw, int fbh, int cx, int cy, int r, uint16_t c);

// An open circle with an arrowhead (CLEAR / reset the preview).
void ui_icon_reload(uint16_t* fb, int fbw, int fbh, int cx, int cy, int r, uint16_t c);

// A ring with a diagonal handle (SEARCH). Same 2px ring band as the others, so
// it holds up on this panel at the same size.
void ui_icon_search(uint16_t* fb, int fbw, int fbh, int cx, int cy, int r, uint16_t c);

// A left-pointing arrow into a bar (BACKSPACE), for the on-screen keyboard.
void ui_icon_bksp(uint16_t* fb, int fbw, int fbh, int cx, int cy, int r, uint16_t c);

// ---- text
// Full size (24 px): straight through the untouched shaper and blitter.
int ui_text(uint16_t* fb, int fbw, int fbh, const char* utf8, int x, int baseline,
            uint16_t col);
int ui_width(const char* utf8);

// Half size (~12 px), for chrome labels.
int ui_text_half(uint16_t* fb, int fbw, int fbh, const char* utf8, int x, int baseline,
                 uint16_t col);
int ui_width_half(const char* utf8);

// Same as above but clipped to `maxw` pixels, ellipsised with ".." when it does
// not fit. Returns the width actually drawn.
int ui_text_clip(uint16_t* fb, int fbw, int fbh, const char* utf8, int x, int baseline,
                 int maxw, uint16_t col);
