#include "ui_draw.h"

#include <string.h>

// NOTE ON LINKAGE -- this is load-bearing.
//
// `bn_font.h` / `bn_shape.h` declare their tables `static const`, so every
// translation unit that *references* one gets its own copy. The atlas alone is
// 522 KB. This file therefore touches no table directly: it only uses the
// bn_run_t type, BN_MAX_RUN, and the functions exported by bangla_text.cpp, so
// the copies in this TU stay unreferenced and are dropped by -gc-sections.
//
// That is also why half-scale text is produced by rendering through the
// exported `bn_draw_run_gray()` into a scratch buffer and box-filtering the
// result, rather than by walking the glyph bitmaps directly. It keeps the atlas
// in exactly one TU and leaves `bangla_text.cpp` byte-for-byte v1's file.

static inline uint16_t blend(uint16_t dst, uint16_t src, uint32_t a /*0..255*/) {
  uint32_t ia = 255 - a;
  uint32_t dr = (dst >> 11) & 0x1F, dg = (dst >> 5) & 0x3F, db = dst & 0x1F;
  uint32_t sr = (src >> 11) & 0x1F, sg = (src >> 5) & 0x3F, sb = src & 0x1F;
  uint32_t r = (sr * a + dr * ia) / 255;
  uint32_t g = (sg * a + dg * ia) / 255;
  uint32_t b = (sb * a + db * ia) / 255;
  return (uint16_t)((r << 11) | (g << 5) | b);
}

static inline void px(uint16_t* fb, int fbw, int fbh, int x, int y, uint16_t c) {
  if (x < 0 || y < 0 || x >= fbw || y >= fbh) return;
  fb[(uint32_t)y * fbw + x] = c;
}

// ------------------------------------------------------------------- shapes

void ui_fill(uint16_t* fb, int fbw, int fbh, uint16_t c) {
  for (int i = 0; i < fbw * fbh; i++) fb[i] = c;
}

void ui_rect(uint16_t* fb, int fbw, int fbh, int x, int y, int w, int h, uint16_t c) {
  for (int yy = y; yy < y + h; yy++) {
    if (yy < 0 || yy >= fbh) continue;
    uint16_t* d = fb + (uint32_t)yy * fbw;
    for (int xx = x; xx < x + w; xx++) {
      if (xx < 0 || xx >= fbw) continue;
      d[xx] = c;
    }
  }
}

void ui_hline(uint16_t* fb, int fbw, int fbh, int x, int y, int w, uint16_t c) {
  ui_rect(fb, fbw, fbh, x, y, w, 1, c);
}

void ui_vline(uint16_t* fb, int fbw, int fbh, int x, int y, int h, uint16_t c) {
  ui_rect(fb, fbw, fbh, x, y, 1, h, c);
}

void ui_frame(uint16_t* fb, int fbw, int fbh, int x, int y, int w, int h, uint16_t c) {
  ui_hline(fb, fbw, fbh, x, y, w, c);
  ui_hline(fb, fbw, fbh, x, y + h - 1, w, c);
  ui_vline(fb, fbw, fbh, x, y, h, c);
  ui_vline(fb, fbw, fbh, x + w - 1, y, h, c);
}

void ui_frame_soft(uint16_t* fb, int fbw, int fbh, int x, int y, int w, int h, uint16_t c) {
  ui_hline(fb, fbw, fbh, x + 1, y, w - 2, c);
  ui_hline(fb, fbw, fbh, x + 1, y + h - 1, w - 2, c);
  ui_vline(fb, fbw, fbh, x, y + 1, h - 2, c);
  ui_vline(fb, fbw, fbh, x + w - 1, y + 1, h - 2, c);
}

void ui_dashed(uint16_t* fb, int fbw, int fbh, int x, int y, int w, int h, uint16_t c,
               int dash, int gap) {
  const int step = dash + gap;
  for (int i = 0; i < w; i++) {
    if (i % step < dash) {
      px(fb, fbw, fbh, x + i, y, c);
      px(fb, fbw, fbh, x + i, y + h - 1, c);
    }
  }
  for (int i = 0; i < h; i++) {
    if (i % step < dash) {
      px(fb, fbw, fbh, x, y + i, c);
      px(fb, fbw, fbh, x + w - 1, y + i, c);
    }
  }
}

void ui_tri_up(uint16_t* fb, int fbw, int fbh, int cx, int cy, int r, uint16_t c) {
  for (int i = 0; i <= r; i++) {
    ui_hline(fb, fbw, fbh, cx - i, cy + i - r / 2, 2 * i + 1, c);
  }
}

void ui_tri_down(uint16_t* fb, int fbw, int fbh, int cx, int cy, int r, uint16_t c) {
  // Widest row at the top, narrowing to the apex at the bottom. (Stepping y
  // the other way draws an up-arrow -- which is exactly what it did at first.)
  for (int i = 0; i <= r; i++) {
    ui_hline(fb, fbw, fbh, cx - r + i, cy - r / 2 + i, 2 * (r - i) + 1, c);
  }
}

void ui_chev_up2(uint16_t* fb, int fbw, int fbh, int cx, int cy, int r, uint16_t c) {
  for (int k = 0; k < 2; k++) {
    int oy = cy + k * (r + 1) - r / 2;
    for (int i = 0; i <= r; i++) {
      px(fb, fbw, fbh, cx - i, oy + i, c);
      px(fb, fbw, fbh, cx - i, oy + i + 1, c);
      px(fb, fbw, fbh, cx + i, oy + i, c);
      px(fb, fbw, fbh, cx + i, oy + i + 1, c);
    }
  }
}

void ui_cross(uint16_t* fb, int fbw, int fbh, int cx, int cy, int r, uint16_t c) {
  for (int i = -r; i <= r; i++) {
    px(fb, fbw, fbh, cx + i, cy + i, c);
    px(fb, fbw, fbh, cx + i + 1, cy + i, c);
    px(fb, fbw, fbh, cx + i, cy - i, c);
    px(fb, fbw, fbh, cx + i + 1, cy - i, c);
  }
}

// ------------------------------------------------------------ mockup chrome

void ui_frame_round(uint16_t* fb, int fbw, int fbh, int x, int y, int w, int h, int r, int t,
                    uint16_t c) {
  if (w <= 0 || h <= 0 || t <= 0) return;
  if (r * 2 > w) r = w / 2;
  if (r * 2 > h) r = h / 2;
  if (r < 0) r = 0;

  // Straight runs, stopping short of the corners.
  for (int k = 0; k < t; k++) {
    ui_hline(fb, fbw, fbh, x + r, y + k, w - 2 * r, c);
    ui_hline(fb, fbw, fbh, x + r, y + h - 1 - k, w - 2 * r, c);
    ui_vline(fb, fbw, fbh, x + k, y + r, h - 2 * r, c);
    ui_vline(fb, fbw, fbh, x + w - 1 - k, y + r, h - 2 * r, c);
  }
  if (r <= 0) return;

  // Corners as an annulus test over the r x r corner square. A stepped arc is
  // what the panel renders anyway, and a distance band is both obviously
  // correct and automatically the right thickness on the diagonal.
  const int outer = r * r;
  const int inner = (r - t) * (r - t);
  for (int j = 0; j < r; j++) {
    for (int i = 0; i < r; i++) {
      const int dx = r - i, dy = r - j;          // distance from the arc centre
      const int d = dx * dx + dy * dy;
      if (d > outer || d < inner) continue;
      px(fb, fbw, fbh, x + i, y + j, c);                         // top-left
      px(fb, fbw, fbh, x + w - 1 - i, y + j, c);                 // top-right
      px(fb, fbw, fbh, x + i, y + h - 1 - j, c);                 // bottom-left
      px(fb, fbw, fbh, x + w - 1 - i, y + h - 1 - j, c);         // bottom-right
    }
  }
}

void ui_chev_down2(uint16_t* fb, int fbw, int fbh, int cx, int cy, int r, uint16_t c) {
  for (int k = 0; k < 2; k++) {
    const int oy = cy + k * (r + 1) - r;
    for (int i = 0; i <= r; i++) {
      px(fb, fbw, fbh, cx - r + i, oy + i, c);
      px(fb, fbw, fbh, cx - r + i, oy + i + 1, c);
      px(fb, fbw, fbh, cx + r - i, oy + i, c);
      px(fb, fbw, fbh, cx + r - i, oy + i + 1, c);
    }
  }
}

void ui_arrow_left(uint16_t* fb, int fbw, int fbh, int cx, int cy, int r, uint16_t c) {
  // Apex on the LEFT, widening rightward, then a stem out of the base.
  for (int i = 0; i <= r; i++) ui_vline(fb, fbw, fbh, cx - r + i, cy - i, 2 * i + 1, c);
  ui_rect(fb, fbw, fbh, cx, cy - 1, r + 2, 2, c);
}

void ui_icon_menu(uint16_t* fb, int fbw, int fbh, int cx, int cy, int r, uint16_t c) {
  for (int k = -1; k <= 1; k++) ui_rect(fb, fbw, fbh, cx - r, cy + k * (r - 1) - 1, 2 * r, 2, c);
}

void ui_icon_capture(uint16_t* fb, int fbw, int fbh, int cx, int cy, int r, uint16_t c) {
  // Ring plus a filled core, drawn as distance tests rather than a Bresenham
  // circle: at r = 7 the ring has to be two pixels thick to survive the panel,
  // and a radius band is the shortest way to say that.
  const int r2o = r * r, r2i = (r - 2) * (r - 2), r2c = (r - 4) * (r - 4);
  for (int dy = -r; dy <= r; dy++) {
    for (int dx = -r; dx <= r; dx++) {
      const int d = dx * dx + dy * dy;
      if ((d <= r2o && d >= r2i) || d <= r2c) px(fb, fbw, fbh, cx + dx, cy + dy, c);
    }
  }
}

void ui_icon_reload(uint16_t* fb, int fbw, int fbh, int cx, int cy, int r, uint16_t c) {
  // A ring with the top-right quadrant left open, and an arrowhead in the gap.
  const int r2o = r * r, r2i = (r - 2) * (r - 2);
  for (int dy = -r; dy <= r; dy++) {
    for (int dx = -r; dx <= r; dx++) {
      if (dx > 0 && dy < 0) continue;            // the gap
      const int d = dx * dx + dy * dy;
      if (d <= r2o && d >= r2i) px(fb, fbw, fbh, cx + dx, cy + dy, c);
    }
  }
  for (int i = 0; i <= 3; i++) ui_hline(fb, fbw, fbh, cx + r - 3 - i, cy - r + i, 2 * i + 1, c);
}

// --------------------------------------------------------------------- text

int ui_text(uint16_t* fb, int fbw, int fbh, const char* utf8, int x, int baseline,
            uint16_t col) {
  return bn_draw_text(fb, fbw, fbh, utf8, x, baseline, col);
}

int ui_width(const char* utf8) {
  static bn_run_t run[BN_MAX_RUN];
  int n = bn_shape_text(utf8, run, BN_MAX_RUN);
  return bn_run_width(run, n);
}

// Scratch for half-scale text, at SOURCE scale. 320 source px is 160 output px,
// wider than any label on this screen; anything longer is simply cut.
static const int kGrayW = 320;
static const int kGrayH = 40;
static const int kGrayBase = 28;   // baseline row inside the scratch
static uint8_t g_gray[kGrayW * kGrayH];

int ui_text_half(uint16_t* fb, int fbw, int fbh, const char* utf8, int x, int baseline,
                 uint16_t col) {
  static bn_run_t run[BN_MAX_RUN];
  int n = bn_shape_text(utf8, run, BN_MAX_RUN);
  int w = bn_run_width(run, n);

  int sw = w + 4;
  if (sw > kGrayW) sw = kGrayW;
  if (sw < 2) return 0;

  // bn_draw_run_gray() paints coverage onto white, exactly as tools/mirror.py
  // does, so 255 here means "no ink".
  memset(g_gray, 255, (size_t)sw * kGrayH);
  bn_draw_run_gray(g_gray, sw, kGrayH, run, n, 2, kGrayBase);

  const int oy0 = baseline - kGrayBase / 2;
  for (int oy = 0; oy < kGrayH / 2; oy++) {
    const int yy = oy0 + oy;
    if (yy < 0 || yy >= fbh) continue;
    const uint8_t* r0 = g_gray + (size_t)(oy * 2) * sw;
    const uint8_t* r1 = r0 + sw;
    uint16_t* dst = fb + (uint32_t)yy * fbw;
    for (int ox = 0; ox < sw / 2; ox++) {
      const int sx = ox * 2;
      // average the 2x2 block, then invert: white background -> zero coverage
      uint32_t v = (uint32_t)r0[sx] + r0[sx + 1] + r1[sx] + r1[sx + 1];
      uint32_t a = 255 - (v >> 2);
      if (!a) continue;
      const int xx = x + ox - 1;
      if (xx < 0 || xx >= fbw) continue;
      dst[xx] = blend(dst[xx], col, a);
    }
  }
  return (w + 1) / 2;
}

int ui_width_half(const char* utf8) {
  return (ui_width(utf8) + 1) / 2;
}

int ui_text_clip(uint16_t* fb, int fbw, int fbh, const char* utf8, int x, int baseline,
                 int maxw, uint16_t col) {
  static bn_run_t run[BN_MAX_RUN];
  int n = bn_shape_text(utf8, run, BN_MAX_RUN);
  int w = bn_run_width(run, n);
  if (w <= maxw) {
    bn_draw_run(fb, fbw, fbh, run, n, x, baseline, col);
    return w;
  }
  // Drop glyphs from the end until ".." fits too. Cutting a shaped run is safe
  // here because each entry already carries its own advance and offset.
  static bn_run_t dots[BN_MAX_RUN];
  int nd = bn_shape_text("..", dots, BN_MAX_RUN);
  int dw = bn_run_width(dots, nd);
  while (n > 0 && w + dw > maxw) {
    n--;
    w = bn_run_width(run, n);
  }
  bn_draw_run(fb, fbw, fbh, run, n, x, baseline, col);
  bn_draw_run(fb, fbw, fbh, dots, nd, x + w, baseline, col);
  return w + dw;
}

void ui_icon_search(uint16_t* fb, int fbw, int fbh, int cx, int cy, int r, uint16_t c) {
  // Ring plus a diagonal handle. The ring sits up and left of centre so the
  // handle has room inside the same bounding box the other icons use; drawn as
  // a radius band for the same reason ui_icon_capture() is -- a 1px Bresenham
  // circle is the first thing this panel loses.
  const int ox = cx - r / 3, oy = cy - r / 3;
  const int r2o = r * r, r2i = (r - 2) * (r - 2);
  for (int dy = -r; dy <= r; dy++) {
    for (int dx = -r; dx <= r; dx++) {
      const int d = dx * dx + dy * dy;
      if (d <= r2o && d >= r2i) px(fb, fbw, fbh, ox + dx, oy + dy, c);
    }
  }
  // handle: two pixels thick, running out along the down-right diagonal
  for (int i = 0; i <= r; i++) {
    const int hx = ox + (r * 7) / 10 + i, hy = oy + (r * 7) / 10 + i;
    px(fb, fbw, fbh, hx, hy, c);
    px(fb, fbw, fbh, hx + 1, hy, c);
    px(fb, fbw, fbh, hx, hy + 1, c);
  }
}

void ui_icon_bksp(uint16_t* fb, int fbw, int fbh, int cx, int cy, int r, uint16_t c) {
  // An arrowhead pointing left with a bar behind it -- the usual backspace, at
  // a size that reads on a 29 px key.
  for (int i = 0; i < r; i++) {
    for (int t = -i; t <= i; t++) px(fb, fbw, fbh, cx - r + i, cy + t, c);
  }
  for (int x = cx; x < cx + r; x++) {
    for (int t = -r / 2; t <= r / 2; t++) px(fb, fbw, fbh, x, cy + t, c);
  }
}
