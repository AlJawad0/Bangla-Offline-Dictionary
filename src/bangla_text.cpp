#include "bangla_text.h"

#include <string.h>

// ---------------------------------------------------------------- utilities

static bool in_set(const uint16_t* set, int n, uint32_t c) {
  for (int i = 0; i < n; i++) {
    if (set[i] == c) return true;
  }
  return false;
}

static inline bool is_cons(uint32_t c) {
  return (c >= 0x0995 && c <= 0x09B9) || c == 0x09DC || c == 0x09DD || c == 0x09DF;
}
static inline bool is_indep(uint32_t c) { return c >= 0x0985 && c <= 0x0994; }
static inline bool is_vsign(uint32_t c) { return in_set(bn_vsign, BN_VSIGN_N, c); }
static inline bool is_mod(uint32_t c) { return in_set(bn_mod, BN_MOD_N, c); }
static inline bool is_post(uint32_t c) { return in_set(bn_post_set, BN_POST_SET_N, c); }
static inline bool is_below(uint32_t c) { return in_set(bn_below_set, BN_BELOW_SET_N, c); }

static bool pre_entry(uint32_t cp, bn_run_t* out) {
  for (int i = 0; i < BN_PRES; i++) {
    if (bn_pre[i].cp == cp) {
      out->gid = bn_pre[i].gid;
      out->adv = bn_pre[i].adv;
      out->xo = 0;
      out->yo = 0;
      return true;
    }
  }
  return false;
}

// Default (unshaped) glyph for one codepoint.
static bool default_entry(uint32_t cp, bn_run_t* out) {
  if (cp >= 0x20 && cp < 0x7F) {
    *out = bn_ascii[cp - 0x20];
  } else if (cp >= BN_CP_LO && cp <= BN_CP_HI) {
    *out = bn_bengali[cp - BN_CP_LO];
  } else {
    return false;
  }
  return out->gid != 0xFFFF;
}

static int utf8_decode(const char* s, uint32_t* cps, int maxcp) {
  int n = 0;
  const uint8_t* p = (const uint8_t*)s;
  while (*p && n < maxcp) {
    uint32_t c = *p++;
    if (c < 0x80) {
      // already complete
    } else if ((c & 0xE0) == 0xC0) {
      if ((*p & 0xC0) != 0x80) continue;
      c = ((c & 0x1F) << 6) | (*p++ & 0x3F);
    } else if ((c & 0xF0) == 0xE0) {
      if ((p[0] & 0xC0) != 0x80 || (p[1] & 0xC0) != 0x80) continue;
      c = ((c & 0x0F) << 12) | ((p[0] & 0x3F) << 6) | (p[1] & 0x3F);
      p += 2;
    } else if ((c & 0xF8) == 0xF0) {
      if ((p[0] & 0xC0) != 0x80 || (p[1] & 0xC0) != 0x80 || (p[2] & 0xC0) != 0x80) continue;
      c = ((c & 0x07) << 18) | ((p[0] & 0x3F) << 12) | ((p[1] & 0x3F) << 6) | (p[2] & 0x3F);
      p += 3;
    } else {
      continue;  // stray continuation byte
    }
    cps[n++] = c;
  }
  return n;
}

// ------------------------------------------------------------ table lookups

// Keys are compared the way Python compares bytes: memcmp over the shared
// prefix, then shorter sorts first. The generator sorted bn_cluster the same
// way, so this binary search is valid.
static int key_cmp(const uint8_t* a, int la, const uint8_t* b, int lb) {
  int m = la < lb ? la : lb;
  int c = memcmp(a, b, m);
  if (c) return c;
  return la - lb;
}

static const bn_cluster_t* cluster_find(const uint8_t* key, int klen) {
  int lo = 0, hi = BN_CLUSTERS - 1;
  while (lo <= hi) {
    int mid = (lo + hi) >> 1;
    const bn_cluster_t* e = &bn_cluster[mid];
    int c = key_cmp(&bn_keys[e->koff], e->klen, key, klen);
    if (c == 0) return e;
    if (c < 0) lo = mid + 1; else hi = mid - 1;
  }
  return NULL;
}

// Only 32 post sequences; a linear scan is cheaper than keeping them sorted.
static const bn_cluster_t* post_find(const uint32_t* cps, int n) {
  for (int i = 0; i < BN_POSTS; i++) {
    const bn_cluster_t* e = &bn_post[i];
    if (e->klen != n) continue;
    bool same = true;
    for (int j = 0; j < n; j++) {
      if (bn_post_keys[e->koff + j] != (uint8_t)(cps[j] - BN_CP_LO)) { same = false; break; }
    }
    if (same) return e;
  }
  return NULL;
}

static bool anchor_find(uint16_t base, uint16_t mark, int8_t* xo, int8_t* yo) {
  uint32_t k = ((uint32_t)base << 16) | mark;
  int lo = 0, hi = BN_ANCHORS - 1;
  while (lo <= hi) {
    int mid = (lo + hi) >> 1;
    uint32_t v = bn_anchor[mid].key;
    if (v == k) { *xo = bn_anchor[mid].xo; *yo = bn_anchor[mid].yo; return true; }
    if (v < k) lo = mid + 1; else hi = mid - 1;
  }
  return false;
}

// ------------------------------------------------------------------ shaping

static inline int8_t clamp8(int v) {
  return (int8_t)(v < -128 ? -128 : (v > 127 ? 127 : v));
}

// Mirror of Model.render() in gen_bangla_font.py.
static int render_cluster(const uint32_t* cps, int n, bn_run_t* out, int maxout) {
  uint8_t key[24];
  bool allbn = (n <= (int)sizeof(key));
  for (int i = 0; i < n && allbn; i++) {
    if (cps[i] < BN_CP_LO || cps[i] > BN_CP_HI) allbn = false;
  }

  // R0: an exact whole-cluster entry wins, and must be tried BEFORE the reph
  // strip or a key that itself begins RA+HASANT could never be reached.
  if (allbn) {
    for (int i = 0; i < n; i++) key[i] = (uint8_t)(cps[i] - BN_CP_LO);
    const bn_cluster_t* e = cluster_find(key, n);
    if (e) {
      int k = e->rlen < maxout ? e->rlen : maxout;
      for (int i = 0; i < k; i++) out[i] = bn_run[e->roff + i];
      return k;
    }
  }

  // R1: leading reph
  bool reph = false;
  const uint32_t* c = cps;
  int m = n;
  if (m >= 3 && c[0] == BN_RA && c[1] == BN_HASANT) { reph = true; c += 2; m -= 2; }

  // The table lookup must not swallow a SPACING post codepoint: reph is
  // inserted relative to the table run, so an absorbed aa-kar would put the
  // reph on the wrong side of it. Zero-advance marks may be absorbed -- they
  // have to be, because bases like ga+u-kar fuse into a single glyph.
  int maxL = m;
  for (int i = 0; i < m; i++) {
    bn_run_t d;
    if (is_post(c[i]) && default_entry(c[i], &d) && d.adv != 0) { maxL = i; break; }
  }

  bn_run_t core[BN_MAX_RUN];
  int ncore = 0, consumed = 0;
  for (int L = maxL; L >= 1; L--) {
    if (L > (int)sizeof(key)) continue;
    bool ok = true;
    for (int i = 0; i < L; i++) {
      if (c[i] < BN_CP_LO || c[i] > BN_CP_HI) { ok = false; break; }
      key[i] = (uint8_t)(c[i] - BN_CP_LO);
    }
    if (!ok) continue;
    const bn_cluster_t* e = cluster_find(key, L);
    if (e) {
      for (int i = 0; i < e->rlen && ncore < BN_MAX_RUN; i++) core[ncore++] = bn_run[e->roff + i];
      consumed = L;
      break;
    }
  }

  // R2/R3: split whatever is left. Consonants no entry covered stay in the core.
  bn_run_t pre[8];
  int npre = 0;
  uint32_t post_cps[16];
  int npost = 0;
  for (int i = consumed; i < m; i++) {
    uint32_t cp = c[i];
    bn_run_t e;
    if (pre_entry(cp, &e)) {
      if (npre < 8) pre[npre++] = e;
    } else if (is_post(cp)) {
      if (npost < 16) post_cps[npost++] = cp;
    } else if (default_entry(cp, &e)) {
      if (ncore < BN_MAX_RUN) core[ncore++] = e;
    }
  }

  bn_run_t post[16];
  int npg = 0;
  const bn_cluster_t* pe = post_find(post_cps, npost);
  if (pe) {
    for (int i = 0; i < pe->rlen && npg < 16; i++) post[npg++] = bn_post_run[pe->roff + i];
  } else {
    for (int i = 0; i < npost; i++) {
      bn_run_t e;
      if (default_entry(post_cps[i], &e) && npg < 16) post[npg++] = e;
    }
  }

  // Assemble. Reph is an above-base mark: after the base and any BELOW-base
  // marks, but before anything drawn above (candrabindu) or beside (aa-kar).
  bn_run_t items[BN_MAX_RUN];
  bool needs[BN_MAX_RUN];
  int ni = 0;
  for (int i = 0; i < ncore && ni < BN_MAX_RUN; i++) { items[ni] = core[i]; needs[ni] = false; ni++; }
  for (int i = 0; i < npg && ni < BN_MAX_RUN; i++) { items[ni] = post[i]; needs[ni] = true; ni++; }
  if (reph) {
    int nbelow = 0;
    for (int i = 0; i < npost; i++) {
      if (is_below(post_cps[i])) nbelow++; else break;
    }
    if (nbelow > npg) nbelow = npg;
    int at = ncore + nbelow;
    if (at > ni) at = ni;
    if (ni < BN_MAX_RUN) {
      for (int i = ni; i > at; i--) { items[i] = items[i - 1]; needs[i] = needs[i - 1]; }
      items[at] = bn_reph;
      needs[at] = false;
      ni++;
    }
  }

  // Place: walk left to right, resolving mark attachment. A mark hangs off
  // whatever glyph precedes it, base or mark, and the stored anchor is
  // relative to that glyph's ORIGIN (HarfBuzz reports pen-relative offsets,
  // which shift when something sits in between).
  int no = 0, pen = 0, prev_x = 0, prev_y = 0;
  int prev_g = -1;
  for (int i = 0; i < npre + ni && no < maxout; i++) {
    bn_run_t r = (i < npre) ? pre[i] : items[i - npre];
    bool need = (i < npre) ? false : needs[i - npre];
    if (need && r.adv == 0 && prev_g >= 0) {
      int8_t ax, ay;
      if (anchor_find((uint16_t)prev_g, r.gid, &ax, &ay)) {
        r.xo = clamp8(prev_x + ax - pen);
        r.yo = clamp8(prev_y + ay);
      }
    }
    out[no++] = r;
    prev_g = r.gid;
    prev_x = pen + r.xo;
    prev_y = r.yo;
    pen += r.adv;
  }
  return no;
}

// Mirror of segment() in gen_bangla_font.py, with the canonical decomposition
// of o-kar / au-kar applied inline.
int bn_shape_text(const char* utf8, bn_run_t* out, int maxout) {
  uint32_t cps[512];
  int n = utf8_decode(utf8, cps, 512);
  int no = 0;
  int i = 0;
  while (i < n && no < maxout) {
    uint32_t cp = cps[i];
    if (cp < BN_CP_LO || cp > BN_CP_HI) {
      bn_run_t e;
      if (default_entry(cp, &e)) out[no++] = e;
      i++;
      continue;
    }
    int start = i;
    if (is_cons(cp) || is_indep(cp)) {
      i++;
      if (i < n && cps[i] == 0x09BC) i++;
      while (i + 1 < n && cps[i] == BN_HASANT && is_cons(cps[i + 1])) {
        i += 2;
        if (i < n && cps[i] == 0x09BC) i++;
      }
      if (i < n && cps[i] == BN_HASANT && (i + 1 >= n || !is_cons(cps[i + 1]))) i++;
      if (i < n && is_vsign(cps[i])) i++;
      while (i < n && is_mod(cps[i])) i++;
    } else {
      i++;
    }

    uint32_t cl[32];
    int m = 0;
    for (int k = start; k < i && m < 30; k++) {
      uint32_t c = cps[k];
      bool split = false;
      for (int d = 0; d < BN_DECOMP_N; d++) {
        if (bn_decomp[d].cp == c) {
          cl[m++] = bn_decomp[d].a;
          cl[m++] = bn_decomp[d].b;
          split = true;
          break;
        }
      }
      if (!split) cl[m++] = c;
    }
    no += render_cluster(cl, m, out + no, maxout - no);
  }
  return no;
}

int bn_run_width(const bn_run_t* run, int n) {
  int w = 0;
  for (int i = 0; i < n; i++) w += run[i].adv;
  return w;
}

// ---------------------------------------------------------------- rendering

static inline uint16_t blend565(uint16_t dst, uint16_t src, uint8_t a15) {
  uint32_t a = (uint32_t)a15 * 17;   // 0..15 coverage -> 0..255
  uint32_t ia = 255 - a;
  uint32_t dr = (dst >> 11) & 0x1F, dg = (dst >> 5) & 0x3F, db = dst & 0x1F;
  uint32_t sr = (src >> 11) & 0x1F, sg = (src >> 5) & 0x3F, sb = src & 0x1F;
  uint32_t r = (sr * a + dr * ia) / 255;
  uint32_t g = (sg * a + dg * ia) / 255;
  uint32_t b = (sb * a + db * ia) / 255;
  return (uint16_t)((r << 11) | (g << 5) | b);
}

void bn_draw_run(uint16_t* fb, int fbw, int fbh, const bn_run_t* run, int n,
                 int x, int baseline, uint16_t color) {
  int pen = x;
  for (int i = 0; i < n; i++) {
    if (run[i].gid >= BN_GLYPHS) { pen += run[i].adv; continue; }
    const bn_glyph_t* g = &bn_glyphs[run[i].gid];
    int gx = pen + run[i].xo + g->bx;
    int gy = baseline - run[i].yo - g->by;
    int stride = (g->w + 1) >> 1;
    for (int ry = 0; ry < g->h; ry++) {
      int yy = gy + ry;
      if (yy < 0 || yy >= fbh) continue;
      const uint8_t* row = &bn_bits[g->off + (uint32_t)ry * stride];
      uint16_t* dst = fb + (uint32_t)yy * fbw;
      for (int rx = 0; rx < g->w; rx++) {
        uint8_t a = (rx & 1) ? (row[rx >> 1] & 0x0F) : (row[rx >> 1] >> 4);
        if (!a) continue;
        int xx = gx + rx;
        if (xx < 0 || xx >= fbw) continue;
        dst[xx] = blend565(dst[xx], color, a);
      }
    }
    pen += run[i].adv;
  }
}

void bn_draw_run_gray(uint8_t* buf, int w, int h, const bn_run_t* run, int n,
                      int x, int baseline) {
  int pen = x;
  for (int i = 0; i < n; i++) {
    if (run[i].gid >= BN_GLYPHS) { pen += run[i].adv; continue; }
    const bn_glyph_t* g = &bn_glyphs[run[i].gid];
    int gx = pen + run[i].xo + g->bx;
    int gy = baseline - run[i].yo - g->by;
    int stride = (g->w + 1) >> 1;
    for (int ry = 0; ry < g->h; ry++) {
      int yy = gy + ry;
      if (yy < 0 || yy >= h) continue;
      const uint8_t* row = &bn_bits[g->off + (uint32_t)ry * stride];
      uint8_t* dst = buf + (uint32_t)yy * w;
      for (int rx = 0; rx < g->w; rx++) {
        uint8_t a = (rx & 1) ? (row[rx >> 1] & 0x0F) : (row[rx >> 1] >> 4);
        if (!a) continue;
        int xx = gx + rx;
        if (xx < 0 || xx >= w) continue;
        uint8_t v = (uint8_t)(255 - a * 17);   // same as mirror.py
        if (v < dst[xx]) dst[xx] = v;
      }
    }
    pen += run[i].adv;
  }
}

int bn_draw_text(uint16_t* fb, int fbw, int fbh, const char* utf8,
                 int x, int baseline, uint16_t color) {
  static bn_run_t run[BN_MAX_RUN];
  int n = bn_shape_text(utf8, run, BN_MAX_RUN);
  bn_draw_run(fb, fbw, fbh, run, n, x, baseline, color);
  return bn_run_width(run, n);
}
