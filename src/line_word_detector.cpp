#include "line_word_detector.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "esp_heap_caps.h"

// Port of pc_pipeline/line_detector/line_detector.py. Stage letters and constant names
// match the reference; where a comment says "as the reference", the Python is the spec.
//
// Arithmetic: per-pixel work is integer or single-precision float (the S3 has an FPU);
// the per-line least-squares fits use double, a few hundred operations per line.

namespace ocr_demo {
namespace {

// ---- constants (line_detector.py) ---------------------------------------------------
constexpr int kDec = 4;
constexpr int kSeR = 4;
constexpr float kInk = 0.5f;
constexpr float kRangeMin = 22.0f;
constexpr float kInkFloor = 10.0f;
constexpr uint32_t kMinArea = 3;
constexpr uint32_t kGlyphMinArea = 12;
constexpr int kGlyphMinH = 5;
constexpr int kPolR = 8;
constexpr int kPolRMax = 24;
constexpr float kPolLines = 1.6f;
constexpr int kPolOverride = 2;
constexpr int kAngleRange = 40;
constexpr float kLinkGap = 2.2f;
constexpr float kLinkDv = 0.38f;
constexpr float kLinkRatio = 2.2f;
constexpr float kTrackSlope = 0.30f;
constexpr double kFitSlope = 0.30;
constexpr float kXbFrac = 0.40f;
constexpr float kTauDefault = 0.40f;
constexpr float kTauMin = 0.28f, kTauMax = 0.75f;
constexpr float kLetterH = 0.55f;
constexpr float kPadU = 0.30f;
constexpr float kPadV = 0.25f;
constexpr float kTruncU = 0.40f;

constexpr int kMaxComps = 8192;
constexpr int kMaxProv = 65534;
constexpr int kMaxLines = 400;
constexpr int kMaxAnglePoints = 4000;   // angle search samples (internal RAM)
constexpr int kHistBins = 1400;
constexpr int kProfBins = 700;
constexpr uint16_t kNone = 0xFFFF;

enum : uint8_t { kFGlyph = 1, kFBackground = 2, kFCounter = 4 };

struct LComp {
  uint16_t x1, y1, x2, y2;
  uint32_t area;
  float u1, u2, v1, v2;
  int16_t line, line2, word;
  uint8_t pol;
  uint8_t f;
  float h() const { return v2 - v1 + 1; }
  float w() const { return u2 - u1 + 1; }
  float uc() const { return 0.5f * (u1 + u2); }
  float vc() const { return 0.5f * (v1 + v2); }
  int bh() const { return y2 - y1 + 1; }
  int bw() const { return x2 - x1 + 1; }
};

struct LLine {
  uint8_t pol;
  uint8_t caps;
  int16_t ring[9];      // the last 9 members, in append order
  int count;
  float u1, u2;
  float hmed, pred, pred_u, pslope;
  double a, b, c;       // centre line v = a + b u + c u^2
  float base_off;
  float xh;
  double centre(double u) const { return a + b * u + c * u * u; }
  double base(double u) const { return centre(u) + base_off; }
  double slope(double u) const { return b + 2 * c * u; }
  int member(int k) const {   // k counts back from the newest member, 0 = newest
    return ring[(count - 1 - k) % 9];
  }
};

struct LWord {
  int16_t line;
  int16_t pool_at, pool_n;
  uint8_t pol;
  float xh, u1, u2, uc, top_rel, bot_rel;
  float bx, by, ang;
  float quad[8];
  int ix1, iy1, ix2, iy2;
  int letters;
  uint32_t area;
  bool trunc_l, trunc_r, trunc_t, trunc_b;
  bool uncertain;
  float key_line, key_u;
};

template <typename T>
T* Alloc(size_t n) {
  void* p = heap_caps_malloc(n * sizeof(T), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (p == nullptr) p = heap_caps_malloc(n * sizeof(T), MALLOC_CAP_8BIT);
  return static_cast<T*>(p);
}

// np.median semantics: the mean of the two middle values for an even count.
float MedianF(float* v, int n) {
  if (n <= 0) return 0.0f;
  std::sort(v, v + n);
  return (n & 1) ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

inline int Clamp(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Python round() is round-half-to-even; lrintf in the default rounding mode is too.
inline int RoundEven(float v) { return static_cast<int>(lrintf(v)); }

// ---- stage A helpers -------------------------------------------------------------
// Separable square min/max filter with clamped ("nearest") borders.
void MinMaxFilter(uint8_t* img, uint8_t* tmp, int w, int h, int r, bool is_max) {
  for (int y = 0; y < h; ++y) {
    const uint8_t* row = img + y * w;
    for (int x = 0; x < w; ++x) {
      uint8_t v = row[x];
      for (int k = -r; k <= r; ++k) {
        const uint8_t s = row[Clamp(x + k, 0, w - 1)];
        v = is_max ? std::max(v, s) : std::min(v, s);
      }
      tmp[y * w + x] = v;
    }
  }
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      uint8_t v = tmp[y * w + x];
      for (int k = -r; k <= r; ++k) {
        const uint8_t s = tmp[Clamp(y + k, 0, h - 1) * w + x];
        v = is_max ? std::max(v, s) : std::min(v, s);
      }
      img[y * w + x] = v;
    }
  }
}

// Binary erosion with outside = true, square radius r.
void Erode(uint8_t* m, uint8_t* tmp, int w, int h, int r) {
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      uint8_t v = 1;
      for (int k = -r; k <= r && v; ++k) {
        const int xx = x + k;
        if (xx >= 0 && xx < w && !m[y * w + xx]) v = 0;
      }
      tmp[y * w + x] = v;
    }
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x) {
      uint8_t v = 1;
      for (int k = -r; k <= r && v; ++k) {
        const int yy = y + k;
        if (yy >= 0 && yy < h && !tmp[yy * w + x]) v = 0;
      }
      m[y * w + x] = v;
    }
}

// Box sum with replicated borders (scipy uniform_filter mode="nearest", times k^2),
// as running sums: O(1) per sample whatever the radius.
void BoxSum(const uint8_t* m, uint16_t* out, uint16_t* tmp, int w, int h, int r) {
  for (int y = 0; y < h; ++y) {
    const uint8_t* row = m + y * w;
    int s = 0;
    for (int k = -r; k <= r; ++k) s += row[Clamp(k, 0, w - 1)];
    for (int x = 0; x < w; ++x) {
      tmp[y * w + x] = static_cast<uint16_t>(s);
      s += row[Clamp(x + r + 1, 0, w - 1)] - row[Clamp(x - r, 0, w - 1)];
    }
  }
  for (int x = 0; x < w; ++x) {
    uint32_t s = 0;
    for (int k = -r; k <= r; ++k) s += tmp[Clamp(k, 0, h - 1) * w + x];
    for (int y = 0; y < h; ++y) {
      out[y * w + x] = static_cast<uint16_t>(std::min<uint32_t>(s, 65535));
      s += tmp[Clamp(y + r + 1, 0, h - 1) * w + x];
      s -= tmp[Clamp(y - r, 0, h - 1) * w + x];
    }
  }
}

// Polarity map (polarity_map() in the reference): only wide areas vote.
void PolarityMap(const uint8_t* ts, const uint8_t* bs, int sw, int sh, int radius, int erode,
                 uint8_t* pol, uint8_t* s8a, uint8_t* s8b, uint8_t* s8c, uint16_t* s16a,
                 uint16_t* s16b, uint16_t* s16c) {
  const int n = sw * sh;
  const int full = 3 * kDec * kDec / 4;
  for (int i = 0; i < n; ++i) {
    s8a[i] = ts[i] >= full;
    s8b[i] = bs[i] >= full;
  }
  if (erode > 0) {
    Erode(s8a, s8c, sw, sh, erode);
    Erode(s8b, s8c, sw, sh, erode);
  }
  uint32_t tsum = 0, bsum = 0;
  for (int i = 0; i < n; ++i) {
    tsum += s8a[i];
    bsum += s8b[i];
  }
  const uint8_t glob = bsum > tsum ? 2 : 1;
  BoxSum(s8a, s16a, s16c, sw, sh, radius);
  BoxSum(s8b, s16b, s16c, sw, sh, radius);
  for (int i = 0; i < n; ++i) {
    pol[i] = glob;
    if (glob == 1 && s16b[i] > kPolOverride * s16a[i] + 2) pol[i] = 2;
    if (glob == 2 && s16a[i] > kPolOverride * s16b[i] + 2) pol[i] = 1;
  }
}

// Bilinear expansion coordinates: small sample j sits at full-res j*4 + 1.5.
inline void ExpandCoord(int p, int small_extent, int* i0, int* i1, float* wgt) {
  float f = (p - 0.5f * (kDec - 1)) / kDec;
  if (f < 0) f = 0;
  if (f > small_extent - 1) f = static_cast<float>(small_extent - 1);
  *i0 = static_cast<int>(f);
  *i1 = std::min(*i0 + 1, small_extent - 1);
  *wgt = f - *i0;
}

void GlyphTest(LComp* comps, int n, int W, int H, const uint8_t* pol, int sw, int sh) {
  for (int i = 0; i < n; ++i) {
    LComp& c = comps[i];
    const int w = c.bw(), h = c.bh();
    const bool big = h > H / 2 || w > (9 * W) / 10 || c.area > static_cast<uint32_t>((W * H) / 8);
    const bool thin = w >= 12 * h || h >= 12 * w;
    const bool sparse = static_cast<uint64_t>(c.area) * 16 < static_cast<uint64_t>(w) * h &&
                        std::max(w, h) > 40;
    c.f = 0;
    if (!(big || thin || sparse)) c.f |= kFGlyph;
    if (big) c.f |= kFBackground;
    const int cy = std::min(((c.y1 + c.y2) / 2) / kDec, sh - 1);
    const int cx = std::min(((c.x1 + c.x2) / 2) / kDec, sw - 1);
    if (pol[cy * sw + cx] != c.pol) {
      c.f &= ~kFGlyph;
      c.f |= kFCounter;
    }
  }
}

inline bool IsGlyph(const LComp& c) {
  return (c.f & kFGlyph) && c.area >= kGlyphMinArea && c.bh() >= kGlyphMinH;
}

// Solve the normal equations of an n-parameter polynomial fit (n = 2 or 3), centred.
bool PolyFit(const float* us, const float* vs, const uint8_t* keep, int n, int terms,
             double* a, double* b, double* c) {
  double um = 0;
  int m = 0;
  for (int i = 0; i < n; ++i)
    if (keep[i]) {
      um += us[i];
      ++m;
    }
  if (m < terms) return false;
  um /= m;
  double S[5] = {0, 0, 0, 0, 0}, T[3] = {0, 0, 0};
  for (int i = 0; i < n; ++i) {
    if (!keep[i]) continue;
    const double u = us[i] - um, v = vs[i];
    double p = 1;
    for (int k = 0; k < 5; ++k) {
      S[k] += p;
      if (k < 3) T[k] += p * v;
      p *= u;
    }
  }
  double ap, bp, cp = 0;
  if (terms == 2) {
    const double det = S[0] * S[2] - S[1] * S[1];
    if (fabs(det) < 1e-12) return false;
    ap = (T[0] * S[2] - S[1] * T[1]) / det;
    bp = (S[0] * T[1] - S[1] * T[0]) / det;
  } else {
    double M[3][4] = {{S[0], S[1], S[2], T[0]}, {S[1], S[2], S[3], T[1]}, {S[2], S[3], S[4], T[2]}};
    for (int col = 0; col < 3; ++col) {
      int piv = col;
      for (int r = col + 1; r < 3; ++r)
        if (fabs(M[r][col]) > fabs(M[piv][col])) piv = r;
      if (fabs(M[piv][col]) < 1e-12) return false;
      for (int k = 0; k < 4; ++k) std::swap(M[col][k], M[piv][k]);
      for (int r = 0; r < 3; ++r) {
        if (r == col) continue;
        const double f = M[r][col] / M[col][col];
        for (int k = col; k < 4; ++k) M[r][k] -= f * M[col][k];
      }
    }
    ap = M[0][3] / M[0][0];
    bp = M[1][3] / M[1][1];
    cp = M[2][3] / M[2][2];
  }
  *a = ap - bp * um + cp * um * um;
  *b = bp - 2 * cp * um;
  *c = cp;
  return true;
}

struct Scratch {
  uint8_t* cls = nullptr;
  uint16_t* parent = nullptr;
  uint16_t* cmap = nullptr;
  LComp* comps = nullptr;
  int16_t* ax = nullptr;
  int16_t* ay = nullptr;
  LLine* lines = nullptr;
  int16_t* order = nullptr;
  int16_t* csr = nullptr;
  int* off = nullptr;
  float* f1 = nullptr;
  float* f2 = nullptr;
  float* f3 = nullptr;
  uint8_t* keep = nullptr;
  LWord* words = nullptr;
  int16_t* pool = nullptr;
  int16_t* grp = nullptr;
  uint8_t* small8 = nullptr;     // 6 decimated uint8 planes
  uint16_t* small16 = nullptr;   // 3 decimated uint16 planes
  int32_t* hist = nullptr;
  float* tmp = nullptr;          // kMaxComps
  float* cl_u1 = nullptr;        // kMaxComps each
  float* cl_u2 = nullptr;
  float* sp_u1 = nullptr;
  float* sp_u2 = nullptr;
  int16_t* cl_of = nullptr;      // 2*kMaxComps
  int16_t* cl_word = nullptr;    // kMaxComps
  uint8_t* w_unc = nullptr;      // kMaxComps
  int* cursor = nullptr;         // kMaxLines+1
  void Release() {
    heap_caps_free(tmp); heap_caps_free(cl_u1); heap_caps_free(cl_u2); heap_caps_free(sp_u1);
    heap_caps_free(sp_u2); heap_caps_free(cl_of); heap_caps_free(cl_word); heap_caps_free(w_unc);
    heap_caps_free(cursor);
    heap_caps_free(cls); heap_caps_free(parent); heap_caps_free(cmap); heap_caps_free(comps);
    heap_caps_free(ax); heap_caps_free(ay); heap_caps_free(lines); heap_caps_free(order);
    heap_caps_free(csr); heap_caps_free(off); heap_caps_free(f1); heap_caps_free(f2);
    heap_caps_free(f3); heap_caps_free(keep); heap_caps_free(words); heap_caps_free(pool);
    heap_caps_free(grp); heap_caps_free(small8); heap_caps_free(small16); heap_caps_free(hist);
  }
  bool Ok() const {
    return cls && parent && cmap && comps && ax && ay && lines && order && csr && off && f1 &&
           f2 && f3 && keep && words && pool && grp && small8 && small16 && hist && tmp &&
           cl_u1 && cl_u2 && sp_u1 && sp_u2 && cl_of && cl_word && w_unc && cursor;
  }
};

uint16_t Find(uint16_t* parent, uint16_t x) {
  while (parent[x] != x) {
    parent[x] = parent[parent[x]];
    x = parent[x];
  }
  return x;
}

// Build CSR membership: for each line, the comps whose line (or line2) is it.
void BuildCsr(const LComp* comps, int nc, int nl, bool with_second, int* off, int16_t* csr,
              int* cursor) {
  for (int i = 0; i <= nl; ++i) off[i] = 0;
  for (int i = 0; i < nc; ++i) {
    if (comps[i].line >= 0) ++off[comps[i].line + 1];
    if (with_second && comps[i].line2 >= 0) ++off[comps[i].line2 + 1];
  }
  for (int i = 0; i < nl; ++i) off[i + 1] += off[i];
  for (int i = 0; i < nl; ++i) cursor[i] = off[i];
  for (int i = 0; i < nc; ++i) {
    if (comps[i].line >= 0) csr[cursor[comps[i].line]++] = static_cast<int16_t>(i);
    if (with_second && comps[i].line2 >= 0) csr[cursor[comps[i].line2]++] = static_cast<int16_t>(i);
  }
}

// Stage F: robust centre line, then the row-density profile of the members' pixels.
void FitLine(LLine& L, const int16_t* mem, int n, const LComp* comps, const uint16_t* labels,
             int W, float ct, float st, float* us, float* vs, float* hs, uint8_t* keep,
             int32_t* hist, float* tmp) {
  L.u1 = 1e9f;
  L.u2 = -1e9f;
  for (int k = 0; k < n; ++k) {
    const LComp& m = comps[mem[k]];
    us[k] = m.uc();
    vs[k] = m.vc();
    hs[k] = m.h();
    L.u1 = std::min(L.u1, m.u1);
    L.u2 = std::max(L.u2, m.u2);
  }
  // median of heights / centres on copies (MedianF sorts in place)
  for (int k = 0; k < n; ++k) tmp[k] = hs[k];
  const float hmed = MedianF(tmp, n);
  for (int k = 0; k < n; ++k) tmp[k] = vs[k];
  L.a = MedianF(tmp, n);
  L.b = 0;
  L.c = 0;
  for (int k = 0; k < n; ++k) keep[k] = 1;
  for (int it = 0; it < 2; ++it) {
    int nk = 0;
    float kmin = 1e9f, kmax = -1e9f;
    for (int k = 0; k < n; ++k)
      if (keep[k]) {
        ++nk;
        kmin = std::min(kmin, us[k]);
        kmax = std::max(kmax, us[k]);
      }
    if (nk >= 3 && (kmax - kmin) > 2 * hmed) {
      const bool quad = nk >= 8 && (kmax - kmin) > 14 * hmed;
      double a, b, c;
      if (PolyFit(us, vs, keep, n, quad ? 3 : 2, &a, &b, &c)) {
        L.a = a;
        L.b = b;
        L.c = quad ? c : 0.0;
        if (fabs(L.b) > kFitSlope) {
          L.b = L.b > 0 ? kFitSlope : -kFitSlope;
          L.c = 0;
          for (int k = 0; k < n; ++k) tmp[k] = static_cast<float>(vs[k] - L.b * us[k]);
          L.a = MedianF(tmp, n);
        }
      }
    }
    int kept = 0;
    for (int k = 0; k < n; ++k) {
      const double res = vs[k] - L.centre(us[k]);
      keep[k] = fabs(res) <= 0.35 * std::max(hs[k], hmed);
      kept += keep[k];
    }
    if (kept < std::max(2, n / 2))
      for (int k = 0; k < n; ++k) keep[k] = 1;
  }
  // profile about the centre line -- per pixel, so single precision (double is
  // soft-float on the S3)
  const float fa = static_cast<float>(L.a), fb = static_cast<float>(L.b), fc = static_cast<float>(L.c);
  float lo = 1e30f;
  for (int pass = 0; pass < 2; ++pass) {
    if (pass == 1) {
      if (lo > 1e29f) lo = 0;
      for (int i = 0; i < kProfBins; ++i) hist[i] = 0;
    }
    const int ilo = static_cast<int>(floorf(lo));
    for (int k = 0; k < n; ++k) {
      const LComp& m = comps[mem[k]];
      const uint16_t id = static_cast<uint16_t>(mem[k] + 1);
      for (int y = m.y1; y <= m.y2; ++y) {
        const uint16_t* row = labels + y * W;
        for (int x = m.x1; x <= m.x2; ++x) {
          if (row[x] != id) continue;
          const float u = x * ct + y * st;
          const float v = -x * st + y * ct;
          const float dv = v - (fa + (fb + fc * u) * u);
          if (pass == 0) {
            if (dv < lo) lo = dv;
          } else {
            const int bin = Clamp(static_cast<int>(floorf(dv)) - ilo, 0, kProfBins - 1);
            ++hist[bin];
          }
        }
      }
    }
  }
  const int ilo = static_cast<int>(floorf(lo));
  int last = 0;
  for (int i = 0; i < kProfBins; ++i)
    if (hist[i]) last = i;
  const int len = last + 1;
  int pk = 0;
  for (int i = 1; i < len; ++i)
    if (hist[i] > hist[pk]) pk = i;
  const float thr = kXbFrac * hist[pk];
  int t = pk, b = pk;
  while (t > 0 && hist[t - 1] >= thr) --t;
  while (b + 1 < len && hist[b + 1] >= thr) ++b;
  L.xh = static_cast<float>(b - t + 1);
  L.base_off = static_cast<float>(b + 1 + ilo);
  int64_t above = 0, band = 0, below = 0;
  for (int i = std::max(0, t - RoundEven(0.5f * L.xh)); i < t; ++i) above += hist[i];
  for (int i = t; i <= b; ++i) band += hist[i];
  for (int i = b + 1; i < std::min(len, b + 1 + RoundEven(0.4f * L.xh)); ++i) below += hist[i];
  L.caps = n >= 4 && above < 0.012 * band && below < 0.012 * band;
  if (L.caps) L.xh = L.xh / 1.4f;
  L.xh = std::max(L.xh, 3.0f);
  L.hmed = hmed;
}

float Otsu1D(float* xs, int n) {
  std::sort(xs, xs + n);
  float best_t = xs[0];
  double best_v = -1;
  double total = 0;
  for (int i = 0; i < n; ++i) total += xs[i];
  double left = 0;
  for (int i = 1; i < n; ++i) {
    left += xs[i - 1];
    const double ma = left / i, mb = (total - left) / (n - i);
    const double v = static_cast<double>(i) * (n - i) * (ma - mb) * (ma - mb);
    if (v > best_v) {
      best_v = v;
      best_t = 0.5f * (xs[i - 1] + xs[i]);
    }
  }
  return best_t;
}

}  // namespace

bool LineWordDetector::EnsureBuffers(int width, int height) {
  const size_t px = static_cast<size_t>(width) * height;
  if (px > label_capacity_) {
    heap_caps_free(labels_);
    labels_ = Alloc<uint16_t>(px);
    label_capacity_ = labels_ ? px : 0;
  }
  if (owner_ == nullptr) owner_ = Alloc<uint16_t>(kMaxComps + 1);
  const size_t small = static_cast<size_t>((width + kDec - 1) / kDec) * ((height + kDec - 1) / kDec);
  if (small > small_capacity_) {
    heap_caps_free(close_s_);
    heap_caps_free(open_s_);
    close_s_ = Alloc<uint8_t>(small);
    open_s_ = Alloc<uint8_t>(small);
    small_capacity_ = (close_s_ && open_s_) ? small : 0;
  }
  return labels_ && owner_ && close_s_ && open_s_;
}

bool LineWordDetector::Detect(const GrayImage& image, DetectionResult* result, Stream* log) {
  if (result == nullptr) return false;
  std::memset(result, 0, sizeof(*result));
  const int W = image.width, H = image.height;
  if (image.pixels == nullptr || W == 0 || H == 0 || W > kDetectorMaxWidth ||
      H > kDetectorMaxHeight) {
    snprintf(result->error, sizeof(result->error), "input: invalid grayscale image");
    return false;
  }
  const uint32_t started = micros();
  if (!EnsureBuffers(W, H)) {
    snprintf(result->error, sizeof(result->error), "alloc: persistent buffers");
    return false;
  }
  const int sw = (W + kDec - 1) / kDec, sh = (H + kDec - 1) / kDec, ns = sw * sh;
  const size_t npx = static_cast<size_t>(W) * H;

  Scratch S;
  S.cls = Alloc<uint8_t>(npx);
  S.parent = Alloc<uint16_t>(kMaxProv + 2);
  S.cmap = Alloc<uint16_t>(kMaxProv + 2);
  S.comps = Alloc<LComp>(kMaxComps);
  S.ax = Alloc<int16_t>(1);
  S.ay = Alloc<int16_t>(1);
  S.lines = Alloc<LLine>(kMaxLines);
  S.order = Alloc<int16_t>(kMaxComps + kMaxLines);
  S.csr = Alloc<int16_t>(2 * kMaxComps);
  S.off = Alloc<int>(kMaxLines + 1);
  S.f1 = Alloc<float>(2 * kMaxComps);
  S.f2 = Alloc<float>(2 * kMaxComps);
  S.f3 = Alloc<float>(2 * kMaxComps);
  S.keep = Alloc<uint8_t>(2 * kMaxComps);
  S.words = Alloc<LWord>(kMaxWordBoxes);
  S.pool = Alloc<int16_t>(2 * kMaxComps);
  S.grp = Alloc<int16_t>(2 * kMaxComps);
  S.small8 = Alloc<uint8_t>(static_cast<size_t>(ns) * 6);
  S.small16 = Alloc<uint16_t>(static_cast<size_t>(ns) * 3);
  S.hist = Alloc<int32_t>(kHistBins);
  S.tmp = Alloc<float>(kMaxComps);
  S.cl_u1 = Alloc<float>(kMaxComps);
  S.cl_u2 = Alloc<float>(kMaxComps);
  S.sp_u1 = Alloc<float>(kMaxComps);
  S.sp_u2 = Alloc<float>(kMaxComps);
  S.cl_of = Alloc<int16_t>(2 * kMaxComps);
  S.cl_word = Alloc<int16_t>(kMaxComps);
  S.w_unc = Alloc<uint8_t>(kMaxComps);
  S.cursor = Alloc<int>(kMaxLines + 1);
  if (!S.Ok()) {
    S.Release();
    snprintf(result->error, sizeof(result->error), "alloc: detector scratch");
    if (log) log->println("LDET stage=alloc status=error");
    return false;
  }
  uint8_t* bts = S.small8;             // near-top block counts
  uint8_t* bbs = S.small8 + ns;        // near-bottom block counts
  uint8_t* pol = S.small8 + 2 * ns;    // polarity map
  uint8_t* s8a = S.small8 + 3 * ns;
  uint8_t* s8b = S.small8 + 4 * ns;
  uint8_t* s8c = S.small8 + 5 * ns;
  uint16_t* s16a = S.small16;
  uint16_t* s16b = S.small16 + ns;
  uint16_t* s16c = S.small16 + 2 * ns;
  const uint8_t* g = image.pixels;

  // ---- A: two-sided local contrast -------------------------------------------------
  uint32_t phase = micros();
  for (int j = 0; j < sh; ++j)
    for (int i = 0; i < sw; ++i) {
      uint8_t mx = 0, mn = 255;
      for (int y = j * kDec; y < std::min(H, (j + 1) * kDec); ++y)
        for (int x = i * kDec; x < std::min(W, (i + 1) * kDec); ++x) {
          const uint8_t v = g[y * W + x];
          mx = std::max(mx, v);
          mn = std::min(mn, v);
        }
      close_s_[j * sw + i] = mx;
      open_s_[j * sw + i] = mn;
    }
  MinMaxFilter(close_s_, s8a, sw, sh, kSeR, true);
  MinMaxFilter(close_s_, s8a, sw, sh, kSeR, false);
  MinMaxFilter(open_s_, s8a, sw, sh, kSeR, false);
  MinMaxFilter(open_s_, s8a, sw, sh, kSeR, true);
  std::memset(bts, 0, ns);
  std::memset(bbs, 0, ns);
  {
    static int xi0[kDetectorMaxWidth], xi1[kDetectorMaxWidth];
    static float xw[kDetectorMaxWidth];
    for (int x = 0; x < W; ++x) ExpandCoord(x, sw, &xi0[x], &xi1[x], &xw[x]);
    static float crow_v[kDetectorMaxWidth / kDec + 1], orow_v[kDetectorMaxWidth / kDec + 1];
    for (int y = 0; y < H; ++y) {
      int j0, j1;
      float wy;
      ExpandCoord(y, sh, &j0, &j1, &wy);
      const uint8_t* c0 = close_s_ + j0 * sw;
      const uint8_t* c1 = close_s_ + j1 * sw;
      const uint8_t* o0 = open_s_ + j0 * sw;
      const uint8_t* o1 = open_s_ + j1 * sw;
      for (int i = 0; i < sw; ++i) {
        crow_v[i] = c0[i] * (1 - wy) + c1[i] * wy;
        orow_v[i] = o0[i] * (1 - wy) + o1[i] * wy;
      }
      const uint8_t* grow = g + y * W;
      uint8_t* crow = S.cls + y * W;
      const int bj = y / kDec;
      for (int x = 0; x < W; ++x) {
        const int i0 = xi0[x], i1 = xi1[x];
        const float wx = xw[x];
        const float C = crow_v[i0] * (1 - wx) + crow_v[i1] * wx;
        const float O = orow_v[i0] * (1 - wx) + orow_v[i1] * wx;
        const float R = C - O;
        const float gv = grow[x];
        uint8_t k = 0;
        if (R >= kRangeMin) {
          const float thr = std::max(kInkFloor, kInk * R);
          if (C - gv > thr) k = 1;
          else if (gv - O > thr) k = 2;
          const int bi = bj * sw + x / kDec;
          if (4 * (C - gv) < R) ++bts[bi];
          if (4 * (gv - O) < R) ++bbs[bi];
        }
        crow[x] = k;
      }
    }
  }
  result->background_us = micros() - phase;

  // ---- B: components, both polarities, 8-connected -----------------------------------
  phase = micros();
  uint16_t* lab = labels_;
  uint16_t* parent = S.parent;
  int nprov = 0;
  bool overflow = false;
  for (int y = 0; y < H; ++y) {
    const uint8_t* crow = S.cls + y * W;
    const uint8_t* prow = y > 0 ? S.cls + (y - 1) * W : nullptr;
    uint16_t* lrow = lab + y * W;
    const uint16_t* lprev = y > 0 ? lab + (y - 1) * W : nullptr;
    for (int x = 0; x < W; ++x) {
      const uint8_t k = crow[x];
      if (!k) {
        lrow[x] = 0;
        continue;
      }
      uint16_t nb[4];
      int nn = 0;
      if (x > 0 && crow[x - 1] == k && lrow[x - 1]) nb[nn++] = lrow[x - 1];
      if (prow) {
        if (x > 0 && prow[x - 1] == k && lprev[x - 1]) nb[nn++] = lprev[x - 1];
        if (prow[x] == k && lprev[x]) nb[nn++] = lprev[x];
        if (x + 1 < W && prow[x + 1] == k && lprev[x + 1]) nb[nn++] = lprev[x + 1];
      }
      if (nn == 0) {
        if (nprov >= kMaxProv) {
          lrow[x] = 0;
          overflow = true;
          continue;
        }
        ++nprov;
        parent[nprov] = static_cast<uint16_t>(nprov);
        lrow[x] = static_cast<uint16_t>(nprov);
        continue;
      }
      uint16_t r = Find(parent, nb[0]);
      for (int q = 1; q < nn; ++q) {
        const uint16_t r2 = Find(parent, nb[q]);
        if (r2 < r) {
          parent[r] = r2;
          r = r2;
        } else if (r2 > r) {
          parent[r2] = r;
        }
      }
      lrow[x] = r;
    }
  }
  const uint32_t t_pass1 = micros() - phase;
  // compact in raster order of first pixel, gather stats
  uint16_t* cmap = S.cmap;
  for (int i = 0; i <= nprov; ++i) cmap[i] = 0;
  LComp* comps = S.comps;
  int nc = 0;
  for (int y = 0; y < H; ++y) {
    uint16_t* lrow = lab + y * W;
    const uint8_t* crow = S.cls + y * W;
    for (int x = 0; x < W; ++x) {
      if (!lrow[x]) continue;
      const uint16_t r = Find(parent, lrow[x]);
      uint16_t id = cmap[r];
      if (id == 0) {
        if (nc >= kMaxComps) {
          cmap[r] = kNone;
          overflow = true;
          lrow[x] = 0;
          continue;
        }
        id = static_cast<uint16_t>(++nc);
        cmap[r] = id;
        LComp& c = comps[id - 1];
        c.x1 = c.x2 = static_cast<uint16_t>(x);
        c.y1 = c.y2 = static_cast<uint16_t>(y);
        c.area = 0;
        c.pol = crow[x];
        c.f = 0;
      } else if (id == kNone) {
        lrow[x] = 0;
        continue;
      }
      LComp& c = comps[id - 1];
      ++c.area;
      if (x < c.x1) c.x1 = static_cast<uint16_t>(x);
      if (x > c.x2) c.x2 = static_cast<uint16_t>(x);
      c.y2 = static_cast<uint16_t>(y);
      lrow[x] = id;
    }
  }
  // drop specks below kMinArea and renumber
  {
    uint16_t* renum = cmap;   // reuse: old id -> new id
    int k = 0;
    for (int i = 0; i < nc; ++i) {
      if (comps[i].area < kMinArea) {
        renum[i + 1] = 0;
        continue;
      }
      renum[i + 1] = static_cast<uint16_t>(k + 1);
      comps[k++] = comps[i];
    }
    for (size_t p = 0; p < npx; ++p)
      if (lab[p]) lab[p] = renum[lab[p]];
    nc = k;
  }
  result->component_stack_overflow = overflow;
  const uint32_t t_ccl = micros() - phase;

  // ---- C: glyph test, polarity map sized from the text --------------------------------
  PolarityMap(bts, bbs, sw, sh, kPolR, 1, pol, s8a, s8b, s8c, s16a, s16b, s16c);
  GlyphTest(comps, nc, W, H, pol, sw, sh);
  {
    int m = 0;
    float* hs = S.f1;
    for (int i = 0; i < nc; ++i)
      if (IsGlyph(comps[i])) hs[m++] = static_cast<float>(comps[i].bh());
    if (m > 0) {
      std::sort(hs, hs + m);
      const float hmid = hs[m / 2];
      const int r = std::min(kPolRMax, std::max(kPolR, RoundEven(kPolLines * hmid * 2 / kDec)));
      const int er = std::max(1, RoundEven(0.15f * hmid / kDec));
      PolarityMap(bts, bbs, sw, sh, r, er, pol, s8a, s8b, s8c, s16a, s16b, s16c);
      GlyphTest(comps, nc, W, H, pol, sw, sh);
    }
  }
  int nglyph = 0;
  for (int i = 0; i < nc; ++i)
    if (IsGlyph(comps[i])) S.order[nglyph++] = static_cast<int16_t>(i);
  result->char_count = static_cast<uint16_t>(nglyph);
  result->character_pass_us = micros() - phase;
  if (log != nullptr)
    log->printf("LDET stage=components pass1_us=%u ccl_us=%u total_us=%u nprov=%d comps=%d\n",
                t_pass1, t_ccl, result->character_pass_us, nprov, nc);

  // ---- D: text angle by projection-profile sharpness ---------------------------------
  phase = micros();
  // Every stride-th glyph pixel in raster order, stride = max(4, ceil(n / 4000)) --
  // the reference's rule, so the two pick the same samples.
  // Internal RAM while this runs (the sample loop is the hot one), PSRAM if it is
  // short; nothing stays resident between detections.
  auto fast_alloc = [](size_t bytes) -> void* {
    void* q = heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    return q ? q : heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  };
  int16_t* ax = static_cast<int16_t*>(fast_alloc(kMaxAnglePoints * sizeof(int16_t)));
  int16_t* ay = static_cast<int16_t*>(fast_alloc(kMaxAnglePoints * sizeof(int16_t)));
  int32_t* ahist = static_cast<int32_t*>(fast_alloc(kHistBins * sizeof(int32_t)));
  if (!ax || !ay || !ahist) {
    heap_caps_free(ax); heap_caps_free(ay); heap_caps_free(ahist);
    S.Release();
    snprintf(result->error, sizeof(result->error), "alloc: angle samples");
    return false;
  }
  int npts = 0;
  {
    uint32_t total_glyph_px = 0;
    for (int i = 0; i < nc; ++i)
      if (IsGlyph(comps[i])) total_glyph_px += comps[i].area;
    const uint32_t stride =
        std::max<uint32_t>(4, (total_glyph_px + kMaxAnglePoints - 1) / kMaxAnglePoints);
    uint32_t seen = 0;
    for (int y = 0; y < H && npts < kMaxAnglePoints; ++y) {
      const uint16_t* lrow = lab + y * W;
      for (int x = 0; x < W && npts < kMaxAnglePoints; ++x) {
        const uint16_t id = lrow[x];
        if (!id || !IsGlyph(comps[id - 1])) continue;
        if (seen++ % stride == 0) {
          ax[npts] = static_cast<int16_t>(x);
          ay[npts] = static_cast<int16_t>(y);
          ++npts;
        }
      }
    }
  }
  auto sharp = [&](float deg) -> int64_t {
    const float t = deg * static_cast<float>(M_PI) / 180.0f;
    const float sn = sinf(t), cs = cosf(t);
    float vmin = 1e9f;
    int vmax_bin = 0;
    for (int i = 0; i < npts; ++i) vmin = std::min(vmin, -ax[i] * sn + ay[i] * cs);
    std::memset(ahist, 0, kHistBins * sizeof(int32_t));
    for (int i = 0; i < npts; ++i) {
      const int bin = Clamp(static_cast<int>(-ax[i] * sn + ay[i] * cs - vmin), 0, kHistBins - 1);
      ++ahist[bin];
      if (bin > vmax_bin) vmax_bin = bin;
    }
    int64_t s = 0;
    for (int i = 0; i <= vmax_bin; ++i) s += static_cast<int64_t>(ahist[i]) * ahist[i];
    return s;
  };
  float theta_deg = 0;
  if (nglyph >= 3 && npts > 0) {
    int64_t best_s = -1;
    for (int d = -kAngleRange; d <= kAngleRange; d += 2) {
      const int64_t s = sharp(static_cast<float>(d));
      if (s >= best_s) {   // ties go to the larger angle, as max() over (s, d) does
        best_s = s;
        theta_deg = static_cast<float>(d);
      }
    }
    const float b0 = theta_deg;
    for (int k = -8; k <= 8; ++k) {
      const float d = b0 + 0.25f * k;
      const int64_t s = sharp(d);
      if (s > best_s) {
        best_s = s;
        theta_deg = d;
      }
    }
  }
  heap_caps_free(ax);
  heap_caps_free(ay);
  heap_caps_free(ahist);
  double theta = theta_deg * M_PI / 180.0;
  const uint32_t t_angle = micros() - phase;
  uint32_t t_ext = 0, t_track = 0, t_fit = 0;

  // ---- E/F: lines, with up to three angle refinements --------------------------------
  LLine* lines = S.lines;
  int nl = 0;
  float ct = 1, st = 0;
  for (int it = 0; it < 4; ++it) {
    ct = static_cast<float>(cos(theta));
    st = static_cast<float>(sin(theta));
    for (int i = 0; i < nc; ++i) {
      comps[i].line = comps[i].line2 = comps[i].word = -1;
      comps[i].u1 = comps[i].v1 = 1e9f;
      comps[i].u2 = comps[i].v2 = -1e9f;
    }
    uint32_t tq = micros();
    for (int y = 0; y < H; ++y) {
      const uint16_t* lrow = lab + y * W;
      for (int x = 0; x < W; ++x) {
        const uint16_t id = lrow[x];
        if (!id) continue;
        LComp& c = comps[id - 1];
        const float u = x * ct + y * st, v = -x * st + y * ct;
        if (u < c.u1) c.u1 = u;
        if (u > c.u2) c.u2 = u;
        if (v < c.v1) c.v1 = v;
        if (v > c.v2) c.v2 = v;
      }
    }
    t_ext += micros() - tq;
    tq = micros();
    // E: left-to-right tracking
    std::stable_sort(S.order, S.order + nglyph,
                     [&](int16_t a, int16_t b) { return comps[a].u1 < comps[b].u1; });
    nl = 0;
    for (int oi = 0; oi < nglyph; ++oi) {
      LComp& c = comps[S.order[oi]];
      const float ch = c.h();
      int best = -1;
      float best_cost = 1e9f;
      for (int li = 0; li < nl; ++li) {
        const LLine& L = lines[li];
        if (L.pol != c.pol) continue;
        const float hm = L.hmed;
        const float mh = std::max(hm, ch);
        const float gap = c.u1 - L.u2;
        if (gap > kLinkGap * mh || gap < -0.5f * std::min(hm, ch)) continue;
        if (mh > kLinkRatio * std::min(hm, ch)) continue;
        const float pv = L.pred + L.pslope * (c.uc() - L.pred_u);
        const float dv = fabsf(c.vc() - pv);
        if (dv > kLinkDv * mh) continue;
        const float ov = std::min(c.v2, pv + hm / 2) - std::max(c.v1, pv - hm / 2);
        if (ov < 0.3f * std::min(ch, hm)) continue;
        const float cost = std::max(gap, 0.0f) / mh + 2 * dv / mh;
        if (cost < best_cost) {
          best = li;
          best_cost = cost;
        }
      }
      if (best < 0) {
        if (nl >= kMaxLines) continue;
        LLine& L = lines[nl];
        std::memset(&L, 0, sizeof(L));
        L.pol = c.pol;
        L.count = 1;
        L.ring[0] = S.order[oi];
        L.u2 = c.u2;
        L.hmed = ch;
        L.pred = c.vc();
        L.pred_u = c.uc();
        L.pslope = 0;
        c.line = static_cast<int16_t>(nl);
        ++nl;
      } else {
        LLine& L = lines[best];
        L.ring[L.count % 9] = S.order[oi];
        ++L.count;
        L.u2 = std::max(L.u2, c.u2);
        float t5[5], t5u[5], t9[9];
        const int n5 = std::min(L.count, 5), n9 = std::min(L.count, 9);
        for (int k = 0; k < n5; ++k) {
          t5[k] = comps[L.member(k)].vc();
          t5u[k] = comps[L.member(k)].uc();
        }
        for (int k = 0; k < n9; ++k) t9[k] = comps[L.member(k)].h();
        L.pred = MedianF(t5, n5);
        L.pred_u = MedianF(t5u, n5);
        L.hmed = MedianF(t9, n9);
        const int n8 = std::min(L.count, 8);
        if (n8 >= 4) {
          const LComp& newest = comps[L.member(0)];
          const LComp& oldest = comps[L.member(n8 - 1)];
          if (newest.uc() - oldest.uc() >= 3 * L.hmed) {
            float mu = 0, mv = 0;
            for (int k = 0; k < n8; ++k) {
              mu += comps[L.member(k)].uc();
              mv += comps[L.member(k)].vc();
            }
            mu /= n8;
            mv /= n8;
            float sxy = 0, sxx = 0;
            for (int k = 0; k < n8; ++k) {
              const float du = comps[L.member(k)].uc() - mu;
              sxy += du * (comps[L.member(k)].vc() - mv);
              sxx += du * du;
            }
            if (sxx > 0) L.pslope = std::max(-kTrackSlope, std::min(kTrackSlope, sxy / sxx));
          }
        }
        c.line = static_cast<int16_t>(best);
      }
    }
    // a lone glyph must look like a word
    {
      static int16_t remap[kMaxLines];
      int k = 0;
      for (int li = 0; li < nl; ++li) {
        if (lines[li].count == 1) {
          const LComp& m = comps[lines[li].ring[0]];
          if (!(m.w() >= 1.2f * m.h() && m.h() >= 6)) {
            remap[li] = -1;
            continue;
          }
        }
        remap[li] = static_cast<int16_t>(k);
        lines[k++] = lines[li];
      }
      for (int i = 0; i < nc; ++i)
        if (comps[i].line >= 0) comps[i].line = remap[comps[i].line];
      nl = k;
    }
    t_track += micros() - tq;
    tq = micros();
    // F: fit every line
    BuildCsr(comps, nc, nl, false, S.off, S.csr, S.cursor);
    for (int li = 0; li < nl; ++li) {
      const int n = S.off[li + 1] - S.off[li];
      lines[li].count = n;
      FitLine(lines[li], S.csr + S.off[li], n, comps, lab, W, ct, st, S.f1, S.f2, S.f3, S.keep,
              S.hist, S.tmp);
    }
    t_fit += micros() - tq;
    // refine the global angle from the lines' own slopes, weighted by length
    if (it >= 3) break;
    int ns2 = 0;
    for (int li = 0; li < nl; ++li)
      if (lines[li].count >= 4) {
        S.f1[ns2] = static_cast<float>(atan(lines[li].b));
        S.f2[ns2] = lines[li].u2 - lines[li].u1;
        ++ns2;
      }
    if (ns2 == 0) break;
    for (int i = 0; i < ns2; ++i) S.order[nglyph + i] = static_cast<int16_t>(i);
    int16_t* idx = S.order + nglyph;
    std::sort(idx, idx + ns2, [&](int16_t a, int16_t b) { return S.f1[a] < S.f1[b]; });
    double tot = 0, acc = 0;
    for (int i = 0; i < ns2; ++i) tot += S.f2[i];
    double med = S.f1[idx[ns2 - 1]];
    for (int i = 0; i < ns2; ++i) {
      acc += S.f2[idx[i]];
      if (acc >= tot / 2) {
        med = S.f1[idx[i]];
        break;
      }
    }
    if (fabs(med * 180.0 / M_PI) <= 0.75) break;
    theta += med;
  }
  result->skew_slope_q12 = static_cast<int16_t>(Clamp(static_cast<int>(tan(theta) * 4096), -32000, 32000));

  // drop tiny noise lines
  if (nl > 0) {
    int nb = 0;
    for (int li = 0; li < nl; ++li)
      if (lines[li].count >= 3) S.f1[nb++] = lines[li].xh;
    if (nb == 0)
      for (int li = 0; li < nl; ++li) S.f1[nb++] = lines[li].xh;
    const float ref = MedianF(S.f1, nb);
    static int16_t remap[kMaxLines];
    int k = 0;
    for (int li = 0; li < nl; ++li) {
      if (lines[li].count <= 2 && lines[li].xh < 0.45f * ref) {
        remap[li] = -1;
        continue;
      }
      remap[li] = static_cast<int16_t>(k);
      lines[k++] = lines[li];
    }
    for (int i = 0; i < nc; ++i)
      if (comps[i].line >= 0) comps[i].line = remap[comps[i].line];
    nl = k;
  }
  result->word_merge_us = micros() - phase;
  if (log != nullptr)
    log->printf("LDET stage=lines angle_us=%u extents_us=%u track_us=%u fit_us=%u total_us=%u pts=%d\n",
                t_angle, t_ext, t_track, t_fit, result->word_merge_us, npts);

  // ---- G: leftovers join the line whose band holds them ------------------------------
  phase = micros();
  for (int i = 0; i < nc; ++i) {
    LComp& c = comps[i];
    if (c.line >= 0 || (c.f & (kFBackground | kFCounter))) continue;
    int best = -1;
    double best_d = 1e18;
    int hits[16];
    int nh = 0;
    const float uc = c.uc(), vc = c.vc();
    for (int li = 0; li < nl; ++li) {
      const LLine& L = lines[li];
      if (L.pol != c.pol || L.xh <= 0) continue;
      if (uc < L.u1 - 1.2f * L.xh || uc > L.u2 + 1.2f * L.xh) continue;
      const double vb = L.base(uc);
      const double xl = vb - L.xh;
      if (c.v2 < xl - 1.3 * L.xh || c.v1 > vb + 0.8 * L.xh) continue;
      if (nh < 16) hits[nh++] = li;
      const double d = fabs(vc - (vb - 0.5 * L.xh));
      if (d < best_d) {
        best = li;
        best_d = d;
      }
    }
    if (best < 0) continue;
    const LLine& L = lines[best];
    c.line = static_cast<int16_t>(best);
    if (c.h() > 1.9f * L.xh) {
      for (int q = 0; q < nh; ++q) {
        if (hits[q] == best) continue;
        const LLine& M = lines[hits[q]];
        const double vb2 = M.base(uc);
        if (c.v1 <= vb2 - 0.5 * M.xh && c.v2 >= vb2 - 0.5 * M.xh) {
          c.line2 = static_cast<int16_t>(hits[q]);
          break;
        }
      }
    }
  }
  BuildCsr(comps, nc, nl, true, S.off, S.csr, S.cursor);

  // ---- H/I: words ---------------------------------------------------------------------
  LWord* words = S.words;
  int nw = 0, pool_used = 0;
  for (int li = 0; li < nl && nw < kMaxWordBoxes; ++li) {
    const LLine& L = lines[li];
    const float xh = L.xh;
    const int16_t* mem = S.csr + S.off[li];
    const int n = S.off[li + 1] - S.off[li];
    // 0 mark, 1 letter (interval material)
    uint8_t* role = S.keep;
    for (int k = 0; k < n; ++k) {
      const LComp& m = comps[mem[k]];
      const double vb = L.base(m.uc());
      const double top = m.v1 - vb, bot = m.v2 - vb;
      const double in_band = std::min(bot, 0.0) - std::max(top, static_cast<double>(-xh));
      const bool grounded = bot >= -0.2 * xh;
      const bool inked = m.area >= 0.06f * xh * xh;
      role[k] = ((m.h() >= kLetterH * xh && in_band >= 0.45 * xh && grounded && inked) ||
                 m.line2 >= 0) ? 1 : 0;
    }
    int nlet = 0;
    for (int k = 0; k < n; ++k) nlet += role[k];
    if (nlet == 0) continue;
    for (int k = 0; k < n; ++k) {
      if (role[k]) continue;
      const LComp& m = comps[mem[k]];
      const double vb = L.base(m.uc());
      if (m.v2 - vb >= -0.9 * xh && m.v1 - vb <= -0.25 * xh) role[k] = 2;   // solid
    }
    // letters (incl. solid) sorted by u1
    int16_t* let = S.grp;     // positions into mem
    int nlt = 0;
    for (int k = 0; k < n; ++k)
      if (role[k]) let[nlt++] = static_cast<int16_t>(k);
    std::stable_sort(let, let + nlt, [&](int16_t a, int16_t b) { return comps[mem[a]].u1 < comps[mem[b]].u1; });
    // clusters
    int16_t* cl_of = S.cl_of;
    float* cl_u1 = S.cl_u1;
    float* cl_u2 = S.cl_u2;
    int ncl = 0;
    float cu2 = -1e9f;
    for (int q = 0; q < nlt; ++q) {
      const LComp& m = comps[mem[let[q]]];
      if (ncl > 0 && m.u1 <= cu2 + 0.02f * xh) {
        cu2 = std::max(cu2, m.u2);
        cl_u1[ncl - 1] = std::min(cl_u1[ncl - 1], m.u1);
        cl_u2[ncl - 1] = std::max(cl_u2[ncl - 1], m.u2);
      } else {
        cl_u1[ncl] = m.u1;
        cl_u2[ncl] = m.u2;
        cu2 = m.u2;
        ++ncl;
      }
      cl_of[let[q]] = static_cast<int16_t>(ncl - 1);
    }
    float* gaps = S.f1;
    for (int k = 0; k + 1 < ncl; ++k) gaps[k] = (cl_u1[k + 1] - cl_u2[k] - 1) / xh;
    const int ng = std::max(0, ncl - 1);
    float tau = kTauDefault;
    if (ng >= 4) {
      for (int k = 0; k < ng; ++k) S.f2[k] = gaps[k];
      const float t = Otsu1D(S.f2, ng);
      double lo_s = 0, hi_s = 0;
      int lo_n = 0, hi_n = 0;
      float hi_min = 1e9f;
      for (int k = 0; k < ng; ++k) {
        if (gaps[k] <= t) {
          lo_s += gaps[k];
          ++lo_n;
        } else {
          hi_s += gaps[k];
          ++hi_n;
          hi_min = std::min(hi_min, gaps[k]);
        }
      }
      if (lo_n && hi_n && t >= kTauMin && t <= kTauMax &&
          hi_s / hi_n >= 1.8 * std::max(lo_s / lo_n, 0.05) && hi_min >= 0.3f)
        tau = std::min(kTauMax, std::max(kTauMin, t));
    }
    // cluster -> word within this line
    int16_t* cl_word = S.cl_word;
    uint8_t* w_unc = S.w_unc;
    int nlw = 1;
    cl_word[0] = 0;
    w_unc[0] = 0;
    for (int k = 0; k < ng; ++k) {
      if (gaps[k] >= tau) {
        w_unc[nlw] = 0;
        ++nlw;
      }
      cl_word[k + 1] = static_cast<int16_t>(nlw - 1);
      if (fabsf(gaps[k] - tau) < 0.08f) w_unc[nlw - 1] = 1;
    }
    // per member: word within line (-1 = unattached)
    int16_t* mw = S.grp + n;   // after let[]
    float* sp_u1 = S.sp_u1;
    float* sp_u2 = S.sp_u2;
    for (int w = 0; w < nlw; ++w) {
      sp_u1[w] = 1e9f;
      sp_u2[w] = -1e9f;
    }
    for (int k = 0; k < n; ++k) {
      mw[k] = -1;
      if (!role[k]) continue;
      mw[k] = cl_word[cl_of[k]];
      const LComp& m = comps[mem[k]];
      sp_u1[mw[k]] = std::min(sp_u1[mw[k]], m.u1);
      sp_u2[mw[k]] = std::max(sp_u2[mw[k]], m.u2);
    }
    for (int k = 0; k < n; ++k) {
      if (role[k]) continue;
      const float muc = comps[mem[k]].uc();
      int best = -1;
      float bd = 1e9f;
      for (int w = 0; w < nlw; ++w) {
        const float a = sp_u1[w], b = sp_u2[w];
        const float d = (a - 0.2f * xh <= muc && muc <= b + 0.2f * xh)
                            ? 0.0f
                            : std::min(fabsf(muc - a), fabsf(muc - b));
        if (d < bd - 1e-6f) {
          best = w;
          bd = d;
        }
      }
      if (best >= 0 && bd <= 0.9f * xh) mw[k] = static_cast<int16_t>(best);
    }
    // emit this line's words
    for (int w = 0; w < nlw && nw < kMaxWordBoxes; ++w) {
      LWord& o = words[nw];
      o.line = static_cast<int16_t>(li);
      o.pol = L.pol;
      o.xh = xh;
      o.u1 = 1e9f;
      o.u2 = -1e9f;
      o.top_rel = 1e9f;
      o.bot_rel = -1e9f;
      o.ix1 = W;
      o.iy1 = H;
      o.ix2 = -1;
      o.iy2 = -1;
      o.letters = 0;
      o.area = 0;
      o.uncertain = w_unc[w] != 0;
      o.pool_at = static_cast<int16_t>(pool_used);
      o.pool_n = 0;
      for (int k = 0; k < n; ++k) {
        if (mw[k] != w) continue;
        const LComp& m = comps[mem[k]];
        if (pool_used < 2 * kMaxComps) {
          S.pool[pool_used++] = mem[k];
          ++o.pool_n;
        }
        o.u1 = std::min(o.u1, m.u1);
        o.u2 = std::max(o.u2, m.u2);
        const double vb = L.base(m.uc());
        double t = m.v1 - vb, b = m.v2 - vb;
        if (m.line2 >= 0) {
          t = std::max(t, -1.9 * xh);
          b = std::min(b, 0.8 * xh);
        }
        o.top_rel = std::min(o.top_rel, static_cast<float>(t));
        o.bot_rel = std::max(o.bot_rel, static_cast<float>(b));
        if (m.h() >= kLetterH * xh) ++o.letters;
        o.area += m.area;
        o.ix1 = std::min<int>(o.ix1, m.x1);
        o.iy1 = std::min<int>(o.iy1, m.y1);
        o.ix2 = std::max<int>(o.ix2, m.x2);
        o.iy2 = std::max<int>(o.iy2, m.y2);
      }
      if (o.pool_n == 0) continue;
      o.uc = 0.5f * (o.u1 + o.u2);
      auto to_frame = [&](double uu, double vv, float* fx, float* fy) {
        *fx = static_cast<float>(uu * ct - vv * st);
        *fy = static_cast<float>(uu * st + vv * ct);
      };
      const double vbc = L.base(o.uc);
      to_frame(o.uc, vbc, &o.bx, &o.by);
      o.ang = static_cast<float>(theta + atan(L.slope(o.uc)));
      const double qt = std::min(static_cast<double>(o.top_rel), static_cast<double>(-xh)) - kPadV * xh;
      const double qb = std::max(static_cast<double>(o.bot_rel), 0.0) + kPadV * xh;
      const double ua = o.u1 - kPadU * xh, ub = o.u2 + kPadU * xh;
      to_frame(ua, L.base(ua) + qt, &o.quad[0], &o.quad[1]);
      to_frame(ub, L.base(ub) + qt, &o.quad[2], &o.quad[3]);
      to_frame(ub, L.base(ub) + qb, &o.quad[4], &o.quad[5]);
      to_frame(ua, L.base(ua) + qb, &o.quad[6], &o.quad[7]);
      o.trunc_l = o.ix1 <= 1;
      o.trunc_t = o.iy1 <= 1;
      o.trunc_r = o.ix2 >= W - 2;
      o.trunc_b = o.iy2 >= H - 2;
      const float mu = kTruncU * xh;
      for (int e = 0; e < 2; ++e) {
        float ex, ey;
        to_frame(e == 0 ? o.u1 : o.u2, vbc - 0.5 * xh, &ex, &ey);
        if (ex <= mu) o.trunc_l = true;
        if (ey <= mu) o.trunc_t = true;
        if (ex >= W - 1 - mu) o.trunc_r = true;
        if (ey >= H - 1 - mu) o.trunc_b = true;
      }
      o.key_line = static_cast<float>(
          RoundEven(static_cast<float>(L.base(0.5 * (L.u1 + L.u2)) / std::max(1.0f, 0.7f * xh))));
      o.key_u = o.u1;
      ++nw;
    }
  }
  if (nw >= kMaxWordBoxes) result->box_capacity_reached = true;

  // reading order
  static int16_t word_order[kMaxWordBoxes];
  for (int i = 0; i < nw; ++i) word_order[i] = static_cast<int16_t>(i);
  std::stable_sort(word_order, word_order + nw, [&](int16_t a, int16_t b) {
    if (words[a].key_line != words[b].key_line) return words[a].key_line < words[b].key_line;
    return words[a].key_u < words[b].key_u;
  });

  // ownership: a comp belongs to the first word (in reading order) that holds it;
  // straddlers belong to nobody, so no crop ever paints them out
  for (int i = 0; i <= nc; ++i) owner_[i] = kNone;
  for (int r = 0; r < nw; ++r) {
    const LWord& o = words[word_order[r]];
    for (int q = 0; q < o.pool_n; ++q) {
      const int ci = S.pool[o.pool_at + q];
      if (comps[ci].word < 0) comps[ci].word = static_cast<int16_t>(r);
    }
  }
  for (int i = 0; i < nc; ++i)
    if (comps[i].word >= 0 && comps[i].line2 < 0) owner_[i + 1] = static_cast<uint16_t>(comps[i].word);

  // ---- output ---------------------------------------------------------------------------
  float xhs[kMaxLines];
  for (int li = 0; li < nl; ++li) xhs[li] = lines[li].xh;
  const float med_xh = nl ? MedianF(xhs, nl) : 0.0f;
  result->median_char_height_x2 = static_cast<uint16_t>(RoundEven(2 * med_xh));
  for (int r = 0; r < nw; ++r) {
    const LWord& o = words[word_order[r]];
    WordBox& b = result->boxes[r];
    std::memset(&b, 0, sizeof(b));
    float qx1 = 1e9f, qy1 = 1e9f, qx2 = -1e9f, qy2 = -1e9f;
    for (int k = 0; k < 4; ++k) {
      qx1 = std::min(qx1, o.quad[2 * k]);
      qx2 = std::max(qx2, o.quad[2 * k]);
      qy1 = std::min(qy1, o.quad[2 * k + 1]);
      qy2 = std::max(qy2, o.quad[2 * k + 1]);
      b.quad[2 * k] = static_cast<int16_t>(lrintf(o.quad[2 * k]));
      b.quad[2 * k + 1] = static_cast<int16_t>(lrintf(o.quad[2 * k + 1]));
    }
    b.x1 = static_cast<uint16_t>(Clamp(static_cast<int>(floorf(qx1)), 0, W - 1));
    b.y1 = static_cast<uint16_t>(Clamp(static_cast<int>(floorf(qy1)), 0, H - 1));
    b.x2 = static_cast<uint16_t>(Clamp(static_cast<int>(ceilf(qx2)), 0, W - 1));
    b.y2 = static_cast<uint16_t>(Clamp(static_cast<int>(ceilf(qy2)), 0, H - 1));
    b.ink_x1 = static_cast<uint16_t>(Clamp(o.ix1, 0, W - 1));
    b.ink_y1 = static_cast<uint16_t>(Clamp(o.iy1, 0, H - 1));
    b.ink_x2 = static_cast<uint16_t>(Clamp(o.ix2, 0, W - 1));
    b.ink_y2 = static_cast<uint16_t>(Clamp(o.iy2, 0, H - 1));
    b.baseline_y = static_cast<uint16_t>(Clamp(static_cast<int>(lrintf(o.by)), 0, H - 1));
    b.area = o.area;
    b.char_count = static_cast<uint16_t>(o.letters);
    uint8_t fl = 0;
    if (o.trunc_l) fl |= kWordFlagTruncLeft;
    if (o.trunc_r) fl |= kWordFlagTruncRight;
    if (o.trunc_t) fl |= kWordFlagTruncTop;
    if (o.trunc_b) fl |= kWordFlagTruncBottom;
    if (o.letters == 0) fl |= kWordFlagPunctuation;
    if (o.uncertain) fl |= kWordFlagSplit;
    b.flags = fl;
    b.touches_edge = (fl & kWordFlagTruncAny) != 0;
    b.geom = 1;
    b.polarity = o.pol;
    b.line_id = static_cast<uint16_t>(o.line);
    b.word_index = static_cast<uint16_t>(r);
    b.xh_x16 = static_cast<int16_t>(Clamp(static_cast<int>(lrintf(o.xh * 16)), 1, 32767));
    b.base_x_x16 = static_cast<int32_t>(lrintf(o.bx * 16));
    b.base_y_x16 = static_cast<int32_t>(lrintf(o.by * 16));
    b.dir_x_q14 = static_cast<int16_t>(lrintf(cosf(o.ang) * 16384));
    b.dir_y_q14 = static_cast<int16_t>(lrintf(sinf(o.ang) * 16384));
    b.u1_x16 = static_cast<int16_t>(Clamp(static_cast<int>(lrintf((o.u1 - o.uc) * 16)), -32767, 32767));
    b.u2_x16 = static_cast<int16_t>(Clamp(static_cast<int>(lrintf((o.u2 - o.uc) * 16)), -32767, 32767));
    b.top_x16 = static_cast<int16_t>(Clamp(static_cast<int>(lrintf(o.top_rel * 16)), -32767, 32767));
    b.bot_x16 = static_cast<int16_t>(Clamp(static_cast<int>(lrintf(o.bot_rel * 16)), -32767, 32767));
    if (b.touches_edge) ++result->truncated_count;
    if (fl & kWordFlagPunctuation) ++result->punctuation_count;
    if (b.recognizable()) ++result->recognizable_count;
    if (o.uncertain) ++result->split_count;
  }
  result->count = static_cast<size_t>(nw);
  result->word_components_us = micros() - phase;

  ctx_.labels = labels_;
  ctx_.owner = owner_;
  ctx_.close_s = close_s_;
  ctx_.open_s = open_s_;
  ctx_.width = static_cast<uint16_t>(W);
  ctx_.height = static_cast<uint16_t>(H);
  ctx_.sw = static_cast<uint16_t>(sw);
  ctx_.sh = static_cast<uint16_t>(sh);
  ctx_.n_labels = static_cast<uint16_t>(nc);
  result->crop = &ctx_;

  S.Release();
  result->total_us = micros() - started;
  if (log != nullptr) {
    log->printf("LDET done comps=%d glyphs=%d lines=%d boxes=%u green=%u angle=%.2f xh=%.1f "
                "overflow=%u us=%u (contrast=%u comps=%u lines=%u words=%u)\n",
                nc, nglyph, nl, static_cast<unsigned>(result->count), result->recognizable_count,
                theta * 180.0 / M_PI, med_xh, overflow ? 1U : 0U, result->total_us,
                result->background_us, result->character_pass_us, result->word_merge_us,
                result->word_components_us);
  }
  return true;
}

}  // namespace ocr_demo
