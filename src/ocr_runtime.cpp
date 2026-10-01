#include "ocr_runtime.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <new>

#include "esp_heap_caps.h"
#include "tiny_ocr_48x320_int8.h"

namespace ocr_demo {
namespace {

bool ShapeEquals(const TfLiteTensor* tensor, const int* expected, int rank) {
  if (tensor == nullptr || tensor->dims == nullptr || tensor->dims->size != rank) return false;
  for (int i = 0; i < rank; ++i) {
    if (tensor->dims->data[i] != expected[i]) return false;
  }
  return true;
}

uint8_t ClampByte(int value) {
  return static_cast<uint8_t>(std::max(0, std::min(255, value)));
}

}  // namespace

OcrRuntime::OcrRuntime()
    : model_(nullptr),
      interpreter_(nullptr),
      input_(nullptr),
      output_(nullptr),
      arena_raw_(nullptr),
      arena_(nullptr),
      arena_used_bytes_(0),
      ready_(false) {}

bool OcrRuntime::Begin(Stream& log) {
  ready_ = false;
  model_ = tflite::GetModel(g_ocr_model);
  if (model_ == nullptr) {
    log.println("ERROR model pointer is null");
    return false;
  }
  if (model_->version() != TFLITE_SCHEMA_VERSION) {
    log.printf("ERROR schema model=%d runtime=%d\n", model_->version(), TFLITE_SCHEMA_VERSION);
    return false;
  }
  if (tiny_ocr::RegisterOcrOps(resolver_) != kTfLiteOk) {
    log.println("ERROR operator registration failed");
    return false;
  }

  arena_raw_ = heap_caps_malloc(kTensorArenaBytes + 16,
                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (arena_raw_ == nullptr) {
    log.printf("ERROR PSRAM arena allocation failed bytes=%u free_psram=%u largest=%u\n",
               static_cast<unsigned>(kTensorArenaBytes),
               static_cast<unsigned>(ESP.getFreePsram()),
               static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM)));
    return false;
  }
  const uintptr_t raw = reinterpret_cast<uintptr_t>(arena_raw_);
  arena_ = reinterpret_cast<uint8_t*>((raw + 15U) & ~static_cast<uintptr_t>(15U));

  interpreter_ = new (std::nothrow)
      tflite::MicroInterpreter(model_, resolver_, arena_, kTensorArenaBytes);
  if (interpreter_ == nullptr) {
    log.println("ERROR interpreter construction failed");
    return false;
  }

  const uint32_t allocation_started = micros();
  if (interpreter_->AllocateTensors() != kTfLiteOk) {
    log.printf("ERROR AllocateTensors failed arena_bytes=%u elapsed_us=%u\n",
               static_cast<unsigned>(kTensorArenaBytes), micros() - allocation_started);
    return false;
  }
  arena_used_bytes_ = interpreter_->arena_used_bytes();
  input_ = interpreter_->input(0);
  output_ = interpreter_->output(0);
  if (!ValidateTensorContract(log)) return false;

  ready_ = true;
  log.printf("MODEL_READY arena_capacity=%u arena_used=%u allocation_us=%u free_psram=%u\n",
             static_cast<unsigned>(kTensorArenaBytes),
             static_cast<unsigned>(arena_used_bytes_), micros() - allocation_started,
             static_cast<unsigned>(ESP.getFreePsram()));
  return true;
}

bool OcrRuntime::ValidateTensorContract(Stream& log) const {
  const int expected_input[] = {1, 1, tiny_ocr::kInputHeight, tiny_ocr::kInputWidth};
  const int expected_output[] = {tiny_ocr::kTimeSteps, 1, tiny_ocr::kClassCount};
  if (input_ == nullptr || input_->type != kTfLiteInt8 ||
      !ShapeEquals(input_, expected_input, 4)) {
    log.println("ERROR input tensor does not match INT8 [1,1,48,320]");
    return false;
  }
  if (output_ == nullptr || output_->type != kTfLiteInt8 ||
      !ShapeEquals(output_, expected_output, 3)) {
    log.println("ERROR output tensor does not match INT8 [80,1,85]");
    return false;
  }
  if (std::fabs(input_->params.scale - tiny_ocr::kInputScale) > 1e-7f ||
      input_->params.zero_point != tiny_ocr::kInputZeroPoint) {
    log.printf("ERROR input quantization scale=%.9g zp=%d\n", input_->params.scale,
               input_->params.zero_point);
    return false;
  }
  if (std::fabs(output_->params.scale - tiny_ocr::kOutputScale) > 1e-6f ||
      output_->params.zero_point != tiny_ocr::kOutputZeroPoint) {
    log.printf("ERROR output quantization scale=%.9g zp=%d\n", output_->params.scale,
               output_->params.zero_point);
    return false;
  }
  return true;
}

uint8_t OcrRuntime::SampleArea(const GrayImage& image, const CropRect& crop,
                               int dx, int dy, int resized_width,
                               int resized_height) const {
  const int source_width = crop.width();
  const int source_height = crop.height();
  const int sx0 = crop.x1 + (dx * source_width) / resized_width;
  const int sx1 = crop.x1 + std::max((dx + 1) * source_width / resized_width, 1 + dx * source_width / resized_width);
  const int sy0 = crop.y1 + (dy * source_height) / resized_height;
  const int sy1 = crop.y1 + std::max((dy + 1) * source_height / resized_height, 1 + dy * source_height / resized_height);
  uint32_t sum = 0;
  uint32_t samples = 0;
  for (int y = sy0; y < std::min<int>(sy1, crop.y2 + 1); ++y) {
    for (int x = sx0; x < std::min<int>(sx1, crop.x2 + 1); ++x) {
      sum += image.pixels[y * image.width + x];
      ++samples;
    }
  }
  return samples == 0 ? 255 : static_cast<uint8_t>(sum / samples);
}

uint8_t OcrRuntime::SampleBilinear(const GrayImage& image, const CropRect& crop,
                                   int dx, int dy, int resized_width,
                                   int resized_height) const {
  const int source_width = crop.width();
  const int source_height = crop.height();

  // Q16 fixed-point version of:
  //   f = (d + 0.5) * source / resized - 0.5
  // The resize path therefore stays integer before the INT8 lookup table.
  int64_t fx_q16 =
      (static_cast<int64_t>(2 * dx + 1) * source_width * 32768LL) / resized_width
      - 32768LL;
  int64_t fy_q16 =
      (static_cast<int64_t>(2 * dy + 1) * source_height * 32768LL) / resized_height
      - 32768LL;
  const int64_t max_x_q16 = static_cast<int64_t>(source_width - 1) << 16;
  const int64_t max_y_q16 = static_cast<int64_t>(source_height - 1) << 16;
  fx_q16 = std::max<int64_t>(0, std::min<int64_t>(fx_q16, max_x_q16));
  fy_q16 = std::max<int64_t>(0, std::min<int64_t>(fy_q16, max_y_q16));

  const int x0 = static_cast<int>(fx_q16 >> 16);
  const int y0 = static_cast<int>(fy_q16 >> 16);
  const int x1 = std::min(x0 + 1, source_width - 1);
  const int y1 = std::min(y0 + 1, source_height - 1);
  const uint32_t wx = static_cast<uint32_t>(fx_q16 & 0xFFFF);
  const uint32_t wy = static_cast<uint32_t>(fy_q16 & 0xFFFF);
  const uint32_t iwx = 65536U - wx;
  const uint32_t iwy = 65536U - wy;

  const uint8_t p00 = image.pixels[(crop.y1 + y0) * image.width + crop.x1 + x0];
  const uint8_t p01 = image.pixels[(crop.y1 + y0) * image.width + crop.x1 + x1];
  const uint8_t p10 = image.pixels[(crop.y1 + y1) * image.width + crop.x1 + x0];
  const uint8_t p11 = image.pixels[(crop.y1 + y1) * image.width + crop.x1 + x1];

  const uint64_t top_q16 = static_cast<uint64_t>(p00) * iwx +
                           static_cast<uint64_t>(p01) * wx;
  const uint64_t bottom_q16 = static_cast<uint64_t>(p10) * iwx +
                              static_cast<uint64_t>(p11) * wx;
  const uint64_t value_q32 = top_q16 * iwy + bottom_q16 * wy;
  return ClampByte(static_cast<int>((value_q32 + (1ULL << 31)) >> 32));
}

uint8_t OcrRuntime::BuildCropLut(const GrayImage& image, const CropRect& crop,
                                 int8_t* lut) const {
  // Histogram the source crop so the stretch uses this word's own ink and
  // paper levels rather than a global assumption about exposure.
  uint32_t histogram[256] = {};
  uint32_t total = 0;
  for (int y = crop.y1; y <= crop.y2; ++y) {
    const uint8_t* row = image.pixels + static_cast<size_t>(y) * image.width;
    for (int x = crop.x1; x <= crop.x2; ++x) {
      ++histogram[row[x]];
      ++total;
    }
  }
  int low = 0;
  int high = 255;
  if (total > 0) {
    // 2nd and 98th percentile: robust to a few stray dark or blown-out pixels
    // at the crop border, which a plain min/max would latch onto.
    const uint32_t low_target = (total * 2U) / 100U;
    const uint32_t high_target = (total * 98U) / 100U;
    uint32_t seen = 0;
    for (int g = 0; g < 256; ++g) {
      seen += histogram[g];
      if (seen > low_target) {
        low = g;
        break;
      }
    }
    seen = 0;
    for (int g = 0; g < 256; ++g) {
      seen += histogram[g];
      if (seen >= high_target) {
        high = g;
        break;
      }
    }
  }

  const int span = high - low;
  if (span < kMinContrastSpan) {
    // Nothing to stretch: a blank or uniformly flat crop. Amplifying it would
    // only amplify sensor noise into fake strokes.
    std::memcpy(lut, tiny_ocr::kInputQuantLut, 256 * sizeof(int8_t));
    return static_cast<uint8_t>(std::max(0, span));
  }
  const int range = kStretchCeiling - kStretchFloor;
  for (int g = 0; g < 256; ++g) {
    const int stretched = ((g - low) * range) / span + kStretchFloor;
    lut[g] = tiny_ocr::kInputQuantLut[ClampByte(stretched)];
  }
  return static_cast<uint8_t>(span);
}

bool OcrRuntime::PrepareInput(const GrayImage& image, const WordBox& box,
                              int page_char_height, OcrResult* result) {
  if (input_ == nullptr || input_->data.int8 == nullptr || box.x2 < box.x1 || box.y2 < box.y1 ||
      box.x2 >= image.width || box.y2 >= image.height) {
    return false;
  }
  const int available_width = tiny_ocr::kInputWidth - 2 * kCropMarginX;

  CropRect crop{box.x1, box.y1, box.x2, box.y2};
  int resized_width = 1;
  int resized_height = 1;
  int top = 0;
  bool fallback = false;  // true = page scale had to be reduced for this box
  bool scaled = false;    // true = baseline-anchored path was used

  if (page_char_height > 0 && box.ink_x2 >= box.ink_x1 && box.ink_y2 >= box.ink_y1) {
    // Page-constant scale: kTargetCharHeight input rows per median character
    // height, for every box on the page, so a word's glyph scale no longer
    // depends on which glyphs it happens to contain.
    const int32_t page_scale_q16 =
        (static_cast<int32_t>(kTargetCharHeight) << 16) / page_char_height;
    const int pad_source = static_cast<int>(
        ((static_cast<int64_t>(kInkPadInput) << 16) + page_scale_q16 - 1) / page_scale_q16);
    crop = CropRect{
        std::max(0, static_cast<int>(box.ink_x1) - pad_source),
        std::max(0, static_cast<int>(box.ink_y1) - pad_source),
        std::min(image.width - 1, static_cast<int>(box.ink_x2) + pad_source),
        std::min(image.height - 1, static_cast<int>(box.ink_y2) + pad_source)};

    // Cap the page scale at what this crop can actually fit, rather than
    // abandoning baseline placement for a word that is merely tall. On a
    // blurred capture the median component height underestimates x-height -
    // measured ink/char height is 1.59 median and 2.54 max on real photos
    // against 1.00 median on clean renders - so a strict fit test sends
    // essentially every real box down the fallback path, which is exactly
    // what the first device run showed. Shrinking keeps the baseline anchor,
    // which is the part that fixes the x-height words.
    const int32_t fit_height_q16 =
        (static_cast<int32_t>(tiny_ocr::kInputHeight) << 16) / crop.height();
    const int32_t fit_width_q16 =
        (static_cast<int32_t>(available_width) << 16) / crop.width();
    const int32_t scale_q16 =
        std::min(page_scale_q16, std::min(fit_height_q16, fit_width_q16));

    if (scale_q16 > 0) {
      resized_width = std::max<int>(1, static_cast<int>(
          (static_cast<int64_t>(crop.width()) * scale_q16) >> 16));
      resized_height = std::max<int>(1, static_cast<int>(
          (static_cast<int64_t>(crop.height()) * scale_q16) >> 16));
      resized_width = std::min(resized_width, available_width);
      resized_height = std::min(resized_height, tiny_ocr::kInputHeight);
      const int baseline_offset = static_cast<int>(box.baseline_y) - crop.y1;
      top = kBaselineRow - static_cast<int>(
          (static_cast<int64_t>(baseline_offset) * scale_q16) >> 16);
      top = std::max(0, std::min(top, tiny_ocr::kInputHeight - resized_height));
      // Reported so the preview can show which boxes could not be held at the
      // page scale; they are still baseline-placed, not re-centred.
      fallback = scale_q16 < page_scale_q16;
      scaled = true;
    }
  }

  if (!scaled) {
    // No usable page statistic. Aspect-fit the padded box the way the original
    // firmware did, still honouring the side margins.
    crop = CropRect{box.x1, box.y1, box.x2, box.y2};
    const int source_width = crop.width();
    const int source_height = crop.height();
    if (static_cast<int64_t>(tiny_ocr::kInputHeight) * source_width <=
        static_cast<int64_t>(available_width) * source_height) {
      resized_height = tiny_ocr::kInputHeight;
      resized_width = static_cast<int>((static_cast<int64_t>(source_width) *
                                        tiny_ocr::kInputHeight + source_height / 2) /
                                       source_height);
    } else {
      resized_width = available_width;
      resized_height = static_cast<int>((static_cast<int64_t>(source_height) *
                                         available_width + source_width / 2) /
                                        source_width);
    }
    resized_width = std::max(1, std::min(available_width, resized_width));
    resized_height = std::max(1, std::min(tiny_ocr::kInputHeight, resized_height));
    top = (tiny_ocr::kInputHeight - resized_height) / 2;
  }

  int8_t crop_lut[256];
  const uint8_t span = BuildCropLut(image, crop, crop_lut);

  std::fill(input_->data.int8,
            input_->data.int8 + tiny_ocr::kInputHeight * tiny_ocr::kInputWidth,
            crop_lut[255]);
  const bool downsampling =
      resized_width < crop.width() || resized_height < crop.height();
  for (int y = 0; y < resized_height; ++y) {
    int8_t* row = input_->data.int8 + static_cast<size_t>(top + y) * tiny_ocr::kInputWidth;
    for (int x = 0; x < resized_width; ++x) {
      const uint8_t gray = downsampling
          ? SampleArea(image, crop, x, y, resized_width, resized_height)
          : SampleBilinear(image, crop, x, y, resized_width, resized_height);
      row[kCropMarginX + x] = crop_lut[gray];
    }
  }

  if (result != nullptr) {
    result->resized_width = static_cast<uint8_t>(std::min(255, resized_width));
    result->resized_height = static_cast<uint8_t>(std::min(255, resized_height));
    result->input_top = static_cast<uint8_t>(std::min(255, top));
    result->scale_fallback = fallback;
    result->crop_span = span;
  }
  return true;
}

namespace {
inline float EnvelopeAt(const uint8_t* small, int sw, int sh, int x, int y) {
  float fx = (x - 1.5f) / 4.0f, fy = (y - 1.5f) / 4.0f;
  fx = std::max(0.0f, std::min(fx, static_cast<float>(sw - 1)));
  fy = std::max(0.0f, std::min(fy, static_cast<float>(sh - 1)));
  const int x0 = static_cast<int>(fx), y0 = static_cast<int>(fy);
  const int x1 = std::min(x0 + 1, sw - 1), y1 = std::min(y0 + 1, sh - 1);
  const float wx = fx - x0, wy = fy - y0;
  return (small[y0 * sw + x0] * (1 - wx) + small[y0 * sw + x1] * wx) * (1 - wy) +
         (small[y1 * sw + x0] * (1 - wx) + small[y1 * sw + x1] * wx) * wy;
}
}  // namespace

// The word is resampled along its own baseline (mirror of pc_pipeline/line_detector/
// ocr_crop.py): the line's x-height lands on kLineTargetXh rows, the baseline on row
// 48 - T/2 - 3, paper stays around the ink, ink owned by OTHER words reads as the local
// paper level, and a light-on-dark word is inverted -- the recognizer has only ever
// seen dark ink on light paper.
bool OcrRuntime::PrepareLineInput(const GrayImage& image, const WordBox& box,
                                  const CropContext& ctx, OcrResult* result) {
  if (input_ == nullptr || input_->data.int8 == nullptr || box.xh_x16 <= 0) return false;
  const int W = image.width, H = image.height;
  const int IH = tiny_ocr::kInputHeight, IW = tiny_ocr::kInputWidth;
  const float xh = box.xh_x16 / 16.0f;
  const float u1 = box.u1_x16 / 16.0f, u2 = box.u2_x16 / 16.0f;
  const float top_rel = box.top_x16 / 16.0f, bot_rel = box.bot_x16 / 16.0f;
  const float bx = box.base_x_x16 / 16.0f, by = box.base_y_x16 / 16.0f;
  const float ca = box.dir_x_q14 / 16384.0f, sa = box.dir_y_q14 / 16384.0f;
  const bool light = box.polarity == 2;
  const uint8_t* env = light ? ctx.open_s : ctx.close_s;

  float sc = kLineTargetXh / xh;
  const int B = static_cast<int>(lrintf(IH - 0.5f * kLineTargetXh - 3));
  const float wu1 = u1 - kLinePadU * xh, wu2 = u2 + kLinePadU * xh;
  const float wv1 = top_rel - kLinePadV * xh, wv2 = bot_rel + kLinePadV * xh;
  const float cw = wu2 - wu1 + 1, ch = wv2 - wv1 + 1;
  const float inkh = bot_rel - top_rel + 1;
  const float fit = std::min(1.0f, std::min((IW - kLineLeft) / (cw * sc), 46.0f / (inkh * sc)));
  sc *= fit;
  const int rw = std::max(1, std::min(IW - kLineLeft, static_cast<int>(lrintf(cw * sc))));
  const int rh = std::max(1, static_cast<int>(lrintf(ch * sc)));
  int top = fit >= 1.0f ? static_cast<int>(lrintf(B + wv1 * sc))
                        : static_cast<int>(lrintf((IH - rh) / 2.0f));
  const int ink_top = static_cast<int>(lrintf((top_rel - wv1) * sc));
  const int ink_bot = static_cast<int>(lrintf((bot_rel - wv1 + 1) * sc));
  top = std::min(std::max(top, 1 - ink_top), IH - 1 - ink_bot);
  const int n = std::min(4, std::max(1, static_cast<int>(ceilf(1.0f / sc))));
  const uint16_t me = box.word_index;

  auto pixel = [&](int xx, int yy) -> float {
    const uint16_t lab = ctx.labels[yy * W + xx];
    if (lab != 0 && lab <= ctx.n_labels) {
      const uint16_t own = ctx.owner[lab];
      if (own != 0xFFFF && own != me) {       // another word's ink: paper instead
        const float e = EnvelopeAt(env, ctx.sw, ctx.sh, xx, yy);
        return light ? 255.0f - e : e;
      }
    }
    const float v = image.pixels[yy * W + xx];
    return light ? 255.0f - v : v;
  };
  auto paper = [&](float x, float y) -> float {
    const int xx = std::max(0, std::min(W - 1, static_cast<int>(lrintf(x))));
    const int yy = std::max(0, std::min(H - 1, static_cast<int>(lrintf(y))));
    const float e = EnvelopeAt(env, ctx.sw, ctx.sh, xx, yy);
    return light ? 255.0f - e : e;
  };

  static uint8_t canvas[tiny_ocr::kInputHeight * tiny_ocr::kInputWidth];
  std::memset(canvas, 255, sizeof(canvas));
  uint32_t hist[256] = {};
  uint32_t total = 0;
  for (int r = 0; r < IH; ++r) {
    const bool inwin = r >= top && r < top + rh;
    for (int c = 0; c < rw; ++c) {
      float acc = 0;
      for (int sy = 0; sy < n; ++sy) {
        const float dv = wv1 + (r - top + (sy + 0.5f) / n) / sc;
        for (int sx = 0; sx < n; ++sx) {
          const float du = wu1 + (c + (sx + 0.5f) / n) / sc;
          float x = bx + du * ca - dv * sa;
          float y = by + du * sa + dv * ca;
          x = std::max(0.0f, std::min(x, W - 1.0f));
          y = std::max(0.0f, std::min(y, H - 1.0f));
          if (!inwin) {
            acc += paper(x, y);
            continue;
          }
          const int x0 = static_cast<int>(x), y0 = static_cast<int>(y);
          const int x1 = std::min(x0 + 1, W - 1), y1 = std::min(y0 + 1, H - 1);
          const float fx = x - x0, fy = y - y0;
          acc += (pixel(x0, y0) * (1 - fx) + pixel(x1, y0) * fx) * (1 - fy) +
                 (pixel(x0, y1) * (1 - fx) + pixel(x1, y1) * fx) * fy;
        }
      }
      const int v = std::max(0, std::min(255, static_cast<int>(lrintf(acc / (n * n)))));
      canvas[r * IW + kLineLeft + c] = static_cast<uint8_t>(v);
      if (inwin) {
        ++hist[v];
        ++total;
      }
    }
  }
  // 2nd / 98th percentile of the word window, then the stretch LUT
  int lo = 0, hi = 255;
  if (total > 0) {
    const uint32_t lo_rank = (total - 1) * 2 / 100, hi_rank = (total - 1) * 98 / 100;
    uint32_t seen = 0;
    bool have_lo = false;
    for (int v = 0; v < 256; ++v) {
      seen += hist[v];
      if (!have_lo && seen > lo_rank) {
        lo = v;
        have_lo = true;
      }
      if (seen > hi_rank) {
        hi = v;
        break;
      }
    }
  }
  const int span = hi - lo;
  int8_t lut[256];
  if (span < kMinContrastSpan) {
    std::memcpy(lut, tiny_ocr::kInputQuantLut, sizeof(lut));
  } else {
    const int range = kStretchCeiling - kStretchFloor;
    for (int v = 0; v < 256; ++v) {
      const int st = ((v - lo) * range) / span + kStretchFloor;
      lut[v] = tiny_ocr::kInputQuantLut[std::max(0, std::min(255, st))];
    }
  }
  for (int i = 0; i < IH * IW; ++i) input_->data.int8[i] = lut[canvas[i]];
  if (result != nullptr) {
    result->resized_width = static_cast<uint8_t>(std::min(255, rw));
    result->resized_height = static_cast<uint8_t>(std::min(255, rh));
    result->input_top = static_cast<uint8_t>(std::max(0, std::min(255, top)));
    result->scale_fallback = fit < 1.0f;
    result->crop_span = static_cast<uint8_t>(std::max(0, std::min(255, span)));
  }
  return true;
}

bool OcrRuntime::RunCrop(const GrayImage& image, const WordBox& box, int page_char_height,
                         uint8_t repeats, OcrResult* result, Stream& log,
                         const CropContext* ctx) {
  if (result == nullptr || !ready_ || repeats == 0) return false;
  std::memset(result, 0, sizeof(*result));
  const uint32_t preprocess_started = micros();
  const bool line_path = ctx != nullptr && box.geom && ctx->width == image.width &&
                         ctx->height == image.height;
  const bool prepared = line_path ? PrepareLineInput(image, box, *ctx, result)
                                  : PrepareInput(image, box, page_char_height, result);
  if (!prepared) {
    log.println("ERROR crop preprocessing failed");
    return false;
  }
  result->preprocess_us = micros() - preprocess_started;
  result->input_min = 127;
  result->input_max = -128;
  for (int i = 0; i < tiny_ocr::kInputHeight * tiny_ocr::kInputWidth; ++i) {
    result->input_min = std::min(result->input_min, input_->data.int8[i]);
    result->input_max = std::max(result->input_max, input_->data.int8[i]);
  }

  uint64_t invoke_sum = 0;
  for (uint8_t repeat = 0; repeat < repeats; ++repeat) {
    const uint32_t invoke_started = micros();
    if (interpreter_->Invoke() != kTfLiteOk) {
      log.printf("ERROR Invoke failed repeat=%u\n", repeat);
      return false;
    }
    const uint32_t elapsed = micros() - invoke_started;
    if (repeat == 0) result->first_invoke_us = elapsed;
    invoke_sum += elapsed;
  }
  result->average_invoke_us = static_cast<uint32_t>(invoke_sum / repeats);

  const uint32_t decode_started = micros();
  if (!tiny_ocr::GreedyCtcDecode(output_->data.int8, result->text,
                                 sizeof(result->text), &result->text_bytes)) {
    log.println("ERROR CTC decode failed");
    return false;
  }
  int32_t margin_sum = 0;
  result->output_min = 127;
  result->output_max = -128;
  result->blank_wins = 0;
  result->output_hash = 2166136261U;
  for (int step = 0; step < tiny_ocr::kTimeSteps; ++step) {
    const int8_t* row = output_->data.int8 + step * tiny_ocr::kClassCount;
    int8_t best = -128;
    int8_t second = -128;
    int best_class = 0;
    for (int cls = 0; cls < tiny_ocr::kClassCount; ++cls) {
      result->output_min = std::min(result->output_min, row[cls]);
      result->output_max = std::max(result->output_max, row[cls]);
      result->output_hash ^= static_cast<uint8_t>(row[cls]);
      result->output_hash *= 16777619U;
      if (row[cls] > best) {
        second = best;
        best = row[cls];
        best_class = cls;
      } else if (row[cls] > second) {
        second = row[cls];
      }
    }
    if (best_class == tiny_ocr::kBlankId) ++result->blank_wins;
    margin_sum += static_cast<int16_t>(best) - static_cast<int16_t>(second);
  }
  result->mean_argmax_margin = static_cast<int8_t>(
      std::min<int32_t>(127, margin_sum / tiny_ocr::kTimeSteps));
  result->decode_us = micros() - decode_started;
  result->ok = true;
  return true;
}

void OcrRuntime::PrintInfo(Stream& log) const {
  log.printf("MODEL_INFO ready=%u arena_capacity=%u arena_used=%u free_heap=%u free_psram=%u largest_psram=%u\n",
             ready_ ? 1U : 0U, static_cast<unsigned>(kTensorArenaBytes),
             static_cast<unsigned>(arena_used_bytes_), static_cast<unsigned>(ESP.getFreeHeap()),
             static_cast<unsigned>(ESP.getFreePsram()),
             static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM)));
}

}  // namespace ocr_demo
