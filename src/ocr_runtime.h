#pragma once

#include <Arduino.h>
#include <cstddef>
#include <cstdint>

#include "mcu_word_detector.h"
#include "ocr_model_config.h"
#include "tflm_ops.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

namespace ocr_demo {

// The arena is a reservation, not a measurement: TFLM reports `arena_used` of
// 776,092 bytes for this model, so the original 5 MB left ~4.4 MB of PSRAM
// reserved and never touched. That reservation, not the camera, was what pushed
// the measured PSRAM trough down to 837 KB free during the 1.44 MB RGB888 JPEG
// decode in DeviceCaptureAndDetect().
//
// Measured on this board, this build: `MODEL_READY arena_capacity=2097152
// arena_used=776092`. 2 MB left the PSRAM trough at 3.87 MB (46 % free) during a
// real camera capture, so this is 1.5 MB -- still 1.93x the measured usage, and
// it puts the trough above 50 % free with the 12k dictionary's 95 KB word index
// resident.
//
// If a larger model is ever loaded, raise this and re-check two lines: the
// `arena_used=` in MODEL_READY for what the model actually needs, and the
// `psram min=` in the M report after a `K` capture for what the whole pipeline
// has left. Neither is ever a guess.
constexpr size_t kTensorArenaBytes = 3U * 512U * 1024U;   // 1.5 MB

// --- crop geometry ---------------------------------------------------------
// The recognizer sees a 48 x 320 window. How a word box is placed inside that
// window is the single largest accuracy lever in this pipeline; the constants
// below were chosen by sweeping them against the real INT8 model on the
// desktop mirror (pc_pipeline) over 277 labelled words. See CROP_GEOMETRY.md.
//
// kTargetCharHeight: the page's median character height is mapped to this many
// input rows, for every box on the page. Scaling each box to fill the input
// height instead makes a word's glyph scale depend on whether it happens to
// contain an ascender or a descender, which is a ~1.7x swing the model never
// saw in training - it is why "can" used to come back as "Can".
constexpr int kTargetCharHeight = 32;
// The word's baseline is placed on this row, leaving room above for ascenders
// and below for descenders.
constexpr int kBaselineRow = 38;
// Blank columns kept on both sides of the resized crop. The first glyph used
// to start at column 0, giving the leading CTC timesteps no context, and left
// truncation errors outnumbered right truncation errors 14 to 5.
constexpr int kCropMarginX = 12;
// Paper kept around the ink, in INPUT pixels. The crop is taken from the box's
// ink extent and this budget, never from the detector's padded box: the
// detector pads by a percentage of the character height, which at page scale
// can exceed the 48-row window on a word carrying both an ascender and a
// descender, and would push exactly those words back onto the fallback path.
// Shrunk automatically, down to zero, when the window is tight.
constexpr int kInkPadInput = 3;
// Per-crop contrast normalisation: the crop's own 2nd/98th percentile is
// mapped onto this range before quantization, so a dim or low-contrast frame
// still fills the input range the model was trained on. Skipped when the crop
// spans less than kMinContrastSpan, which means it carries no ink to stretch.
constexpr int kStretchFloor = 8;
constexpr int kStretchCeiling = 247;
constexpr int kMinContrastSpan = 12;

// --- oriented crop (LineWordDetector boxes) ---------------------------------
// Measured with the real INT8 model on device-scale text (pc_pipeline/
// detection_theory, pc_pipeline/line_detector): the LINE's x-height on 20 of
// the 48 rows reads best (91.0 % vs 89.4 % for the page-median scale above,
// 96.6 % vs 86.2 % on x-height-only words), with ~0.25 x-height of paper above
// and below the ink and ~0.45 x-height before and after it -- which is what the
// training crops carry (0.12 em / 0.23 em). The baseline goes on row
// 48 - T/2 - 3.
constexpr int kLineTargetXh = 20;
constexpr int kLineLeft = 12;
constexpr float kLinePadU = 0.45f;
constexpr float kLinePadV = 0.25f;

struct OcrResult {
  bool ok;
  char text[192];
  size_t text_bytes;
  uint32_t preprocess_us;
  uint32_t first_invoke_us;
  uint32_t average_invoke_us;
  uint32_t decode_us;
  int8_t mean_argmax_margin;
  int8_t input_min;
  int8_t input_max;
  int8_t output_min;
  int8_t output_max;
  uint8_t blank_wins;
  uint32_t output_hash;
  // Crop geometry actually used, for the web preview and for debugging.
  uint8_t resized_width;
  uint8_t resized_height;
  uint8_t input_top;
  // The box did not fit at the page scale (an oversized word, or a heading in
  // a larger face than the page median) and was aspect-fitted instead.
  bool scale_fallback;
  // Contrast span of the source crop before normalisation, 0..255.
  uint8_t crop_span;
};

class OcrRuntime {
 public:
  OcrRuntime();
  bool Begin(Stream& log);
  // page_char_height is DetectionResult::median_char_height_x2 / 2. Pass 0 to
  // fall back to the legacy aspect-fit placement.
  // `ctx` is DetectionResult::crop. When it is set and the box carries line geometry
  // (LineWordDetector), the crop is resampled along the word's own baseline: see
  // PrepareLineInput. Otherwise the legacy axis-aligned path runs, unchanged.
  bool RunCrop(const GrayImage& image, const WordBox& box, int page_char_height,
               uint8_t repeats, OcrResult* result, Stream& log,
               const CropContext* ctx = nullptr);
  void PrintInfo(Stream& log) const;
  bool ready() const { return ready_; }
  size_t arena_used_bytes() const { return arena_used_bytes_; }

 private:
  bool PrepareInput(const GrayImage& image, const WordBox& box, int page_char_height,
                    OcrResult* result);
  bool PrepareLineInput(const GrayImage& image, const WordBox& box, const CropContext& ctx,
                        OcrResult* result);
  bool ValidateTensorContract(Stream& log) const;
  // A rectangle of the source frame, inclusive on both ends. PrepareInput
  // derives it from the box's ink extent rather than using the padded box.
  struct CropRect {
    int x1;
    int y1;
    int x2;
    int y2;
    int width() const { return x2 - x1 + 1; }
    int height() const { return y2 - y1 + 1; }
  };

  // Fills lut with kInputQuantLut composed with this crop's contrast stretch,
  // and returns the crop's measured 2nd-to-98th percentile span.
  uint8_t BuildCropLut(const GrayImage& image, const CropRect& crop, int8_t* lut) const;
  uint8_t SampleArea(const GrayImage& image, const CropRect& crop,
                     int dx, int dy, int resized_width, int resized_height) const;
  uint8_t SampleBilinear(const GrayImage& image, const CropRect& crop,
                         int dx, int dy, int resized_width, int resized_height) const;

  const tflite::Model* model_;
  tiny_ocr::OcrOpResolver resolver_;
  tflite::MicroInterpreter* interpreter_;
  TfLiteTensor* input_;
  TfLiteTensor* output_;
  void* arena_raw_;
  uint8_t* arena_;
  size_t arena_used_bytes_;
  bool ready_;
};

}  // namespace ocr_demo
