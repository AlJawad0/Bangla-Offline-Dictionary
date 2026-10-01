#pragma once

// Line-first word detector.
//
// A word is a segment of a TEXT LINE, not a blob. Lines are found first -- each with its
// own angle, baseline and x-height -- and words are cut along the line in units of that
// line's x-height. That is what makes it work for any size (mixed sizes in one frame),
// any tilt up to +/-40 degrees, dark-on-light or light-on-dark, and fonts whose letters
// blur together.
//
// The reference implementation, and the measurements behind every constant, are in
// pc_pipeline/line_detector/ (line_detector.py). Keep the two in step.
//
// Output is the same DetectionResult McuWordDetector produces, plus per-box line
// geometry (WordBox::geom) and a CropContext for the oriented OCR crop.

#include <Arduino.h>

#include "mcu_word_detector.h"

namespace ocr_demo {

class LineWordDetector {
 public:
  LineWordDetector() = default;
  bool Detect(const GrayImage& image, DetectionResult* result, Stream* log = nullptr);
  const CropContext& context() const { return ctx_; }

 private:
  bool EnsureBuffers(int width, int height);

  // Persistent: kept after Detect() for the crop and the panel.
  uint16_t* labels_ = nullptr;
  uint16_t* owner_ = nullptr;
  uint8_t* close_s_ = nullptr;
  uint8_t* open_s_ = nullptr;
  size_t label_capacity_ = 0;
  size_t small_capacity_ = 0;
  CropContext ctx_{};
};

}  // namespace ocr_demo
