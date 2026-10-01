#pragma once

#include <Arduino.h>
#include <cstddef>
#include <cstdint>

namespace ocr_demo {

constexpr uint16_t kDetectorMaxWidth = 480;
constexpr uint16_t kDetectorMaxHeight = 640;
constexpr size_t kMaxWordBoxes = 256;
constexpr size_t kMaxCharBoxes = 4096;

struct GrayImage {
  const uint8_t* pixels;
  uint16_t width;
  uint16_t height;
};

// Truncation / provenance bits for WordBox::flags.
enum WordBoxFlag : uint8_t {
  // The ink itself reaches the frame border, so the glyph run is cut off.
  // A word carrying any of these is NOT safe for an exact dictionary lookup.
  kWordFlagTruncLeft = 1u << 0,
  kWordFlagTruncRight = 1u << 1,
  kWordFlagTruncTop = 1u << 2,
  kWordFlagTruncBottom = 1u << 3,
  // The requested padding was clipped by the frame, but the ink is complete.
  // This box is still fully valid for recognition and lookup.
  kWordFlagPadClipped = 1u << 4,
  // This box came out of the gap-based split of a larger merged blob.
  kWordFlagSplit = 1u << 5,
  // No character component could be matched inside the blob; the box is the
  // raw blurred-blob extent rather than a refined ink extent.
  kWordFlagUnrefined = 1u << 6,
  // The box holds punctuation or a speck, not a word: at most two components
  // and an ink height well below x-height. Recognizing it wastes ~1.7 s and
  // returns noise, so the caller should skip it.
  kWordFlagPunctuation = 1u << 7,
};

constexpr uint8_t kWordFlagTruncAny =
    kWordFlagTruncLeft | kWordFlagTruncRight | kWordFlagTruncTop | kWordFlagTruncBottom;

struct WordBox {
  // Padded crop box. This is what the OCR crop should use.
  uint16_t x1;
  uint16_t y1;
  uint16_t x2;
  uint16_t y2;
  // Tight ink extent before padding. Truncation is judged on this box, and it
  // is the right box to draw when debugging segmentation.
  uint16_t ink_x1;
  uint16_t ink_y1;
  uint16_t ink_x2;
  uint16_t ink_y2;
  // Baseline row of this word: the median bottom edge of its character
  // components. Descenders are a minority of the glyphs in a word, so the
  // median bottom is the baseline and not the descender line. The OCR crop
  // uses it to place every word of a page on the same input row instead of
  // stretching each box to fill the input height.
  uint16_t baseline_y;
  uint32_t area;         // ink-mask pixel count of the merged blob
  uint16_t char_count;   // character components matched inside this word
  uint8_t flags;         // WordBoxFlag bits
  bool touches_edge;     // == (flags & kWordFlagTruncAny) != 0

  // ---- line geometry, filled by LineWordDetector only (geom == 0 otherwise) --------
  // The word lives on a text line with its own baseline, angle and x-height. The OCR
  // crop resamples along that baseline; the panel draws the oriented quad.
  uint8_t geom;          // 1 when the fields below are valid
  uint8_t polarity;      // 1 dark ink on light, 2 light ink on dark
  uint16_t line_id;
  uint16_t word_index;    // this box's own index, for the crop's ownership test
  int16_t xh_x16;        // the LINE's x-height, 1/16 px
  int32_t base_x_x16;    // baseline point at the word centre, frame px * 16
  int32_t base_y_x16;
  int16_t dir_x_q14;     // unit vector along the line at the word centre, Q14
  int16_t dir_y_q14;
  int16_t u1_x16;        // ink extent along the line, relative to the centre point
  int16_t u2_x16;
  int16_t top_x16;       // ink extent across the line, relative to the baseline
  int16_t bot_x16;       //   (negative = above the baseline)
  int16_t quad[8];       // display quad TL,TR,BR,BL as x,y frame px

  // True when this box is worth sending to the recognizer: the ink is complete
  // and it is a word rather than punctuation. This is exactly the "green box"
  // condition drawn by the web preview.
  bool recognizable() const {
    return (flags & (kWordFlagTruncAny | kWordFlagPunctuation)) == 0;
  }
};

struct DetectorConfig {
  // Crop padding as a percentage of the median character height, on top of a
  // 2 px floor. The 2 px floor is always justified: the ink threshold drops the
  // faint anti-aliased ramp at a stroke edge, so the measured ink extent is
  // always slightly inside the true glyph. The percentage on top of it must
  // match the margin the recognizer was trained with - set it near zero for a
  // model trained on glyph-tight crops, and to ~15 for one trained with the
  // usual crop jitter.
  uint8_t pad_x_pct = 12;
  uint8_t pad_y_pct = 15;
  // Word-merge blur sigmas as a percentage of the median character height.
  uint8_t merge_sigma_x_pct = 30;
  uint8_t merge_sigma_y_pct = 12;
  // Word mask threshold as a percentage of the local (per text line) peak.
  uint8_t word_threshold_pct = 35;
  // Split a merged blob when an internal gap exceeds this percentage of the
  // median character height AND exceeds split_gap_ratio_x10/10 of the blob's
  // own median gap.
  uint8_t split_gap_pct = 45;
  uint8_t split_gap_ratio_x10 = 18;
  // Upper bound on the frame-wide Otsu gap threshold, as a multiple (x10) of
  // split_gap_pct * char_height. Otsu adapts the split point to this frame's
  // letter spacing, but on a dense page it can settle above the real word gap
  // and then no blob is ever cut. No printed word gap exceeds ~0.9 * x-height,
  // so capping it costs nothing and removes that failure mode.
  uint8_t split_gap_otsu_cap_x10 = 20;
  // A box with at most punct_max_components components whose ink is shorter
  // than this percentage of the median character height is punctuation, not a
  // word. Every letter reaches at least x-height, which is ~0.8 of the median
  // component height, while a comma reaches ~0.3 - the two do not overlap.
  uint8_t punct_max_height_pct = 55;
  uint8_t punct_max_components = 2;
  // Fixed adaptive-threshold offset used when adaptive_threshold_c is false.
  uint8_t fixed_threshold_c = 12;
  // Ink within this many pixels of the frame counts as truncated.
  uint8_t border_margin = 1;
  // Background estimation decimation factor (1, 2 or 4).
  uint8_t bg_decimate = 4;

  bool adaptive_threshold_c = true;
  bool adaptive_bg_radius = true;
  bool enable_skew = true;
  bool enable_gap_split = true;
  bool drop_rules = true;      // erase page rules / borders / figure edges
  bool drop_truncated = false; // if true, truncated boxes are not returned
};

// What the oriented OCR crop needs besides the frame, owned by LineWordDetector and
// valid until its next Detect(): which word owns each ink pixel (so a crop can paint
// its neighbours' ink as paper), and the paper/ink envelopes (so it knows what paper is).
struct CropContext {
  const uint16_t* labels;     // width*height component labels, 0 = no ink
  const uint16_t* owner;      // per label: word index, or 0xFFFF for none/shared
  const uint8_t* close_s;     // paper envelope under dark ink, decimated by 4
  const uint8_t* open_s;      // paper envelope under light ink, decimated by 4
  uint16_t width, height, sw, sh;
  uint16_t n_labels;
};

struct DetectionResult {
  WordBox boxes[kMaxWordBoxes];
  const CropContext* crop;      // non-null only for LineWordDetector results
  size_t count;
  uint16_t median_char_height_x2;
  uint16_t char_count;          // accepted character components
  uint16_t truncated_count;     // boxes carrying any truncation bit
  uint16_t punctuation_count;   // boxes classified as punctuation / speck
  uint16_t recognizable_count;  // boxes that are neither truncated nor punctuation
  uint16_t split_count;         // boxes produced by the gap split
  uint16_t rules_removed;       // oversized components erased before merging
  uint8_t threshold_c;          // adaptive-threshold offset actually used
  uint8_t bg_radius;            // background blur radius actually used (full-res px)
  int16_t skew_slope_q12;       // dy/dx of the text baseline, Q12
  uint16_t contrast;            // paper peak minus 1st-percentile ink
  bool component_stack_overflow;
  bool char_box_overflow;
  bool box_capacity_reached;
  uint32_t background_us;
  uint32_t threshold_us;
  uint32_t character_pass_us;
  uint32_t word_merge_us;
  uint32_t word_components_us;
  uint32_t refine_us;
  uint32_t total_us;
  char error[128];
};

class McuWordDetector {
 public:
  McuWordDetector() = default;
  explicit McuWordDetector(const DetectorConfig& config) : config_(config) {}

  bool Detect(const GrayImage& image, DetectionResult* result, Stream* log = nullptr);

  const DetectorConfig& config() const { return config_; }
  void set_config(const DetectorConfig& config) { config_ = config; }

 private:
  DetectorConfig config_;
};

}  // namespace ocr_demo
