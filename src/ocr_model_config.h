#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace tiny_ocr {

constexpr int kInputHeight = 48;
constexpr int kInputWidth = 320;
constexpr int kInputChannels = 1;
constexpr int kTimeSteps = 80;
constexpr int kClassCount = 85;
constexpr int kBlankId = 0;
constexpr float kInputScale = 0.0069443029351532459f;
constexpr int kInputZeroPoint = -17;
constexpr float kOutputScale = 0.1338304728269577f;
constexpr int kOutputZeroPoint = -25;

// Maps an 8-bit grayscale pixel through: pixel / 127.5 - 1.0, then
// round(real / input_scale + input_zero_point), clipped to INT8.
constexpr int8_t kInputQuantLut[256] = {
  -128, -128, -128, -128, -128, -128, -128, -128, -128, -128, -128, -128, -128, -128, -128, -128,
  -128, -128, -128, -128, -128, -128, -128, -128, -128, -128, -128, -128, -128, -128, -127, -126,
  -125, -124, -123, -121, -120, -119, -118, -117, -116, -115, -114, -112, -111, -110, -109, -108,
  -107, -106, -105, -103, -102, -101, -100, -99, -98, -97, -95, -94, -93, -92, -91, -90,
  -89, -88, -86, -85, -84, -83, -82, -81, -80, -79, -77, -76, -75, -74, -73, -72,
  -71, -70, -68, -67, -66, -65, -64, -63, -62, -60, -59, -58, -57, -56, -55, -54,
  -53, -51, -50, -49, -48, -47, -46, -45, -44, -42, -41, -40, -39, -38, -37, -36,
  -35, -33, -32, -31, -30, -29, -28, -27, -25, -24, -23, -22, -21, -20, -19, -18,
  -16, -15, -14, -13, -12, -11, -10, -9, -7, -6, -5, -4, -3, -2, -1, 1,
  2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 14, 15, 16, 17, 19,
  20, 21, 22, 23, 24, 25, 26, 28, 29, 30, 31, 32, 33, 34, 36, 37,
  38, 39, 40, 41, 42, 43, 45, 46, 47, 48, 49, 50, 51, 52, 54, 55,
  56, 57, 58, 59, 60, 61, 63, 64, 65, 66, 67, 68, 69, 71, 72, 73,
  74, 75, 76, 77, 78, 80, 81, 82, 83, 84, 85, 86, 87, 89, 90, 91,
  92, 93, 94, 95, 97, 98, 99, 100, 101, 102, 103, 104, 106, 107, 108, 109,
  110, 111, 112, 113, 115, 116, 117, 118, 119, 120, 121, 122, 124, 125, 126, 127
};

// IDs 1..84. ID 0 is the CTC blank. Strings are explicit UTF-8 bytes.
constexpr const char* kTokens[kClassCount - 1] = {
  "\x61",
  "\x62",
  "\x63",
  "\x64",
  "\x65",
  "\x66",
  "\x67",
  "\x68",
  "\x69",
  "\x6a",
  "\x6b",
  "\x6c",
  "\x6d",
  "\x6e",
  "\x6f",
  "\x70",
  "\x71",
  "\x72",
  "\x73",
  "\x74",
  "\x75",
  "\x76",
  "\x77",
  "\x78",
  "\x79",
  "\x7a",
  "\x41",
  "\x42",
  "\x43",
  "\x44",
  "\x45",
  "\x46",
  "\x47",
  "\x48",
  "\x49",
  "\x4a",
  "\x4b",
  "\x4c",
  "\x4d",
  "\x4e",
  "\x4f",
  "\x50",
  "\x51",
  "\x52",
  "\x53",
  "\x54",
  "\x55",
  "\x56",
  "\x57",
  "\x58",
  "\x59",
  "\x5a",
  "\x20",
  "\xc2\xb0",
  "\xc2\xb1",
  "\xc3\x97",
  "\xc3\xb7",
  "\xc2\xb5",
  "\xce\xa9",
  "\xce\x94",
  "\xce\xb8",
  "\xce\xb7",
  "\xcf\x89",
  "\xcf\x86",
  "\xce\xbb",
  "\xcf\x80",
  "\xe2\x89\xa4",
  "\xe2\x89\xa5",
  "\xe2\x89\x88",
  "\xe2\x89\xa0",
  "\xce\xb1",
  "\xce\xb2",
  "\xce\xb3",
  "\xce\xb4",
  "\xce\xa3",
  "\xe2\x88\x9a",
  "\xe2\x88\x9e",
  "\xc2\xb2",
  "\xc2\xb3",
  "\xc2\xb9",
  "\xe2\x82\x80",
  "\xe2\x82\x81",
  "\xe2\x82\x82",
  "\xe2\x82\x83",
};

// Expects row-major output with shape [80, 1, 85]. Positive affine output
// scaling preserves argmax, so greedy CTC decoding can use INT8 logits directly.
inline bool GreedyCtcDecode(const int8_t* logits, char* output,
                            size_t capacity, size_t* output_length) {
  if (logits == nullptr || output == nullptr || output_length == nullptr || capacity == 0) {
    return false;
  }
  size_t written = 0;
  int previous = kBlankId;
  for (int step = 0; step < kTimeSteps; ++step) {
    const int8_t* row = logits + step * kClassCount;
    int best = 0;
    for (int cls = 1; cls < kClassCount; ++cls) {
      if (row[cls] > row[best]) best = cls;
    }
    if (best != kBlankId && best != previous) {
      const char* token = kTokens[best - 1];
      const size_t length = std::strlen(token);
      if (written + length + 1 > capacity) return false;
      std::memcpy(output + written, token, length);
      written += length;
    }
    previous = best;
  }
  output[written] = '\0';
  *output_length = written;
  return true;
}

}  // namespace tiny_ocr
