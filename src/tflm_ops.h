#pragma once
#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"

namespace tiny_ocr {

constexpr int kResolverOpCount = 9;
using OcrOpResolver = tflite::MicroMutableOpResolver<kResolverOpCount>;

inline TfLiteStatus RegisterOcrOps(OcrOpResolver& resolver) {
  if (resolver.AddAdd() != kTfLiteOk) return kTfLiteError;
  if (resolver.AddConv2D() != kTfLiteOk) return kTfLiteError;
  if (resolver.AddDepthwiseConv2D() != kTfLiteOk) return kTfLiteError;
  if (resolver.AddLogistic() != kTfLiteOk) return kTfLiteError;
  if (resolver.AddMul() != kTfLiteOk) return kTfLiteError;
  if (resolver.AddPad() != kTfLiteOk) return kTfLiteError;
  if (resolver.AddReshape() != kTfLiteOk) return kTfLiteError;
  if (resolver.AddSum() != kTfLiteOk) return kTfLiteError;
  if (resolver.AddTranspose() != kTfLiteOk) return kTfLiteError;
  return kTfLiteOk;
}

}  // namespace tiny_ocr
