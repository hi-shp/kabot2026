#pragma once
#include <stdint.h>

namespace kabot {
inline uint16_t centerReferenced(uint16_t pulse, uint16_t center) {
  const int value = 1500 + int(pulse) - int(center);
  return value < 1100 ? 1100 : value > 1900 ? 1900 : uint16_t(value);
}
struct ManualOutput { uint16_t left, right; };

// Signed quadratic response after the neutral deadband, independently per channel.
// Continuous at the endpoint: no separate full-power switch and no speed integration.
class ManualResponse {
 public:
  ManualResponse(uint16_t minimum = 1000, uint16_t center = 1500,
                 uint16_t maximum = 2000, uint16_t margin = 100,
                 uint16_t deadband = 40)
      : minimum_(minimum), center_(center), maximum_(maximum), margin_(margin), deadband_(deadband) {}
  ManualOutput update(uint16_t left, uint16_t right, bool enabled) const {
    if (!enabled) return {1500, 1500};
    return {side(left), side(right)};
  }
 private:
  uint16_t minimum_, center_, maximum_, margin_, deadband_;
  uint16_t side(uint16_t pulse) const {
    const int32_t delta = int32_t(pulse) - center_;
    const uint32_t magnitude = delta < 0 ? uint32_t(-delta) : uint32_t(delta);
    if (magnitude <= deadband_) return 1500;
    const int32_t span = delta > 0 ? int32_t(maximum_) - center_ : int32_t(center_) - minimum_;
    const int32_t activeSpan = span - margin_ - deadband_;
    if (activeSpan <= 0 || activeSpan > 500) return 1500;
    const uint32_t denominator = uint32_t(activeSpan);
    uint32_t input = magnitude - deadband_;
    if (input > denominator) input = denominator;
    const uint32_t square = denominator * denominator;
    // Round to nearest us. At configured 360us span, numerator fits in uint32_t.
    const uint16_t output = uint16_t((400UL * input * input + square / 2) / square);
    return uint16_t(delta > 0 ? 1500 + output : 1500 - output);
  }
};
}  // namespace kabot
