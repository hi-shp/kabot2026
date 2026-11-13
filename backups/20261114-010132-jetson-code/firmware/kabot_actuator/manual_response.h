#pragma once
#include <stdint.h>

namespace kabot {
inline uint16_t centerReferenced(uint16_t pulse, uint16_t center) {
  const int value = 1500 + int(pulse) - int(center);
  return value < 1100 ? 1100 : value > 1900 ? 1900 : uint16_t(value);
}
struct ManualOutput { uint16_t left, right; };
constexpr uint16_t smaller(uint16_t a, uint16_t b) { return a < b ? a : b; }
// Intersection after subtracting each input center, made symmetric about zero.
constexpr uint16_t commonHalfSpan(uint16_t lmin, uint16_t lc, uint16_t lmax,
                                  uint16_t rmin, uint16_t rc, uint16_t rmax) {
  return smaller(smaller(lc-lmin, lmax-lc), smaller(rc-rmin, rmax-rc));
}

// Signed quadratic response after the neutral deadband, independently per channel.
// Continuous at the endpoint: no separate full-power switch and no speed integration.
class ManualResponse {
 public:
  ManualResponse(uint16_t minimum = 1000, uint16_t center = 1500,
                 uint16_t maximum = 2000, uint16_t margin = 100,
                 uint16_t deadband = 40, uint16_t rightCenter = 0, uint16_t commonSpan = 0)
      : minimum_(minimum), center_(center), rightCenter_(rightCenter ? rightCenter : center),
        maximum_(maximum), margin_(margin), deadband_(deadband), commonSpan_(commonSpan) {}
  ManualOutput update(uint16_t left, uint16_t right, bool enabled) const {
    if (!enabled) return {1500, 1500};
    return {side(left, center_), side(right, rightCenter_)};
  }
 private:
  uint16_t minimum_, center_, rightCenter_, maximum_, margin_, deadband_, commonSpan_;
  uint16_t side(uint16_t pulse, uint16_t center) const {
    const int32_t delta = int32_t(pulse) - center;
    const uint32_t magnitude = delta < 0 ? uint32_t(-delta) : uint32_t(delta);
    if (magnitude <= deadband_) return 1500;
    const int32_t span = commonSpan_ ? commonSpan_ :
      (delta > 0 ? int32_t(maximum_) - center : int32_t(center) - minimum_);
    const int32_t activeSpan = span - margin_ - deadband_;
    if (activeSpan <= 0 || activeSpan > 500) return 1500;
    const uint32_t denominator = uint32_t(activeSpan);
    uint32_t input = magnitude - deadband_;
    if (input > denominator) input = denominator;
    const uint32_t square = denominator * denominator;
    // Start at zero after deadband, without a minimum-output offset.
    // Round the quadratic to nearest us; u=1 still gives exactly 400us.
    const uint16_t output = uint16_t((400UL * input * input + square / 2) / square);
    return uint16_t(delta > 0 ? 1500 + output : 1500 - output);
  }
};
}  // namespace kabot
