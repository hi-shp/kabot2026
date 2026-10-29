#pragma once
#include <stdint.h>

namespace kabot {
inline uint16_t centerReferenced(uint16_t pulse, uint16_t center) {
  const int value = 1500 + int(pulse) - int(center);
  return value < 1100 ? 1100 : value > 1900 ? 1900 : uint16_t(value);
}
inline uint16_t gentleRc(uint16_t pulse, uint16_t center) {
  int delta = int(pulse) - int(center);
  if (delta >= -40 && delta <= 40) return 1500;
  if (delta < -400) delta = -400;
  if (delta > 400) delta = 400;
  return uint16_t(1500 + delta / 5);
}
struct ManualOutput { uint16_t left, right; };

// No speed integration. Each stick has its own independent endpoint confirmation.
// A broad endpoint region tolerates receiver travel error; confirmation filters spikes.
class ManualResponse {
 public:
  ManualResponse(uint16_t minimum = 1000, uint16_t center = 1500,
                 uint16_t maximum = 2000, uint16_t margin = 100,
                 uint16_t confirmMs = 150)
      : minimum_(minimum), center_(center), maximum_(maximum), margin_(margin), confirmMs_(confirmMs) {}
  ManualOutput update(uint32_t now, uint16_t left, uint16_t right, bool enabled) {
    if (!enabled) { candidate_[0] = candidate_[1] = 0; return {1500, 1500}; }
    return {side(now, left, 0), side(now, right, 1)};
  }
 private:
  uint16_t minimum_, center_, maximum_, margin_, confirmMs_;
  uint32_t since_[2] = {};
  int8_t candidate_[2] = {};
  uint16_t side(uint32_t now, uint16_t pulse, uint8_t index) {
    const int8_t direction = pulse >= maximum_ - margin_ ? 1 : pulse <= minimum_ + margin_ ? -1 : 0;
    if (direction == 0) { candidate_[index] = 0; return gentleRc(pulse, center_); }
    if (candidate_[index] != direction) { candidate_[index] = direction; since_[index] = now; }
    if (uint32_t(now - since_[index]) >= confirmMs_) return direction > 0 ? 1900 : 1100;
    return gentleRc(pulse, center_);
  }
};
}  // namespace kabot
