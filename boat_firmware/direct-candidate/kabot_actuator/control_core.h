#pragma once
#include <stdint.h>

namespace kabot {
enum Mode : uint8_t { INHIBITED = 0, MANUAL = 1, AUTOMATIC = 2 };
inline Mode modeFromPwm(uint16_t pulse, bool fresh, bool highIsManual = true) {
  if (!fresh || pulse < 900 || pulse > 2100 || (pulse >= 1300 && pulse <= 1700)) return INHIBITED;
  return ((pulse > 1700) == highIsManual) ? MANUAL : AUTOMATIC;
}

class Controller {
 public:
  explicit Controller(uint16_t neutral = 1500, uint16_t delta = 100)
      : left(neutral), right(neutral), neutral_(neutral), delta_(delta) {}
  Mode mode = INHIBITED;
  bool armed = false;
  uint16_t left, right;
  uint16_t last_sequence = 0;

  void disarm() {
    armed = false;
    has_command_ = false;
    if (mode != MANUAL) neutral();
  }
  void inputs(uint32_t now, bool valid, bool manual, uint16_t l, uint16_t r, bool estop) {
    Mode next = !valid || estop ? INHIBITED : manual ? MANUAL : AUTOMATIC;
    if (next != mode) { mode = next; disarm(); neutral(); }
    rc_neutral_ = distance(l, neutral_) <= 40 && distance(r, neutral_) <= 40;
    if (next == MANUAL) {
      disarm(); left = clamp(l); right = clamp(r);
    } else if (next == INHIBITED) {
      disarm(); neutral();
    } else if (!armed || (has_command_ && uint32_t(now - last_command_) > 400)) {
      disarm(); neutral();
    }
  }
  bool command(uint32_t now, uint16_t sequence, char operation, uint16_t l, uint16_t r) {
    // A disarm is always safe and also establishes the sequence for a new host.
    if (operation == 'D' && l == neutral_ && r == neutral_) {
      disarm(); last_sequence = sequence; has_sequence_ = true; return true;
    }
    // A fresh host MUST first disarm. Duplicate/reordered packets never refresh timeouts.
    const uint16_t gap = uint16_t(sequence - last_sequence);
    if (!has_sequence_ || gap == 0 || gap >= 32768 || mode != AUTOMATIC) return false;
    if (operation == 'A') {
      if (!rc_neutral_ || l != neutral_ || r != neutral_) return false;
      armed = true; has_command_ = false; neutral();
    } else if (operation == 'C') {
      // Check timeout here too: a late command cannot revive an expired controller.
      if (!armed || (has_command_ && uint32_t(now - last_command_) > 400) ||
          distance(l, neutral_) > delta_ || distance(r, neutral_) > delta_) {
        if (has_command_ && uint32_t(now - last_command_) > 400) disarm();
        return false;
      }
      left = l; right = r; last_command_ = now; has_command_ = true;
    } else return false;
    last_sequence = sequence; return true;
  }
 private:
  uint16_t neutral_, delta_;
  bool rc_neutral_ = false, has_sequence_ = false, has_command_ = false;
  uint32_t last_command_ = 0;
  static uint16_t distance(uint16_t a, uint16_t b) { return a > b ? a - b : b - a; }
  static uint16_t clamp(uint16_t p) { return p < 1100 ? 1100 : p > 1900 ? 1900 : p; }
  void neutral() { left = neutral_; right = neutral_; }
};
}  // namespace kabot
