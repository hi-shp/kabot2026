#pragma once
#include <stdint.h>

namespace kabot {
enum Mode : uint8_t { INHIBITED = 0, MANUAL = 1, AUTOMATIC = 2 };
inline Mode modeFromPwm(uint16_t pulse, bool fresh, bool highIsManual = true) {
  if (!fresh || pulse < 900 || pulse > 2100 || (pulse >= 1300 && pulse <= 1700)) return INHIBITED;
  return ((pulse > 1700) == highIsManual) ? MANUAL : AUTOMATIC;
}

inline Mode selectMode(uint16_t pulse, bool modeFresh, bool channelsFresh, bool hardwareReady) {
  if (!hardwareReady) return INHIBITED;
  // Only a complete, fresh and explicitly selected RC frame overrides Jetson.
  // No receiver, missing channels or a centered failsafe mode channel -> AUTO.
  return modeFresh && channelsFresh && modeFromPwm(pulse, true) == MANUAL
      ? MANUAL : AUTOMATIC;
}

class Controller {
 public:
  explicit Controller(uint16_t neutral = 1500, uint16_t delta = 400)
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
    if (next != mode) {
      mode = next; disarm(); neutral();
    }
    if (next == MANUAL) {
      disarm();
      if (manual_resume_required_) {
        neutral();
        if (l == neutral_ && r == neutral_) {
          if (!neutral_seen_) { neutral_seen_ = true; neutral_since_ = now; }
          if (uint32_t(now - neutral_since_) >= 500) manual_resume_required_ = false;
        } else neutral_seen_ = false;
      } else { left = clamp(l); right = clamp(r); }
    } else if (next == INHIBITED) {
      neutral_seen_ = false;
      disarm(); neutral();
    } else {
      neutral_seen_ = false;
      if (!armed ||
          (has_command_ && uint32_t(now - last_command_) > 400)) {
        disarm(); neutral();
      }
    }
  }
  bool command(uint32_t now, uint16_t sequence, char operation, uint16_t l, uint16_t r) {
    if (operation == 'I' && l == neutral_ && r == neutral_) {
      // Explicit bringup initialization: no thrust, accepted without RC history.
      disarm(); neutral();
      last_sequence = sequence; has_sequence_ = true; return true;
    }
    if (operation == 'S' && l == neutral_ && r == neutral_) {
      // Exit/stop cannot be overwritten on the next loop by a held RC stick.
      // No persistent latch/release command: center both sticks for 0.5s to resume.
      manual_resume_required_ = true; neutral_seen_ = false;
      disarm(); neutral();
      last_sequence = sequence; has_sequence_ = true; return true;
    }
    // A disarm is always safe and also establishes the sequence for a new host.
    if (operation == 'D' && l == neutral_ && r == neutral_) {
      disarm(); last_sequence = sequence; has_sequence_ = true; return true;
    }
    // A fresh host MUST first disarm. Duplicate/reordered packets never refresh timeouts.
    const uint16_t gap = uint16_t(sequence - last_sequence);
    if (!has_sequence_ || gap == 0 || gap >= 32768) return false;
    if (operation == 'R' && l == neutral_ && r == neutral_) {
      disarm(); neutral(); last_sequence = sequence; return true;
    }
    // Mode changes discard the old arm; accept a fresh AUTO arm immediately.
    if (mode != AUTOMATIC) return false;
    if (operation == 'A') {
      if (l != neutral_ || r != neutral_) return false;
      manual_resume_required_ = false; neutral_seen_ = false;
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
  bool has_sequence_ = false, has_command_ = false;
  bool manual_resume_required_ = false, neutral_seen_ = false;
  uint32_t neutral_since_ = 0;
  uint32_t last_command_ = 0;
  static uint16_t distance(uint16_t a, uint16_t b) { return a > b ? a - b : b - a; }
  static uint16_t clamp(uint16_t p) { return p < 1100 ? 1100 : p > 1900 ? 1900 : p; }
  void neutral() { left = neutral_; right = neutral_; }
};
}  // namespace kabot
