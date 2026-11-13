#pragma once
#include "control_core.h"

enum class StatusColor : uint8_t { Unknown, Off, Green, Yellow };

constexpr StatusColor statusColor(kabot::Mode mode, bool rcValid, bool automaticActive,
                                  bool inhibited) {
  return inhibited ? StatusColor::Off
      : mode == kabot::MANUAL && rcValid ? StatusColor::Yellow
      : mode == kabot::AUTOMATIC && automaticActive ? StatusColor::Green
      : StatusColor::Off;
}
