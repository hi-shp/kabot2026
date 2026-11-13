#pragma once
#include "control_core.h"
enum class StatusColor : uint8_t { Unknown, Off, Green, Yellow, Red };
constexpr StatusColor statusColor(kabot::Mode mode, bool rcValid, bool, bool) {
  return mode == kabot::MANUAL && rcValid ? StatusColor::Green : StatusColor::Yellow;
}
