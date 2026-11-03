#pragma once
#include <stdint.h>
#include "board_config.h"

namespace kabot {
// Translate the logical 1100..1500..1900 command to the calibrated ESC range.
// Reverse the signed command before scaling, preserving both full endpoints.
inline uint16_t escOutput(uint16_t command, bool reversed) {
  int32_t delta = int32_t(command) - COMMAND_NEUTRAL_US;
  if (delta < -400) delta = -400;
  if (delta > 400) delta = 400;
  if (reversed) delta = -delta;
  const uint32_t span = delta >= 0 ? ESC_MAX_US - NEUTRAL_US : NEUTRAL_US - ESC_MIN_US;
  const uint32_t magnitude = delta >= 0 ? uint32_t(delta) : uint32_t(-delta);
  const uint16_t offset = uint16_t((magnitude * span + 200) / 400);
  return delta >= 0 ? NEUTRAL_US + offset : NEUTRAL_US - offset;
}
}  // namespace kabot
