#pragma once
#include "manual_response.h"

// Keep the confirmed port/starboard output pins; direction is configured below.
constexpr int LEFT_ESC_PIN = 9;
constexpr int RIGHT_ESC_PIN = 10;

// UNO R3 and individual RC inputs confirmed by the operator, 2026-10-29.
// Only individual PWM receiver channels are implemented, not SBUS/IBUS/PPM.
#ifndef KABOT_WIRING_VERIFIED
#define KABOT_WIRING_VERIFIED 1
#endif
#ifndef KABOT_RC_FIRST_PIN
#define KABOT_RC_FIRST_PIN 2
#endif
#ifndef KABOT_RC_SECOND_PIN
#define KABOT_RC_SECOND_PIN 3
#endif
#ifndef KABOT_RC_MODE_PIN
#define KABOT_RC_MODE_PIN 4
#endif
#ifndef KABOT_COMPILE_ONLY
#define KABOT_COMPILE_ONLY 0
#endif
constexpr int RC_FIRST_PIN = KABOT_RC_FIRST_PIN;    // Receiver CH2 -> D2, port ESC D9.
constexpr int RC_SECOND_PIN = KABOT_RC_SECOND_PIN;  // Receiver CH3 -> D3, starboard ESC D10.
constexpr int RC_MODE_PIN = KABOT_RC_MODE_PIN;      // Receiver CH5 -> D4 (not receiver CH6).
// Latest operator wiring correction: independent left/right PWM, no mixing.
constexpr bool RC_HIGH_IS_MANUAL = true;
// Measured resting inputs; both channels use the same center-relative curve.
constexpr uint16_t RC_LEFT_MIN_US = 1000;
constexpr uint16_t RC_LEFT_CENTER_US = 1496;
constexpr uint16_t RC_LEFT_MAX_US = 2000;
constexpr uint16_t RC_RIGHT_MIN_US = RC_LEFT_MIN_US;
constexpr uint16_t RC_RIGHT_CENTER_US = 1496;
constexpr uint16_t RC_RIGHT_MAX_US = RC_LEFT_MAX_US;
constexpr uint16_t RC_ENDPOINT_MARGIN_US = 0;
constexpr uint16_t RC_DEADBAND_US = 50;
constexpr uint16_t RC_COMMON_HALF_SPAN_US = kabot::commonHalfSpan(
    RC_LEFT_MIN_US, RC_LEFT_CENTER_US, RC_LEFT_MAX_US,
    RC_RIGHT_MIN_US, RC_RIGHT_CENTER_US, RC_RIGHT_MAX_US);
static_assert(RC_COMMON_HALF_SPAN_US > RC_ENDPOINT_MARGIN_US + RC_DEADBAND_US,
              "Common symmetric RC travel must exceed margin and deadband");
static_assert(RC_ENDPOINT_MARGIN_US + RC_DEADBAND_US < RC_LEFT_MAX_US - RC_LEFT_CENTER_US, "Invalid upper RC range");
static_assert(RC_ENDPOINT_MARGIN_US + RC_DEADBAND_US < RC_LEFT_CENTER_US - RC_LEFT_MIN_US, "Invalid lower RC range");
// RC polarity stays independent from the Jetson/AUTO forward convention.
constexpr bool LEFT_REVERSED = true;
constexpr bool RIGHT_REVERSED = false;
constexpr bool AUTO_LEFT_REVERSED = true;
constexpr bool AUTO_RIGHT_REVERSED = true;
constexpr int ESTOP_PIN = -1;       // External EMO cuts ESC supply; no EMO input on this UNO.
// Jetson's public command convention stays at 1500. ESC calibration is local.
constexpr uint16_t COMMAND_NEUTRAL_US = 1500;
// Receiver trim and public command center remain independent of ESC stop PWM.
// Operator-requested ESC stop PWM.
constexpr uint16_t NEUTRAL_US = 1480;
constexpr uint16_t ESC_MIN_US = 1100;
constexpr uint16_t ESC_MAX_US = 1900;
constexpr uint16_t AUTO_DELTA_LIMIT_US = 400;
constexpr uint16_t ESC_INITIALIZE_MS = 7000;
