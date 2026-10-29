#pragma once

// Confirmed by the operator, 2026-10-29: port D9, starboard D10.
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
constexpr int RC_FIRST_PIN = KABOT_RC_FIRST_PIN;    // Port/left receiver PWM -> D9.
constexpr int RC_SECOND_PIN = KABOT_RC_SECOND_PIN;  // Starboard/right receiver PWM -> D10.
constexpr int RC_MODE_PIN = KABOT_RC_MODE_PIN;
// Latest operator wiring correction: independent left/right PWM, no mixing.
constexpr bool RC_HIGH_IS_MANUAL = true;
// Operator correction: identical channels; right stick has no self-centering.
constexpr uint16_t RC_LEFT_MIN_US = 1000;
constexpr uint16_t RC_LEFT_CENTER_US = 1500;
constexpr uint16_t RC_LEFT_MAX_US = 2000;
constexpr uint16_t RC_RIGHT_MIN_US = RC_LEFT_MIN_US;
constexpr uint16_t RC_RIGHT_CENTER_US = RC_LEFT_CENTER_US;
constexpr uint16_t RC_RIGHT_MAX_US = RC_LEFT_MAX_US;
constexpr uint16_t RC_ENDPOINT_MARGIN_US = 100;
constexpr uint16_t RC_DEADBAND_US = 40;
static_assert(RC_ENDPOINT_MARGIN_US + RC_DEADBAND_US < RC_LEFT_MAX_US - RC_LEFT_CENTER_US, "Invalid upper RC range");
static_assert(RC_ENDPOINT_MARGIN_US + RC_DEADBAND_US < RC_LEFT_CENTER_US - RC_LEFT_MIN_US, "Invalid lower RC range");
constexpr bool LEFT_REVERSED = false;
constexpr bool RIGHT_REVERSED = false;
constexpr int ESTOP_PIN = -1;       // External EMO cuts ESC supply; no EMO input on this UNO.
constexpr uint16_t NEUTRAL_US = 1500;  // Provisional: must match actual ESC setup.
constexpr uint16_t AUTO_DELTA_LIMIT_US = 100;
