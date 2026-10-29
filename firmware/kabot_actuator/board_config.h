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
#define KABOT_RC_FIRST_PIN 3
#endif
#ifndef KABOT_RC_SECOND_PIN
#define KABOT_RC_SECOND_PIN 2
#endif
#ifndef KABOT_RC_MODE_PIN
#define KABOT_RC_MODE_PIN 4
#endif
#ifndef KABOT_COMPILE_ONLY
#define KABOT_COMPILE_ONLY 0
#endif
constexpr int RC_FIRST_PIN = KABOT_RC_FIRST_PIN;    // Left throttle OR throttle with RC_MIX.
constexpr int RC_SECOND_PIN = KABOT_RC_SECOND_PIN;  // Right throttle OR steering.
constexpr int RC_MODE_PIN = KABOT_RC_MODE_PIN;
// Recovered from the original flash: ch D3 centered 1485, D2 centered 1490,
// 39us deadband; left=1500+(1485-D3)+(D2-1490), right=1500+(1485-D3)-(D2-1490).
constexpr bool RC_MIX = true;
constexpr uint16_t RC_FIRST_CENTER_US = 1485;
constexpr uint16_t RC_SECOND_CENTER_US = 1490;
constexpr bool RC_MODE_IS_PWM = true;
constexpr bool RC_HIGH_IS_MANUAL = true;
constexpr bool LEFT_REVERSED = false;
constexpr bool RIGHT_REVERSED = false;
constexpr int ESTOP_PIN = -1;       // Optional NC-to-GND circuit; HIGH/open means stop.
constexpr uint16_t NEUTRAL_US = 1500;  // Provisional: must match actual ESC setup.
constexpr uint16_t AUTO_DELTA_LIMIT_US = 100;
