#pragma once
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <avr/interrupt.h>
#include "board_config.h"
#include "status_led_policy.h"

constexpr uint8_t STATUS_LED_PIN = 5;
constexpr uint8_t STATUS_LED_COUNT = 12;
static_assert(STATUS_LED_PIN != LEFT_ESC_PIN && STATUS_LED_PIN != RIGHT_ESC_PIN &&
              STATUS_LED_PIN != RC_FIRST_PIN && STATUS_LED_PIN != RC_SECOND_PIN &&
              STATUS_LED_PIN != RC_MODE_PIN && STATUS_LED_PIN != ESTOP_PIN,
              "Status LED pin conflicts with existing wiring");
static_assert(F_CPU == 16000000UL, "LED scheduling requires the existing 16 MHz UNO timing");

class StatusLed {
 public:
  void begin() {
    digitalWrite(11, LOW);
    pinMode(11, OUTPUT);
    pinMode(12, INPUT_PULLUP);
    strip_.begin();
    strip_.setBrightness(24);
  }

  void update(StatusColor color, volatile bool &refreshing) {
    const uint32_t now = millis();
    if (color == StatusColor::Yellow && previousModeColor_ != StatusColor::Yellow) {
      automaticStartedAt_ = now;
    }
    previousModeColor_ = color;
    if (color == StatusColor::Yellow && uint32_t(now - automaticStartedAt_) < 1000UL) {
      color = StatusColor::Red;
    }
    if (digitalRead(12) == LOW) color = StatusColor::Red;
    if (color == shown_ || !strip_.canShow()) return;
    if (color != buffered_) {
      const uint32_t pixel = color == StatusColor::Green ? strip_.Color(0, 255, 0)
          : color == StatusColor::Yellow ? strip_.Color(255, 255, 0)
          : color == StatusColor::Red ? strip_.Color(255, 0, 0) : 0;
      strip_.fill(pixel);
      buffered_ = color;
    }
    // Fit the 12-pixel, ~360 us frame between Timer1 ESC pulse edges.
    const uint8_t savedInterrupts = SREG;
    cli();
    const uint16_t phase = TCNT1;
    if (phase < 3880 || phase > 4020) {
      SREG = savedInterrupts;
      return;
    }
    refreshing = true;
    strip_.show();
    refreshing = false;
    SREG = savedInterrupts;
    shown_ = color;
  }

 private:
  StatusColor previousModeColor_ = StatusColor::Unknown;
  uint32_t automaticStartedAt_ = 0;
  Adafruit_NeoPixel strip_{STATUS_LED_COUNT, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800};
  StatusColor shown_ = StatusColor::Unknown;
  StatusColor buffered_ = StatusColor::Unknown;
};
