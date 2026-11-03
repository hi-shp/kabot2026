#pragma once
#include <Arduino.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "board_config.h"

// Basic ESC 500 accepts 50..500 Hz. Timer1 drives both pins directly at 400 Hz;
// RC/serial interrupts cannot move the physical PWM edges as with Servo ISRs.
constexpr uint16_t ESC_PWM_TOP = 4999;  // 16 MHz / 8 / (4999 + 1) = 400 Hz.
inline void beginEscPwm() {
  static_assert(F_CPU == 16000000UL, "ESC PWM requires a 16 MHz UNO");
  static_assert(LEFT_ESC_PIN == 9 && RIGHT_ESC_PIN == 10,
                "Timer1 OC1A/OC1B mapping requires D9 left and D10 right");
  static_assert(ESC_MAX_US * 2 < ESC_PWM_TOP, "Pulse must fit inside PWM period");
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    TCCR1A = 0;
    TCCR1B = 0;
    TIMSK1 = 0;
    PORTB &= uint8_t(~(_BV(PB1) | _BV(PB2)));
    TCNT1 = 0;
    ICR1 = ESC_PWM_TOP;
    OCR1A = NEUTRAL_US * 2;  // Timer tick is 0.5 us.
    OCR1B = NEUTRAL_US * 2;
    TIFR1 = _BV(ICF1) | _BV(OCF1B) | _BV(OCF1A) | _BV(TOV1);
    DDRB |= _BV(PB1) | _BV(PB2);
    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);  // Fast PWM mode 14.
  }
}

inline void writeEscPwm(uint16_t left, uint16_t right) {
  const uint16_t leftTicks = constrain(left, ESC_MIN_US, ESC_MAX_US) * 2;
  const uint16_t rightTicks = constrain(right, ESC_MIN_US, ESC_MAX_US) * 2;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    OCR1A = leftTicks;
    OCR1B = rightTicks;
  }
}
