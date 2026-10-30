#include <Arduino.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <avr/wdt.h>
#include "board_config.h"
#include "control_core.h"
#include "manual_response.h"
#include "protocol.h"
#include "esc_output.h"
#include "esc_pwm.h"
#include "esc_startup.h"

// This firmware is an actuator/RC arbiter. Navigation and MPC run on Jetson.
kabot::Controller controller(COMMAND_NEUTRAL_US, AUTO_DELTA_LIMIT_US);
kabot::EscStartup powerOnStartup(ESC_INITIALIZE_MS);
kabot::EscStartup modeSwitchNeutral(20);  // Brief physical neutral; no command delay.
kabot::ManualResponse manualRc(RC_LEFT_MIN_US, RC_LEFT_CENTER_US, RC_LEFT_MAX_US,
                              RC_ENDPOINT_MARGIN_US, RC_DEADBAND_US,
                              RC_RIGHT_CENTER_US, RC_COMMON_HALF_SPAN_US);
volatile uint32_t riseUs[3] = {}, fallUs[3] = {};
volatile uint16_t pulseUs[3] = {};
volatile uint8_t inputLevels = 0, inputRiseSeen = 0;
volatile uint32_t outputRiseUs[2] = {}, outputFallUs[2] = {};
volatile uint16_t outputPulseUs[2] = {};
volatile uint8_t outputLevels = 0;
bool hardwareReady = false, rcValid = false, estopActive = true;
char receiveLine[64];
uint8_t lineLength = 0;
bool dropping = false;
uint32_t lastStatus = 0;

// UNO D2/D3/D4 are PD2/PD3/PD4. Only an edge on that exact bit
// updates its channel. No external-interrupt or library callback routing.
ISR(PCINT2_vect) {
  const uint8_t levels = PIND;
  const uint8_t changed = (levels ^ inputLevels) & (_BV(PD2) | _BV(PD3) | _BV(PD4));
  inputLevels = levels;
  const uint32_t now = micros();
  for (uint8_t i = 0; i < 3; ++i) {
    const uint8_t mask = uint8_t(1U << (i + 2));
    if (!(changed & mask)) continue;
    if (levels & mask) {
      riseUs[i] = now; inputRiseSeen |= mask;
    } else {
      const uint32_t width = now - riseUs[i];
      pulseUs[i] = (inputRiseSeen & mask) && width >= 900 && width <= 2100 ? uint16_t(width) : 0;
      inputRiseSeen &= uint8_t(~mask);
      fallUs[i] = now;
    }
  }
}
// D9/D10 feedback is observed separately on PORTB; never read via digitalRead().
ISR(PCINT0_vect) {
  const uint8_t levels = PINB;
  const uint8_t changed = (levels ^ outputLevels) & (_BV(PB1) | _BV(PB2));
  outputLevels = levels;
  const uint32_t now = micros();
  for (uint8_t side = 0; side < 2; ++side) {
    const uint8_t pin = side == 0 ? LEFT_ESC_PIN : RIGHT_ESC_PIN;
    const uint8_t mask = uint8_t(1U << (pin - 8));
    if (!(changed & mask)) continue;
    if (levels & mask) outputRiseUs[side] = now;
    else {
      outputPulseUs[side] = uint16_t(now - outputRiseUs[side]);
      outputFallUs[side] = now;
    }
  }
}
uint16_t limited(int value) { return constrain(value, 1100, 1900); }
void readInputs() {
  uint16_t pulses[3]; uint32_t stamps[3];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (uint8_t i = 0; i < 3; ++i) { pulses[i] = pulseUs[i]; stamps[i] = fallUs[i]; }
  }
  const uint32_t now = micros();
  bool fresh[3];
  for (uint8_t i = 0; i < 3; ++i)
    fresh[i] = pulses[i] >= 900 && pulses[i] <= 2100 && uint32_t(now - stamps[i]) < 100000;
  const bool previouslyValid = rcValid;
  rcValid = hardwareReady && fresh[0] && fresh[1] && fresh[2];
  // Receiver connection initializes both ESCs without ROS or a stick movement.
  if (!previouslyValid && rcValid) powerOnStartup.start(millis());
  // A detected receiver-PWM loss also cancels an active AUTO command once.
  // A later explicit host arm may resume AUTO; no persistent stop lock is used.
  if (previouslyValid && !rcValid) controller.disarm();
  const kabot::Mode selected = kabot::selectMode(pulses[2], fresh[2], fresh[0] && fresh[1], hardwareReady);
  const bool manual = selected == kabot::MANUAL;
  // Independent receiver channels follow LEFT_ESC_PIN/RIGHT_ESC_PIN.
  const kabot::ManualOutput manualOutput = manualRc.update(pulses[0], pulses[1], rcValid && manual);
  const int l = manual ? manualOutput.left : COMMAND_NEUTRAL_US;
  const int r = manual ? manualOutput.right : COMMAND_NEUTRAL_US;
  estopActive = ESTOP_PIN >= 0 && digitalRead(ESTOP_PIN) == HIGH;
  if (controller.mode == kabot::MANUAL && selected == kabot::AUTOMATIC)
    modeSwitchNeutral.start(millis());
  controller.inputs(millis(), selected != kabot::INHIBITED && (!manual || rcValid),
                    manual, limited(l), limited(r), estopActive);
}
void outputs() {
  if (!hardwareReady) return;
  const bool automatic = controller.mode == kabot::AUTOMATIC;
  if (powerOnStartup.holding(millis()) || modeSwitchNeutral.holding(millis())) {
    writeEscPwm(NEUTRAL_US, NEUTRAL_US);
    return;
  }
  writeEscPwm(kabot::escOutput(controller.left, automatic ? AUTO_LEFT_REVERSED : LEFT_REVERSED),
              kabot::escOutput(controller.right, automatic ? AUTO_RIGHT_REVERSED : RIGHT_REVERSED));
}
void status() {
  char payload[56], message[64];
  const int length = snprintf(payload, sizeof(payload), "S3,%u,%u,%u,%u,%u,%u,%u,%u,0",
      controller.last_sequence, unsigned(controller.mode), controller.armed,
      controller.left, controller.right, rcValid, ESTOP_PIN >= 0, estopActive);
  snprintf(message, sizeof(message), "%s*%04X\n", payload, kabot::crc16(payload, length));
  // Never block RC/failsafe processing on a disconnected or slow serial host.
  if (Serial.availableForWrite() >= int(strlen(message))) Serial.print(message);
}
void rawStatus() {
  uint16_t p[3], out[2]; uint32_t stamps[2], inputStamps[3];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (uint8_t i = 0; i < 3; ++i) { p[i] = pulseUs[i]; inputStamps[i] = fallUs[i]; }
    for (uint8_t i = 0; i < 2; ++i) { out[i] = outputPulseUs[i]; stamps[i] = outputFallUs[i]; }
  }
  const uint32_t now = micros();
  for (uint8_t i = 0; i < 3; ++i) if (uint32_t(now - inputStamps[i]) >= 100000) p[i] = 0;
  for (uint8_t i = 0; i < 2; ++i) if (uint32_t(now - stamps[i]) >= 100000) out[i] = 0;
  char payload[48], message[56];
  const int length = snprintf(payload, sizeof(payload), "R1,%u,%u,%u,%u,%u", p[0], p[1], p[2], out[0], out[1]);
  snprintf(message, sizeof(message), "%s*%04X\n", payload, kabot::crc16(payload, length));
  if (Serial.availableForWrite() >= int(strlen(message))) Serial.print(message);
}
void serialInput() {
  // Bounded work per loop, including when the host continuously floods junk.
  for (uint8_t budget = 0; budget < 48 && Serial.available(); ++budget) {
    const char c = Serial.read();
    if (c == '\n') {
      if (!dropping) {
        receiveLine[lineLength] = 0;
        kabot::Command command;
        if (kabot::parse(receiveLine, command)) {
          readInputs();
          // Never acknowledge a propulsion command while physical outputs are
          // still held at startup neutral. The sender retries with a fresh arm.
          const bool initializing = powerOnStartup.holding(millis());
          const bool motion = command.operation == 'A' || command.operation == 'C';
          const bool accepted = !(initializing && motion) &&
              controller.command(millis(), command.sequence, command.operation, command.left, command.right);
          if (accepted && command.operation == 'I') powerOnStartup.start(millis());
          outputs(); status();
        }
      }
      lineLength = 0; dropping = false;
    } else if (c != '\r') {
      if (c < 32 || c > 126 || lineLength >= sizeof(receiveLine) - 1) dropping = true;
      if (!dropping) receiveLine[lineLength++] = c;
    }
  }
}
void setup() {
  MCUSR = 0; wdt_disable();
  Serial.begin(115200);
#if KABOT_WIRING_VERIFIED
  static_assert(RC_FIRST_PIN == 2 && RC_SECOND_PIN == 3 && RC_MODE_PIN == 4, "UNO inputs must be D2/D3/D4");
  static_assert((LEFT_ESC_PIN == 9 && RIGHT_ESC_PIN == 10) ||
                (LEFT_ESC_PIN == 10 && RIGHT_ESC_PIN == 9), "UNO ESC outputs must be D9/D10");
  const int pins[] = {LEFT_ESC_PIN, RIGHT_ESC_PIN, RC_FIRST_PIN, RC_SECOND_PIN, RC_MODE_PIN};
  hardwareReady = true;
  for (uint8_t i = 0; i < 5; ++i) {
    hardwareReady = hardwareReady && pins[i] > 1 && pins[i] < NUM_DIGITAL_PINS;
    for (uint8_t j = 0; j < i; ++j) hardwareReady = hardwareReady && pins[i] != pins[j];
    if (ESTOP_PIN >= 0) hardwareReady = hardwareReady && ESTOP_PIN != pins[i];
  }
  hardwareReady = hardwareReady && (ESTOP_PIN < 0 || (ESTOP_PIN > 1 && ESTOP_PIN < NUM_DIGITAL_PINS));
  if (KABOT_COMPILE_ONLY) hardwareReady = false;  // Validation profiles cannot drive ESC pins.
  if (hardwareReady) {
    if (ESTOP_PIN >= 0) pinMode(ESTOP_PIN, INPUT_PULLUP);
    // Load neutral before enabling the hardware outputs.
    beginEscPwm();
    powerOnStartup.start(millis());
    pinMode(RC_FIRST_PIN, INPUT); pinMode(RC_SECOND_PIN, INPUT); pinMode(RC_MODE_PIN, INPUT);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      inputLevels = PIND; outputLevels = PINB;
      PCMSK2 = _BV(PD2) | _BV(PD3) | _BV(PD4);
      PCMSK0 = _BV(PB1) | _BV(PB2);
      PCIFR = _BV(PCIF2) | _BV(PCIF0);
      PCICR |= _BV(PCIE2) | _BV(PCIE0);
    }
  }
#endif
  wdt_enable(WDTO_250MS);
  // Unconfigured builds deliberately NEVER attach ESC pins or drive any output.
}
void loop() {
  wdt_reset();
  readInputs(); outputs(); serialInput();
  if (uint32_t(millis() - lastStatus) >= 100) { lastStatus = millis(); status(); }
  // Separate phase keeps the bounded UART buffer from blocking either report.
  static uint32_t lastRaw = 0;
  if (uint32_t(millis() - lastRaw) >= 100 && uint32_t(millis() - lastStatus) >= 30) { lastRaw = millis(); rawStatus(); }
}
