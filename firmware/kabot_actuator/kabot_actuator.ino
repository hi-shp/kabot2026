#include <Arduino.h>
#include <Servo.h>
#include <PinChangeInterrupt.h>
#include <util/atomic.h>
#include <avr/wdt.h>
#include "board_config.h"
#include "control_core.h"
#include "protocol.h"

// This firmware is an actuator/RC arbiter. Navigation and MPC run on Jetson.
kabot::Controller controller(NEUTRAL_US, AUTO_DELTA_LIMIT_US);
Servo leftEsc, rightEsc;
volatile uint32_t riseUs[3] = {}, fallUs[3] = {};
volatile uint16_t pulseUs[3] = {};
volatile uint32_t outputRiseUs[2] = {}, outputFallUs[2] = {};
volatile uint16_t outputPulseUs[2] = {};
bool hardwareReady = false, rcValid = false, estopActive = true;
char receiveLine[64];
uint8_t lineLength = 0;
bool dropping = false;
uint32_t lastStatus = 0;

void edge(uint8_t index, uint8_t pin) {
  const uint32_t now = micros();
  if (digitalRead(pin)) riseUs[index] = now;
  else {
    const uint32_t width = now - riseUs[index];
    // Invalidate on a malformed pulse; never retain the last valid throttle.
    pulseUs[index] = width >= 900 && width <= 2100 ? uint16_t(width) : 0;
    fallUs[index] = now;
  }
}
void firstEdge() { edge(0, RC_FIRST_PIN); }
void secondEdge() { edge(1, RC_SECOND_PIN); }
void modeEdge() { edge(2, RC_MODE_PIN); }
void outputEdge(uint8_t side, uint8_t pin) {
  const uint32_t now = micros();
  if (digitalRead(pin)) outputRiseUs[side] = now;
  else {
    outputPulseUs[side] = uint16_t(now - outputRiseUs[side]);
    outputFallUs[side] = now;
  }
}
void leftOutputEdge() { outputEdge(0, LEFT_ESC_PIN); }
void rightOutputEdge() { outputEdge(1, RIGHT_ESC_PIN); }

bool supportedInput(int pin) {
  return pin >= 0 && pin < NUM_DIGITAL_PINS &&
      (digitalPinToInterrupt(pin) != NOT_AN_INTERRUPT || digitalPinToPCINT(pin) != NOT_AN_INTERRUPT);
}
void capture(int pin, void (*callback)()) {
  pinMode(pin, INPUT);  // Receiver supplies a PWM signal; no pull-up mask of a disconnected line.
  if (digitalPinToInterrupt(pin) != NOT_AN_INTERRUPT)
    attachInterrupt(digitalPinToInterrupt(pin), callback, CHANGE);
  else attachPCINT(digitalPinToPCINT(pin), callback, CHANGE);
}
uint16_t limited(int value) { return constrain(value, 1100, 1900); }
void readInputs() {
  uint16_t pulses[3]; uint32_t stamps[3];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (uint8_t i = 0; i < 3; ++i) { pulses[i] = pulseUs[i]; stamps[i] = fallUs[i]; }
  }
  const uint32_t now = micros();
  rcValid = hardwareReady;
  for (uint8_t i = 0; i < 3; ++i)
    rcValid = rcValid && pulses[i] >= 900 && pulses[i] <= 2100 && uint32_t(now - stamps[i]) < 100000;
  // Intermediate/unknown switch level is inhibited, never an automatic fallback.
  rcValid = rcValid && (pulses[2] < 1300 || pulses[2] > 1700);
  bool manual = RC_HIGH_IS_MANUAL ? pulses[2] > 1700 : pulses[2] < 1300;
  int l = pulses[0], r = pulses[1];
  if (RC_MIX) {
    uint16_t ml, mr;
    kabot::legacyRcMix(pulses[0], pulses[1], ml, mr, RC_FIRST_CENTER_US, RC_SECOND_CENTER_US, NEUTRAL_US);
    l = ml; r = mr;
  }
  estopActive = ESTOP_PIN >= 0 && digitalRead(ESTOP_PIN) == HIGH;
  controller.inputs(millis(), rcValid, manual, limited(l), limited(r), estopActive);
}
void outputs() {
  if (!hardwareReady) return;
  leftEsc.writeMicroseconds(LEFT_REVERSED ? 2 * NEUTRAL_US - controller.left : controller.left);
  rightEsc.writeMicroseconds(RIGHT_REVERSED ? 2 * NEUTRAL_US - controller.right : controller.right);
}
void status() {
  char payload[56], message[64];
  const int length = snprintf(payload, sizeof(payload), "S1,%u,%u,%u,%u,%u,%u,%u,%u",
      controller.last_sequence, unsigned(controller.mode), controller.armed,
      controller.left, controller.right, rcValid, ESTOP_PIN >= 0, estopActive);
  snprintf(message, sizeof(message), "%s*%04X\n", payload, kabot::crc16(payload, length));
  // Never block RC/failsafe processing on a disconnected or slow serial host.
  if (Serial.availableForWrite() >= int(strlen(message))) Serial.print(message);
}
void rawStatus() {
  uint16_t p[3], out[2]; uint32_t stamps[2];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    for (uint8_t i = 0; i < 3; ++i) p[i] = pulseUs[i];
    for (uint8_t i = 0; i < 2; ++i) { out[i] = outputPulseUs[i]; stamps[i] = outputFallUs[i]; }
  }
  for (uint8_t i = 0; i < 2; ++i) if (uint32_t(micros() - stamps[i]) > 100000) out[i] = 0;
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
          controller.command(millis(), command.sequence, command.operation, command.left, command.right);
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
  static_assert(RC_FIRST_PIN >= 0 && RC_SECOND_PIN >= 0 && RC_MODE_PIN >= 0, "RC wiring not configured");
  const int pins[] = {LEFT_ESC_PIN, RIGHT_ESC_PIN, RC_FIRST_PIN, RC_SECOND_PIN, RC_MODE_PIN};
  hardwareReady = supportedInput(RC_FIRST_PIN) && supportedInput(RC_SECOND_PIN) && supportedInput(RC_MODE_PIN);
  for (uint8_t i = 0; i < 5; ++i) {
    hardwareReady = hardwareReady && pins[i] > 1 && pins[i] < NUM_DIGITAL_PINS;
    for (uint8_t j = 0; j < i; ++j) hardwareReady = hardwareReady && pins[i] != pins[j];
    if (ESTOP_PIN >= 0) hardwareReady = hardwareReady && ESTOP_PIN != pins[i];
  }
  hardwareReady = hardwareReady && (ESTOP_PIN < 0 || (ESTOP_PIN > 1 && ESTOP_PIN < NUM_DIGITAL_PINS));
  if (KABOT_COMPILE_ONLY) hardwareReady = false;  // Validation profiles cannot drive ESC pins.
  if (hardwareReady) {
    if (ESTOP_PIN >= 0) pinMode(ESTOP_PIN, INPUT_PULLUP);
    // Servo defaults to 1500us; verify NEUTRAL_US before enabling this profile.
    leftEsc.writeMicroseconds(NEUTRAL_US); rightEsc.writeMicroseconds(NEUTRAL_US);
    leftEsc.attach(LEFT_ESC_PIN, 1100, 1900); rightEsc.attach(RIGHT_ESC_PIN, 1100, 1900);
    capture(RC_FIRST_PIN, firstEdge); capture(RC_SECOND_PIN, secondEdge); capture(RC_MODE_PIN, modeEdge);
    // Observe emitted GPIO pulses without changing their OUTPUT configuration.
    attachPCINT(digitalPinToPCINT(LEFT_ESC_PIN), leftOutputEdge, CHANGE);
    attachPCINT(digitalPinToPCINT(RIGHT_ESC_PIN), rightOutputEdge, CHANGE);
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
