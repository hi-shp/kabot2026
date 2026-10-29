#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include "../kabot_actuator/control_core.h"
#include "../kabot_actuator/protocol.h"
#include "../kabot_actuator/board_config.h"
#include "../kabot_actuator/manual_response.h"

std::string wire(const char* payload) {
  char crc[8]; snprintf(crc, sizeof(crc), "*%04X", kabot::crc16(payload, strlen(payload)));
  return std::string(payload) + crc;
}
bool parse(std::string line) {
  kabot::Command cmd;
  return kabot::parse(&line[0], cmd);
}
void ready(kabot::Controller& c, uint32_t now = 100, uint16_t seq = 10) {
  c.inputs(now, true, false, 1500, 1500, false);
  assert(!c.command(now, seq, 'A', 1500, 1500)); // Must establish a disarmed session first.
  assert(c.command(now, seq, 'D', 1500, 1500));
  assert(c.command(now, uint16_t(seq + 1), 'A', 1500, 1500));
}
int main() {
  static_assert(RC_FIRST_PIN == 2 && RC_SECOND_PIN == 3 && RC_MODE_PIN == 4, "Operator-confirmed receiver wiring");
  static_assert(LEFT_ESC_PIN == 9 && RIGHT_ESC_PIN == 10 && ESTOP_PIN == -1, "Operator-confirmed output wiring");
  for (uint16_t pulse : {uint16_t(900), uint16_t(1000), uint16_t(1299)})
    assert(kabot::modeFromPwm(pulse, true) == kabot::AUTOMATIC);
  for (uint16_t pulse : {uint16_t(1701), uint16_t(2000), uint16_t(2100)})
    assert(kabot::modeFromPwm(pulse, true) == kabot::MANUAL);
  for (uint16_t pulse : {uint16_t(0), uint16_t(899), uint16_t(1300), uint16_t(1500), uint16_t(1700), uint16_t(2101)})
    assert(kabot::modeFromPwm(pulse, true) == kabot::INHIBITED);
  assert(kabot::modeFromPwm(1000, false) == kabot::INHIBITED);
  assert(kabot::modeFromPwm(2000, true, false) == kabot::AUTOMATIC);
  kabot::ManualResponse rc;
  auto manual = rc.update(1700, 1300, true);
  assert(manual.left == 1579 && manual.right == 1421);
  manual = rc.update(1700, 1300, true);
  assert(manual.left == 1579 && manual.right == 1421); // No time/speed integration.
  manual = rc.update(1988, 1500, true);
  assert(manual.left == 1900 && manual.right == 1500); // Inexact endpoint still reaches maximum.
  manual = rc.update(1500, 1940, true);
  assert(manual.left == 1500 && manual.right == 1900); // Completely independent channels.
  manual = rc.update(1899, 2000, true);
  assert(manual.left == 1898 && manual.right == 1900); // Smooth approach, no large endpoint jump.
  manual = rc.update(1050, 2000, true);
  assert(manual.left == 1100 && manual.right == 1900);
  manual = rc.update(1050, 1012, false);
  assert(manual.left == 1500 && manual.right == 1500); // RC->AUTO gives immediate neutral.
  manual = rc.update(1540, 1460, true);
  assert(manual.left == 1500 && manual.right == 1500);
  uint16_t previous = 1100;
  for (uint16_t input = 900; input <= 2100; ++input) {
    manual = rc.update(input, uint16_t(3000 - input), true);
    assert(manual.left >= previous && manual.left >= 1100 && manual.left <= 1900);
    assert(manual.left - previous <= 3); // Max adjacent step after integer rounding.
    assert(manual.left + manual.right == 3000); // Same curve, mirrored reverse.
    const int difference = int(input) - 1500;
    const int magnitude = difference < 0 ? -difference : difference;
    double normalized = magnitude <= 40 ? 0.0 : double(magnitude - 40) / 360.0;
    if (normalized > 1) normalized = 1;
    const uint16_t expected = uint16_t(400 * normalized * normalized + .5);
    assert(manual.left == uint16_t(difference < 0 ? 1500 - expected : 1500 + expected));
    previous = manual.left;
  }
  assert(kabot::centerReferenced(1500, RC_RIGHT_CENTER_US) == 1500);
  static_assert(RC_LEFT_CENTER_US == RC_RIGHT_CENTER_US && RC_LEFT_MIN_US == RC_RIGHT_MIN_US && RC_LEFT_MAX_US == RC_RIGHT_MAX_US, "Identical RC channel settings");
  assert(kabot::crc16("123456789", 9) == 0x29B1);
  assert(parse(wire("K1,65535,C,1560,1500")));
  const char* bad[] = {"K1,-1,C,1500,1500", "K1,65536,C,1500,1500", "K1,1,C,1500,1500,9",
      "K1,1,C,1500,999", "K1,1,C,1500,1901", "K1,1,C,1500,", "K1,1,,1500,1500",
      "K1,1,A", "K1,1,Z,1500,1500", "K1,1,C,1500, 1500", "K1,1,C,1500,+1500"};
  for (const char* text : bad) assert(!parse(wire(text)));
  std::string corrupted = wire("K1,1,C,1560,1500"); corrupted[10] ^= 1;
  assert(!parse(corrupted));
  kabot::Controller c;
  assert(c.mode == kabot::INHIBITED && !c.armed && c.left == 1500 && c.right == 1500);
  ready(c);
  assert(c.command(110, 12, 'C', 1560, 1530));
  assert(c.left == 1560 && c.right == 1530);
  assert(!c.command(200, 12, 'C', 1600, 1600));
  assert(!c.command(200, 11, 'C', 1600, 1600));
  assert(!c.command(200, 13, 'C', 1601, 1500));
  assert(c.left == 1560 && c.right == 1530); // Rejected frame cannot change either side.
  c.inputs(511, true, false, 1500, 1500, false);
  assert(!c.armed && c.left == 1500 && c.right == 1500);
  assert(!c.command(512, 13, 'C', 1560, 1560)); // No revival without explicit arm.
  assert(c.command(512, 13, 'A', 1500, 1500));
  assert(c.command(513, 14, 'C', 1560, 1560));
  c.inputs(514, true, true, 1700, 1300, false);
  assert(!c.armed && c.mode == kabot::MANUAL && c.left == 1700 && c.right == 1300);
  assert(!c.command(515, 15, 'C', 1560, 1560)); // RC priority even with serial traffic.
  assert(c.command(516, 15, 'D', 1500, 1500));
  assert(c.left == 1700 && c.right == 1300); // Host shutdown cannot cancel RC control.
  c.inputs(517, true, false, 1500, 1500, false);
  assert(c.mode == kabot::AUTOMATIC && !c.armed && c.left == 1500 && c.right == 1500);
  assert(!c.command(517, 16, 'C', 1540, 1540)); // RC->AUTO must stop; no retained arming.
  c.inputs(517, true, false, 1700, 1500, false);
  assert(c.left == 1500 && !c.armed);
  assert(!c.command(518, 16, 'A', 1500, 1500)); // Sticks must be neutral to arm.
  c.inputs(519, true, false, 1500, 1500, false);
  assert(c.command(520, 16, 'A', 1500, 1500));
  c.inputs(521, false, false, 1500, 1500, false);
  assert(c.mode == kabot::INHIBITED && !c.armed && c.left == 1500);
  c.inputs(522, true, true, 1900, 1900, true);
  assert(c.mode == kabot::INHIBITED && c.left == 1500 && c.right == 1500);
  kabot::Controller wrap;
  ready(wrap, 0xFFFFFF00U, 65534);
  assert(wrap.command(0xFFFFFF01U, 0, 'C', 1560, 1560));
  assert(!wrap.command(146, 1, 'C', 1560, 1560)); // 401ms, millis wrap + late frame.
  assert(!wrap.armed);
  // Host parser robustness: correctly checksummed arbitrary syntax, no hardware calls.
  uint32_t rng = 17;
  for (unsigned i = 0; i < 20000; ++i) {
    std::string payload = "K1,";
    for (unsigned n = 0; n < i % 45; ++n) { rng = rng * 1664525U + 1013904223U; payload += char(32 + rng % 95); }
    parse(wire(payload.c_str()));
  }
  puts("CONTROL_AND_PROTOCOL_TESTS_PASS (including 20000 malformed-frame cases)");
}
