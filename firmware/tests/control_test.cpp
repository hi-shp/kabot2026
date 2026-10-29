#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include "../kabot_actuator/control_core.h"
#include "../kabot_actuator/protocol.h"

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
  uint16_t left, right;
  kabot::legacyRcMix(1485, 1490, left, right); assert(left == 1500 && right == 1500);
  kabot::legacyRcMix(1385, 1490, left, right); assert(left == 1600 && right == 1600);
  kabot::legacyRcMix(1485, 1590, left, right); assert(left == 1600 && right == 1400);
  kabot::legacyRcMix(1460, 1510, left, right); assert(left == 1500 && right == 1500);
  kabot::legacyRcMix(900, 2100, left, right); assert(left == 1900 && right == 1475);
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
