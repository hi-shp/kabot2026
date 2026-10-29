#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace kabot {
inline uint16_t crc16(const char* text, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= uint16_t(uint8_t(text[i])) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = crc & 0x8000 ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
  }
  return crc;
}
inline bool number(const char*& p, uint16_t& result, char separator) {
  uint32_t value = 0;
  uint8_t digits = 0;
  while (*p >= '0' && *p <= '9') {
    if (++digits > 5) return false;
    value = value * 10 + uint8_t(*p++ - '0');
    if (value > 65535) return false;
  }
  if (!digits || *p != separator) return false;
  if (separator) ++p;
  result = uint16_t(value); return true;
}
inline bool verify(char* line) {
  char* star = strchr(line, '*');
  if (!star || strlen(star + 1) != 4) return false;
  uint16_t checksum = 0;
  for (const char* p = star + 1; *p; ++p) {
    uint8_t digit;
    if (*p >= '0' && *p <= '9') digit = *p - '0';
    else if (*p >= 'A' && *p <= 'F') digit = *p - 'A' + 10;
    else return false;
    checksum = uint16_t((checksum << 4) | digit);
  }
  if (checksum != crc16(line, size_t(star - line))) return false;
  *star = 0; return true;
}
struct Command { uint16_t sequence, left, right; char operation; };
inline bool parse(char* line, Command& cmd) {
  if (!verify(line) || strncmp(line, "K1,", 3) != 0) return false;
  const char* p = line + 3;
  if (!number(p, cmd.sequence, ',')) return false;
  cmd.operation = *p++;
  if ((cmd.operation != 'A' && cmd.operation != 'C' && cmd.operation != 'D') || *p++ != ',') return false;
  return number(p, cmd.left, ',') && number(p, cmd.right, 0) &&
      cmd.left >= 1100 && cmd.left <= 1900 && cmd.right >= 1100 && cmd.right <= 1900;
}
}  // namespace kabot
