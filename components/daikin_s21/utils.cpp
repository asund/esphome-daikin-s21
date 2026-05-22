#include <cinttypes>
#include "esphome/core/alloc_helpers.h"
#include "utils.h"

namespace esphome::daikin_s21 {

std::string hex_repr(std::span<const uint8_t> bytes) {
  return format_hex_pretty(bytes.data(), bytes.size(), ':', false);
}

// Adapated from ESPHome UART debugger
std::string str_repr(std::span<const uint8_t> bytes) {
  std::string res;
  char buf[5];
  for (const auto b : bytes) {
    if (b == 7) {
      res += "\\a";
    } else if (b == 8) {
      res += "\\b";
    } else if (b == 9) {
      res += "\\t";
    } else if (b == 10) {
      res += "\\n";
    } else if (b == 11) {
      res += "\\v";
    } else if (b == 12) {
      res += "\\f";
    } else if (b == 13) {
      res += "\\r";
    } else if (b == 27) {
      res += "\\e";
    } else if (b == 34) {
      res += "\\\"";
    } else if (b == 39) {
      res += "\\'";
    } else if (b == 92) {
      res += "\\\\";
    } else if (b < 32 || b > 127) {
      sprintf(buf, "\\x%02" PRIX8, b);
      res += buf;
    } else {
      res += b;
    }
  }
  return res;
}

} // namespace esphome::daikin_s21
