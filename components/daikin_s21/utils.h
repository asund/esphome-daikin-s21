#pragma once

#include <cstdint>
#include <string>
#include <span>

namespace esphome::daikin_s21 {

// printf format specifier macros for std::string_view
#define PRI_SV ".*s"
#define PRI_SV_ARGS(x) (x).size(), (x).data()

std::string hex_repr(std::span<const uint8_t> bytes);
std::string str_repr(std::span<const uint8_t> bytes);

} // namespace esphome::daikin_s21
