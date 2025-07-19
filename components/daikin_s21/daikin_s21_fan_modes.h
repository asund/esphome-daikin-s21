#pragma once

#include <cstdint>
#include <string_view>

namespace esphome {
namespace daikin_s21 {

enum class DaikinFanMode : uint8_t {
  Auto = 'A',
  Silent = 'B',
  Speed1 = '3',
  Speed2 = '4',
  Speed3 = '5',
  Speed4 = '6',
  Speed5 = '7',
};

static constexpr DaikinFanMode supported_daikin_fan_modes[] = {
  DaikinFanMode::Auto,
  DaikinFanMode::Silent,
  DaikinFanMode::Speed1,
  DaikinFanMode::Speed2,
  DaikinFanMode::Speed3,
  DaikinFanMode::Speed4,
  DaikinFanMode::Speed5,
};

constexpr std::string_view daikin_fan_mode_to_string_view(const DaikinFanMode mode) {
  switch (mode) {
    case DaikinFanMode::Auto:
    default:
      return "Automatic";
    case DaikinFanMode::Silent:
      return "Silent";
    case DaikinFanMode::Speed1:
      return "1";
    case DaikinFanMode::Speed2:
      return "2";
    case DaikinFanMode::Speed3:
      return "3";
    case DaikinFanMode::Speed4:
      return "4";
    case DaikinFanMode::Speed5:
      return "5";
  }
}

constexpr DaikinFanMode string_to_daikin_fan_mode(const std::string_view mode) {
  for (const auto supported_mode : supported_daikin_fan_modes) {
    if (daikin_fan_mode_to_string_view(supported_mode) == mode) {
      return supported_mode;
    }
  }
  return DaikinFanMode::Auto;
}

}  // namespace daikin_s21
}  // namespace esphome
