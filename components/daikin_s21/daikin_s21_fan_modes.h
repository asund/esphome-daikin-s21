#pragma once

#include <cstdint>
#include <ranges>
#include <string_view>
#include <utility>

namespace esphome::daikin_s21 {

enum class DaikinFanMode : uint8_t {
  Auto = 'A',
  Silent = 'B',
  Speed1 = '3',
  Speed2 = '4',
  Speed3 = '5',
  Speed4 = '6',
  Speed5 = '7',
};

static constexpr std::pair<DaikinFanMode, const char *> supported_daikin_fan_modes[] = {
  {DaikinFanMode::Auto, "Automatic"},
  {DaikinFanMode::Silent, "Silent"},
  {DaikinFanMode::Speed1, "1"},
  {DaikinFanMode::Speed2, "2"},
  {DaikinFanMode::Speed3, "3"},
  {DaikinFanMode::Speed4, "4"},
  {DaikinFanMode::Speed5, "5"},
};

constexpr const char * daikin_fan_mode_to_cstr(const DaikinFanMode mode) {
  const auto iter = std::ranges::find(supported_daikin_fan_modes, mode, [](const auto &elem){ return elem.first; });
  if (iter != std::ranges::end(supported_daikin_fan_modes)) {
    return iter->second;
  }
  return supported_daikin_fan_modes[0].second;
}

constexpr DaikinFanMode string_to_daikin_fan_mode(const std::string_view mode) {
  const auto iter = std::ranges::find(supported_daikin_fan_modes, mode, [](const auto &elem){ return elem.second; });
  if (iter != std::ranges::end(supported_daikin_fan_modes)) {
    return iter->first;
  }
  return supported_daikin_fan_modes[0].first;
}

} // namespace esphome::daikin_s21
