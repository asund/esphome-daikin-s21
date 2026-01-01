#include "daikin_s21_types.h"

namespace esphome::daikin_s21 {

DaikinFanMode string_to_daikin_fan_mode(const std::string_view mode) {
  const auto iter = std::ranges::find(supported_daikin_fan_modes, mode, [](const auto &elem){ return std::get<const char *>(elem); });
  if (iter != std::ranges::end(supported_daikin_fan_modes)) {
    return static_cast<DaikinFanMode>(std::ranges::distance(std::begin(supported_daikin_fan_modes), iter));
  }
  return DaikinFanAuto;
}

} // namespace esphome::daikin_s21
