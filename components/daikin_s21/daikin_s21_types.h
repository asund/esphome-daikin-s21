#pragma once

#include <compare>
#include <functional>
#include <limits>
#include <type_traits>
#include "esphome/components/climate/climate.h"
#include "daikin_s21_fan_modes.h"

namespace esphome::daikin_s21 {

// forward declaration
class DaikinS21;

class ProtocolVersion {
 public:
  uint8_t major{};
  uint8_t minor{};

  auto operator<=>(const ProtocolVersion&) const = default;
};

inline constexpr ProtocolVersion ProtocolUndetected{0xFF, 0xFF};
inline constexpr ProtocolVersion ProtocolUnknown{0,0xFF};  // treat as a protocol 0

/**
 * Class representing a temperature in degrees C scaled by 10, the most granular internal temperature measurement format
 */
class DaikinC10 {
 public:
  static constexpr auto nan_sentinel = 500; // internal Daikin 0x80

  constexpr DaikinC10() = default;

  template <typename T, typename std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
  constexpr DaikinC10(const T valf) : value((static_cast<int16_t>(valf * 10 * 2) + 1) / 2) {} // round to nearest 0.1C

  template <typename T, typename std::enable_if_t<std::is_integral_v<T>, bool> = true>
  constexpr DaikinC10(const T vali) : value(vali) {}

  explicit constexpr operator float() const { return (value == nan_sentinel) ? NAN : (value / 10.0F); }
  explicit constexpr operator int16_t() const { return value; }
  constexpr float f_degc() const { return static_cast<float>(*this); }

  constexpr auto operator<=>(const DaikinC10 &other) const = default;
  constexpr DaikinC10 operator+(const DaikinC10 &arg) const { return this->value + arg.value; }
  constexpr DaikinC10 operator-(const DaikinC10 &arg) const { return this->value - arg.value; }
  constexpr DaikinC10 operator*(const DaikinC10 &arg) const { return this->value * arg.value; }
  constexpr DaikinC10 operator/(const DaikinC10 &arg) const { return this->value / arg.value; }

  static DaikinC10 diff(const DaikinC10 &a, const DaikinC10 &b) { return std::abs(a.value - b.value); }

 private:
  int16_t value{};
};

inline constexpr DaikinC10 SETPOINT_STEP{1.0F}; // Daikin setpoint granularity
inline constexpr DaikinC10 TEMPERATURE_STEP{0.5F}; // Daikin temperature sensor granularity
inline constexpr DaikinC10 TEMPERATURE_INVALID{DaikinC10::nan_sentinel}; // NaN

/**
 * Possible sources of active flag.
 */
enum class ActiveSource : uint8_t {
  Unknown,
  CompressorOnOff,  // directly read from query
  UnitState,        // interpreted from unit state bitfield
  Unsupported,      // hardcoded to active
};

/**
 * Possible sources of powerful flag.
 */
enum class PowerfulSource : uint8_t {
  Unknown,
  SpecialModes, // directly read from query
  UnitState,    // interpreted from unit state bitfield
  Disabled,
};

/**
 * Unit state (RzB2) bitfield decoder
 */
class DaikinUnitState {
 public:
  constexpr DaikinUnitState(const uint8_t value = 0U) : raw(value) {}
  constexpr bool powerful() const { return (this->raw & 0x1) != 0; }
  constexpr bool defrost() const { return (this->raw & 0x2) != 0; }
  constexpr bool active() const { return (this->raw & 0x4) != 0; }
  constexpr bool online() const { return (this->raw & 0x8) != 0; }
  uint8_t raw{};
};

/**
 * System state (RzC3) bitfield decoder
 */
class DaikinSystemState {
 public:
  constexpr DaikinSystemState(const uint8_t value = 0U) : raw(value) {}
  constexpr bool locked() const { return (this->raw & 0x01) != 0; }
  constexpr bool active() const { return (this->raw & 0x04) != 0; }
  constexpr bool defrost() const { return (this->raw & 0x08) != 0; }
  constexpr bool multizone_conflict() const { return (this->raw & 0x20) != 0; }
  uint8_t raw{};
};

struct DaikinClimateSettings {
  climate::ClimateMode mode{climate::CLIMATE_MODE_OFF};
  DaikinC10 setpoint{23};
  climate::ClimateSwingMode swing{climate::CLIMATE_SWING_OFF};
  DaikinFanMode fan{DaikinFanMode::Auto};
  climate::ClimatePreset preset{climate::CLIMATE_PRESET_NONE};

  constexpr bool operator==(const DaikinClimateSettings &other) const = default;
};

// MiscQuery::Model or StateQuery::ModelCode responses, reversed ascii hex (these are byte swapped from controller response)
using DaikinModel = uint16_t;
inline constexpr DaikinModel ModelUnknown{0xFFFF};

// V0 model families?
inline constexpr DaikinModel ModelRXB35C2V1B{0x2806}; // indoor FTXB25C2V1B
inline constexpr DaikinModel Model4MXL36TVJU{0x35E3}; // indoor CTXS07LVJU, FTXS12LVJU, FTXS15LVJU
inline constexpr DaikinModel ModelRXC24AXVJU{0x4431}; // indoor FTXC24AXVJU

// V2+ indoor units
inline constexpr DaikinModel ModelFTXC24AXVJU{0x0B66};  // outdoor RXC24AXVJU

} // namespace esphome::daikin_s21
