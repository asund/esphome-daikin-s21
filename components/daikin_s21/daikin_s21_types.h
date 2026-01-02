#pragma once

#include <bitset>
#include <compare>
#include <functional>
#include <limits>
#include <type_traits>
#include <utility>
#include "esphome/components/climate/climate.h"

namespace esphome::daikin_s21 {

// forward declaration
class DaikinS21;

struct ProtocolVersion {
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
  static constexpr int16_t nan_sentinel = std::numeric_limits<int16_t>::min();

  constexpr DaikinC10() = default;

  template <typename T, typename std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
  constexpr DaikinC10(const T valf) : value(std::isfinite(valf) ? ((static_cast<int16_t>(valf * 10 * 2) + 1) / 2) : nan_sentinel) {} // round to nearest 0.1C

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

 private:
  int16_t value{};
};

inline constexpr DaikinC10 SETPOINT_STEP{1.0F}; // Daikin setpoint granularity
inline constexpr DaikinC10 TEMPERATURE_STEP{0.5F}; // Daikin temperature sensor granularity
inline constexpr DaikinC10 TEMPERATURE_INVALID{DaikinC10::nan_sentinel}; // NaN

/**
 * Function template for looking up an index enum value from an encoding.
 *
 * Encoding is checked and the first index is returned if out of range.
 *
 * @tparam T enum type
 * @tparam N size of enum and encoding table
 * @tparam &encoding_table encoding table
 * @param encoding encoding to look up
 * @return associated index enum value
 */
template <typename T, size_t N, const std::array<uint8_t, N> &encoding_table>
constexpr T encoding_to_enum(const uint8_t encoding) {
  const auto iter = std::ranges::find(encoding_table, encoding);
  if (iter != std::ranges::end(encoding_table)) {
    return static_cast<T>(std::ranges::distance(std::begin(encoding_table), iter));
  }
  return static_cast<T>(0);
}

/**
 * Function template for looking up an encoding from an index enum value for enums outside of our control.
 *
 * Index is checked and the first value is returned if out of range.
 *
 * @tparam T enum type
 * @tparam N size of enum and encoding table
 * @tparam &encoding_table encoding table
 * @param index index to look up
 * @return associated encoding
 */
template <typename T, size_t N, const std::array<uint8_t, N> &encoding_table>
constexpr uint8_t enum_to_encoding_checked(const T index) {
  if (index < N) {
    return encoding_table[index];
  }
  return encoding_table[0];
}

/**
 * Command states.
 *
 * For values that can be commanded by the user, track the progress of applying the setting.
 * Used to temporarily return the pending value to prevent logic and UI glitches.
 * @tparam T the type of the setting value sent in a command
 */
template<typename T>
class CommandState {
  static constexpr uint8_t active_value = std::numeric_limits<uint8_t>::min();
  static constexpr uint8_t staged_value = std::numeric_limits<uint8_t>::max();
  uint8_t state{active_value};

 public:
  // command state tracking
  constexpr bool staged() const { return this->state == staged_value; } // the pending value should be sent to the unit
  constexpr void reset() {
    this->state = active_value;
    this->pending = this->active;
  }
  constexpr void set_confirm_ms(const uint32_t cycle_interval_ms, const uint32_t timeout_ms = 1000) {
    this->state = std::min(static_cast<int>(timeout_ms / cycle_interval_ms) + 2, staged_value - 1); // +2 for truncation and short first cycle
  }

  // values
  T pending{};
  T active{};

  constexpr const T& value() const { return (this->state == active_value) ? this->active : this->pending; }
  constexpr void stage(const T& value) {
    this->pending = value;
    this->state = staged_value;
  }
  constexpr void check_confirm() {
    if ((this->state != active_value) && (this->staged() == false)) {
      if (this->pending == this->active) {
        this->state = active_value;
      } else {
        this->state--;
      }
    }
  }
};

enum DaikinFanMode : uint8_t {
  DaikinFanAuto,
  DaikinFanSilent,
  DaikinFan1,
  DaikinFan2,
  DaikinFan3,
  DaikinFan4,
  DaikinFan5,
  DaikinFanModeCount, // for array sizing
};

inline constexpr std::array<std::pair<uint8_t, const char *>, DaikinFanModeCount> supported_daikin_fan_modes = {{
  {'A', "Auto"},
  {'B', "Silent"},
  {'3', "1"},
  {'4', "2"},
  {'5', "3"},
  {'6', "4"},
  {'7', "5"},
}};

constexpr const char * daikin_fan_mode_to_cstr(const DaikinFanMode mode) {
  return std::get<const char *>(supported_daikin_fan_modes[mode]);
}

constexpr DaikinFanMode string_to_daikin_fan_mode(const std::string_view mode) {
  const auto iter = std::ranges::find(supported_daikin_fan_modes, mode, [](const auto &elem){ return std::get<const char *>(elem); });
  if (iter != std::ranges::end(supported_daikin_fan_modes)) {
    return static_cast<DaikinFanMode>(std::ranges::distance(std::begin(supported_daikin_fan_modes), iter));
  }
  return DaikinFanAuto;
}

struct DaikinClimateSettings {
  climate::ClimateMode mode{climate::CLIMATE_MODE_OFF};
  DaikinFanMode fan{DaikinFanAuto};
  DaikinC10 setpoint{23};

  constexpr bool operator==(const DaikinClimateSettings &other) const = default;
};

enum DaikinHumidityMode : uint8_t {
  DaikinHumidityOff,
  DaikinHumidityLow,
  DaikinHumidityStandard,
  DaikinHumidityHigh,
  DaikinHumidityContinuous,
  DaikinHumidityModeCount,  // for array sizing
};

struct DaikinSwingHumiditySettings {
  climate::ClimateSwingMode swing{climate::ClimateSwingMode::CLIMATE_SWING_OFF};
  DaikinHumidityMode humidity{DaikinHumidityOff};

  constexpr bool operator==(const DaikinSwingHumiditySettings &other) const = default;
};

enum DaikinMode : uint8_t {
  ModePowerful,     // maximum output (20 minute timeout), mutaully exclusive with comfort/quiet/econo
  ModeComfort,      // fan angle depends on heating/cooling action
  ModeQuiet,        // outdoor unit fan/compressor limit
  ModeStreamer,     // electron emitter decontamination
  ModeSensorLED,    // motion sensor LED control
  ModeMotionSensor, // "intelligent eye" PIR occupancy setpoint offset
  ModeEcono,        // limits demand for power consumption
  DaikinModeCount,  // for mode array sizing
};

inline constexpr auto DaikinSpecialModesCount = ModeMotionSensor + 1;
using DaikinSpecialModes = std::bitset<DaikinSpecialModesCount>;

struct DaikinDemandEcono {
  uint8_t demand{100};
  bool econo{};

  constexpr bool operator==(const DaikinDemandEcono &other) const = default;
};

enum DaikinVerticalSwingMode : uint8_t {
  DaikinVerticalSwingOff,
  DaikinVerticalSwingTop,
  DaikinVerticalSwingUpper,
  DaikinVerticalSwingMiddle,
  DaikinVerticalSwingLower,
  DaikinVerticalSwingBottom,
  DaikinVerticalSwingOn,
  DaikinVerticalSwingModeCount, // for array sizing
};

/**
 * Possible sources of active flag.
 */
enum ActiveSource : uint8_t {
  ActiveSourceUnknown,
  ActiveSourceCompressorOnOff,  // directly read from query
  ActiveSourceUnitState,        // interpreted from unit state bitfield
  ActiveSourceUnsupported,      // hardcoded to active
  ActiveSourceCount,            // for array sizing
};

/**
 * Possible sources of powerful flag.
 */
enum PowerfulSource : uint8_t {
  PowerfulSourceUnknown,
  PowerfulSourceSpecialModes, // directly read from query
  PowerfulSourceUnitState,    // interpreted from unit state bitfield
  PowerfulSourceDisabled,
  PowerfulSourceCount,        // for array sizing
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
