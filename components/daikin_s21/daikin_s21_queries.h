#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <string_view>
#include <span>
#include "daikin_s21_serial.h"
#include "daikin_s21_types.h"

namespace esphome::daikin_s21 {

namespace StateQuery {
  inline constexpr std::string_view Basic{"F1"};
  inline constexpr std::string_view OptionalFeatures{"F2"};
  inline constexpr std::string_view OnOffTimer{"F3"};
  inline constexpr std::string_view ErrorStatus{"F4"};
  inline constexpr std::string_view SwingOrHumidity{"F5"};
  inline constexpr std::string_view SpecialModes{"F6"};
  inline constexpr std::string_view DemandAndEcono{"F7"};
  inline constexpr std::string_view OldProtocol{"F8"};
  inline constexpr std::string_view InsideOutsideTemperatures{"F9"};
  // FA
  inline constexpr std::string_view FB{"FB"};
  inline constexpr std::string_view ModelCode{"FC"};
  inline constexpr std::string_view IRCounter{"FG"};
  inline constexpr std::string_view V2OptionalFeatures{"FK"};
  // FL
  inline constexpr std::string_view PowerConsumption{"FM"};
  inline constexpr std::string_view ITELC{"FN"};
  inline constexpr std::string_view FP{"FP"};
  inline constexpr std::string_view FQ{"FQ"};
  inline constexpr std::string_view LouvreAngleSetpoint{"FR"};
  inline constexpr std::string_view FS{"FS"};
  inline constexpr std::string_view FT{"FT"};
  inline constexpr std::string_view V3OptionalFeatures{"FU00"};
  inline constexpr std::string_view AllowedTemperatureRange{"FU02"};
  // FU04
  inline constexpr std::string_view ModelName{"FU05"};
  inline constexpr std::string_view ProductionInformation{"FU15"};
  inline constexpr std::string_view ProductionOrder{"FU25"};
  inline constexpr std::string_view IndoorProductionInformation{"FU35"};
  inline constexpr std::string_view OutdoorProductionInformation{"FU45"};
  inline constexpr std::string_view FV{"FV"};
  // FX00
  // FX10
  // FX20
  // FX30
  // FX40
  // FX50
  // FX60
  // FX70
  // FX80
  // FX90
  // FXA0
  // FXB0
  // FXC0
  // FXD0
  // FXE0
  // FXF0
  // FX01
  // FX11
  // FX21
  // FX31
  // FX41
  // FX51
  // FX61
  // FX71
  // FX81
  inline constexpr std::string_view NewProtocol{"FY00"};
  // FY10
  // FY20
} // namespace StateQuery

namespace EnvironmentQuery {
  inline constexpr std::string_view PowerOnOff{"RA"};
  inline constexpr std::string_view IndoorUnitMode{"RB"};
  inline constexpr std::string_view TemperatureSetpoint{"RC"};
  inline constexpr std::string_view OnTimerSetting{"RD"};
  inline constexpr std::string_view OffTimerSetting{"RE"};
  inline constexpr std::string_view SwingMode{"RF"};
  inline constexpr std::string_view FanMode{"RG"};
  inline constexpr std::string_view InsideTemperature{"RH"};
  inline constexpr std::string_view LiquidTemperature{"RI"};
  inline constexpr std::string_view FanSpeedSetpoint{"RK"};
  inline constexpr std::string_view FanSpeed{"RL"};
  inline constexpr std::string_view LouvreAngleSetpoint{"RM"};
  inline constexpr std::string_view VerticalSwingAngle{"RN"};
  inline constexpr std::string_view RW{"RW"};
  inline constexpr std::string_view TargetTemperature{"RX"}; // (not user setpoint, see details)
  inline constexpr std::string_view OutsideTemperature{"Ra"};
  inline constexpr std::string_view IndoorFrequencyCommandSignal{"Rb"};
  inline constexpr std::string_view CompressorFrequency{"Rd"};
  inline constexpr std::string_view IndoorHumidity{"Re"};
  inline constexpr std::string_view CompressorOnOff{"Rg"};
  inline constexpr std::string_view UnitState{"RzB2"};
  inline constexpr std::string_view SystemState{"RzC3"};
  inline constexpr std::string_view Rz{"Rz"};
  inline constexpr std::string_view Rz52{"Rz52"};
  inline constexpr std::string_view Rz72{"Rz72"};
} // namespace EnvironmentQuery

namespace MiscQuery {
  inline constexpr std::string_view Model{"M"};
  inline constexpr std::string_view Version{"V"};
  inline constexpr std::string_view SoftwareVersion{"VS000M"};
} // namespace MiscQuery

namespace StateCommand {
  inline constexpr std::string_view PowerModeTempFan{"D1"};
  // D2
  inline constexpr std::string_view OnOffTimer{"D3"};
  inline constexpr std::string_view LouvreSwingMode{"D5"};
  inline constexpr std::string_view Powerful{"D6"};
  inline constexpr std::string_view Econo{"D7"};
  // DH
  inline constexpr std::string_view DJ{"DJ"};
  inline constexpr std::string_view LouvreAngleSetpoint{"DR"};
}

using PayloadBuffer = std::array<uint8_t, DaikinSerial::STANDARD_PAYLOAD_SIZE>;
using ExtendedPayloadBuffer = std::array<uint8_t, DaikinSerial::EXTENDED_PAYLOAD_SIZE>;

class DaikinQueryValue {
 public:
  static inline constexpr uint8_t software_version_length = 8;

  DaikinQueryValue() {}
  DaikinQueryValue(const std::string_view command) : command(command) {}

  std::string_view command{};

  std::span<const uint8_t> value() const {
    return {this->command == MiscQuery::SoftwareVersion ? DaikinQueryValue::software_version.begin() : this->value_buffer.begin(), value_length};
  }

  void set_value(std::span<const uint8_t> payload);

  // command fetching projection for std::ranges use
  static constexpr std::string_view GetCommand(const DaikinQueryValue &query) { return query.command; }

 protected:
  inline static std::array<uint8_t, software_version_length> software_version;  // static buffer for useful portion of longer software version string
  PayloadBuffer value_buffer;  // last received value
  uint8_t value_length{};
};

class DaikinQueryState : public DaikinQueryValue {
  using handler_fn = void (DaikinS21::*)(std::span<uint8_t>&);

 public:
  DaikinQueryState() {}
  DaikinQueryState(const std::string_view command, const handler_fn handler = nullptr, const bool is_static = false)
      : DaikinQueryValue(command), handler(handler), is_static(is_static) {}

  handler_fn handler{};
  bool is_static{}; // result never changes
  bool acked{};
  uint8_t naks{};
};

class DaikinQueryResult {
 public:
  operator bool() const {
    return this->ack || this->nak;
  }

  bool operator==(const DaikinQueryResult &other) const {
    return std::ranges::equal(this->value, other.value) && (this->ack == other.ack) && (this->nak == other.nak);
  }

  std::span<const uint8_t> value{};
  bool ack{};
  bool nak{};
};

/* Sentinel value for a never scheduled query, let handler code treat it as nak'd. */
inline constexpr uint8_t QueryNotScheduledString[] = {'N','/','A'};
inline constexpr DaikinQueryResult QueryNotScheduled{QueryNotScheduledString, false, true};

} // namespace esphome::daikin_s21
