#pragma once

#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "utils.h"

namespace esphome {
namespace daikin_s21 {

enum class DaikinClimateMode : uint8_t {
  Disabled = '1',
  Auto = '0',
  Dry = '2',
  Cool = '3',
  Heat = '4',
  Fan = '6',
};

enum class DaikinFanMode : uint8_t {
  Auto = 'A',
  Silent = 'B',
  Speed1 = '3',
  Speed2 = '4',
  Speed3 = '5',
  Speed4 = '6',
  Speed5 = '7',
};

namespace StateQuery {
  inline constexpr const char* Basic = "F1";
  inline constexpr const char* OptionalFeatures = "F2";
  inline constexpr const char* OnOffTimer = "F3";
  inline constexpr const char* ErrorStatus = "F4";
  inline constexpr const char* SwingOrHumidity = "F5";
  inline constexpr const char* SpecialModes = "F6";
  inline constexpr const char* DemandAndEcono = "F7";
  inline constexpr const char* OldProtocol = "F8";
  inline constexpr const char* InsideOutsideTemperatures = "F9";
  // FA
  // FB
  inline constexpr const char* ModelCode = "FC";
  inline constexpr const char* IRCounter = "FG";
  inline constexpr const char* V2OptionalFeatures = "FK";
  // FL
  inline constexpr const char* PowerConsumption = "FM";
  // FN 
  // FP
  // FQ
  inline constexpr const char* LouvreAngle = "FR";
  // FS
  // FT
  inline constexpr const char* V3OptionalFeatures = "FU00";
  inline constexpr const char* AllowedTemperatureRange = "FU02";
  // FU04
  inline constexpr const char* ModelName = "FU05";
  inline constexpr const char* ProductionInformation = "FU15";
  inline constexpr const char* ProductionOrder = "FU25";
  inline constexpr const char* IndoorProductionInformation = "FU35";
  inline constexpr const char* OutdoorProductionInformation = "FU45";
  // FV
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
  inline constexpr const char* NewProtocol = "FY00";
  // FY10
  // FY20
}  // namespace StateQuery

namespace EnvironmentQuery {
  inline constexpr const char* PowerOnOff = "RA";
  inline constexpr const char* IndoorUnitMode = "RB";
  inline constexpr const char* TemperatureSetPoint = "RC";
  inline constexpr const char* OnTimerSetting = "RD";
  inline constexpr const char* OffTimerSetting = "RE";
  // RF
  inline constexpr const char* FanMode = "RG";
  inline constexpr const char* InsideTemperature = "RH";
  inline constexpr const char* LiquidTemperature = "RI";
  inline constexpr const char* FanSetPoint = "RK";
  inline constexpr const char* FanSpeed = "RL";
  inline constexpr const char* LouvreAngleSetPoint = "RM";
  inline constexpr const char* VerticalSwingAngle = "RN";
  // RW
  inline constexpr const char* TargetTemperature = "RX"; // (not set point, see details)
  inline constexpr const char* OutsideTemperature = "Ra";
  inline constexpr const char* IndoorFrequencyCommandSignal = "Rb";
  inline constexpr const char* CompressorFrequency = "Rd";
  inline constexpr const char* IndoorHumidity = "Re";
  inline constexpr const char* CompressorOnOff = "Rg";
  inline constexpr const char* UnitState = "RzB2";
  inline constexpr const char* SystemState = "RzC3";
  // Rz52
  // Rz72
} // namespace EnvironmentQuery

namespace StateCommand {
  inline constexpr const char* PowerModeTempFan = "D1";
  // D2
  inline constexpr const char* OnOffTimer = "D3";
  inline constexpr const char* LouvreSwing = "D5";
  inline constexpr const char* Powerful = "D6";
  inline constexpr const char* Econo = "D7";
  // DH
  // DJ
  // DR
}

namespace StateResponse {
  inline constexpr const char* Basic = "G1"; //
  inline constexpr const char* OptionalFeatures = "G2";
  inline constexpr const char* OnOffTimer = "G3";
  inline constexpr const char* ErrorStatus = "G4";
  inline constexpr const char* SwingOrHumidity = "G5"; //
  inline constexpr const char* SpecialModes = "G6"; //
  inline constexpr const char* DemandAndEcono = "G7"; //
  inline constexpr const char* OldProtocol = "G8"; //
  inline constexpr const char* InsideOutsideTemperatures = "G9"; //
  // GA
  // GB
  inline constexpr const char* ModelCode = "GC";
  inline constexpr const char* IRCounter = "GG";
  inline constexpr const char* V2OptionalFeatures = "GK";
  // GL
  inline constexpr const char* PowerConsumption = "GM";
  // GN
  // GP
  // GQ
  inline constexpr const char* LouvreAngle = "GR";
  // GS
  // GT
  inline constexpr const char* V3OptionalFeatures = "GU00";
  inline constexpr const char* AllowedTemperatureRange = "GU02";
  // GU04
  inline constexpr const char* ModelName = "GU05";
  inline constexpr const char* ProductionInformation = "GU15";
  inline constexpr const char* ProductionOrder = "GU25";
  inline constexpr const char* IndoorProductionInformation = "GU35";
  inline constexpr const char* OutdoorProductionInformation = "GU45";
  // GV
  // GX00
	// GX10
	// GX20
	// GX30
	// GX40
	// GX50
	// GX60
	// GX70
	// GX80
	// GX90
	// GXA0
	// GXB0
	// GXC0
	// GXD0
	// GXE0
	// GXF0
	// GX01
	// GX11
	// GX21
	// GX31
	// GX41
	// GX51
	// GX61
	// GX71
	// GX81
  inline constexpr const char* NewProtocol = "GY00"; //
  // GY10
  // GY20
}  // namespace StateResponse

namespace EnvironmentResponse {
  inline constexpr const char* PowerOnOff = "SA";
  inline constexpr const char* IndoorUnitMode = "SB";
  inline constexpr const char* TemperatureSetPoint = "SC";
  inline constexpr const char* OnTimerSetting = "SD";
  inline constexpr const char* OffTimerSetting = "SE";
  inline constexpr const char* FanMode = "SG";
  inline constexpr const char* InsideTemperature = "SH";
  inline constexpr const char* LiquidTemperature = "SI";
  inline constexpr const char* FanSetPoint = "SK";
  inline constexpr const char* FanSpeed = "SL";
  inline constexpr const char* LouvreAngleSetPoint = "SM";
  inline constexpr const char* VerticalSwingAngle = "SN";
  // SW
  inline constexpr const char* TargetTemperature = "SX"; // (not set point, see details)
  inline constexpr const char* OutsideTemperature = "Sa";
  inline constexpr const char* IndoorFrequencyCommandSignal = "Sb";
  inline constexpr const char* CompressorFrequency = "Sd";
  inline constexpr const char* IndoorHumidity = "Se";
  inline constexpr const char* CompressorOnOff = "Sg";
  inline constexpr const char* UnitState = "SzB2";
  inline constexpr const char* SystemState = "SzC3";
  // Sz52
  // Sz72
} // namespace EnvironmentResponse

namespace OldProtocol {
  inline constexpr const char* Protocol0 = "\x30\x00\x00\x00";
  inline constexpr const char* Protocol2_1 = "\x30\x32\x00\x00";
  inline constexpr const char* Protocol2_2 = "\x30\x32\x30\x30";
}  // namespace OldProtocol

namespace NewProtocol {
  inline constexpr const char* Protocol3_00_or_3_10 = "0030";
  inline constexpr const char* Protocol3_20 = "0230";
  inline constexpr const char* Protocol3_40 = "0430";
}  // namespace NewProtocol

namespace HumanReadableProtocol {
  inline constexpr const char* ProtocolUnknown = "unknown";
  inline constexpr const char* Protocol0 = "0";
  inline constexpr const char* Protocol2 = "2";
  inline constexpr const char* Protocol3_0 = "3.0";
  inline constexpr const char* Protocol3_1 = "3.1";
  inline constexpr const char* Protocol3_2 = "3.2";
  inline constexpr const char* Protocol3_4 = "3.4";
} // namespace HumanReadableProtocol

std::string daikin_climate_mode_to_string(DaikinClimateMode mode);
std::string daikin_fan_mode_to_string(DaikinFanMode mode);

inline float c10_c(int16_t c10) { return c10 / 10.0; }
inline float c10_f(int16_t c10) { return c10_c(c10) * 1.8 + 32.0; }

class DaikinS21 : public PollingComponent {
 public:
  void update() override;
  void dump_config() override;
  void set_uarts(uart::UARTComponent *tx, uart::UARTComponent *rx);
  void set_debug_protocol(bool set) { this->debug_protocol = set; }
  bool is_ready() { return this->ready; }

  bool is_power_on() { return this->power_on; }
  DaikinClimateMode get_climate_mode() { return this->mode; }
  DaikinFanMode get_fan_mode() { return this->fan; }
  float get_setpoint() { return this->setpoint / 10.0; }
  void set_daikin_climate_settings(bool power_on, DaikinClimateMode mode,
                                   float setpoint, DaikinFanMode fan_mode);
  void set_swing_settings(bool swing_v, bool swing_h);
  bool send_cmd(std::vector<uint8_t> code, std::vector<uint8_t> payload);

  float get_temp_inside() { return this->temp_inside / 10.0; }
  float get_temp_outside() { return this->temp_outside / 10.0; }
  float get_temp_coil() { return this->temp_coil / 10.0; }
  uint16_t get_fan_rpm() { return this->fan_rpm; }
  bool is_idle() { return this->idle; }
  bool get_swing_h() { return this->swing_h; }
  bool get_swing_v() { return this->swing_v; }
  bool get_powerful() { return this->powerful; }
  bool get_econo() { return this->econo; }
  void set_powerful_settings(bool value);
  void set_econo_settings(bool value);
  void set_has_presets(bool value) {
    this->has_presets = value;
  };

 protected:
  bool read_frame(std::vector<uint8_t> &payload);
  void write_frame(std::vector<uint8_t> payload);
  bool s21_query(std::vector<uint8_t> code);
  bool parse_response(std::vector<uint8_t> rcode, std::vector<uint8_t> payload);
  bool run_queries(std::vector<std::string> queries);
  bool run_query(std::string query);
  void dump_state();
  void check_uart_settings();
  bool wait_byte_available(uint32_t  timeout);
  const char* get_protocol_version();
  std::vector<std::string> get_startup_queries();
  bool run_next_startup_query();
  std::vector<std::string> get_required_update_queries();
  bool run_next_required_query();
  std::vector<std::string> get_optional_update_queries();
  bool run_next_optional_query();
  void set_queries();

  uart::UARTComponent *tx_uart{nullptr};
  uart::UARTComponent *rx_uart{nullptr};
  bool ready = false;
  bool debug_protocol = false;

  bool power_on = false;
  DaikinClimateMode mode = DaikinClimateMode::Disabled;
  DaikinFanMode fan = DaikinFanMode::Auto;
  int16_t setpoint = 21;
  bool swing_v = false;
  bool swing_h = false;
  bool powerful = false;
  bool econo = false;
  int16_t temp_inside = 0;
  int16_t temp_outside = 0;
  int16_t temp_coil = 0;
  uint16_t fan_rpm = 0;
  bool idle = true;
  bool has_presets = true;
  bool protocol_checked = false;
  bool startup_complete = false;
  uint8_t f8_protocol = -1;
  uint8_t f8_protocol_variant = -1;
  uint8_t fy00_protocol_major = -1;
  uint8_t fy00_protocol_minor = -1;
  std::vector<std::string> startup_queries = {StateQuery::OldProtocol, StateQuery::NewProtocol};
  uint8_t startup_query_index = 0;
  std::vector<std::string> required_queries = {};
  uint8_t required_query_index = 0;
  std::vector<std::string> optional_queries = {};
  uint8_t optional_query_index = 0;
};

class DaikinS21Client {
 public:
  void set_s21(DaikinS21 *s21) { this->s21 = s21; }

 protected:
  DaikinS21 *s21;
};

}  // namespace daikin_s21
}  // namespace esphome
