#pragma once

#include <bitset>
#include <ranges>
#include <span>
#include <string_view>
#include <vector>
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "daikin_s21_fan_modes.h"
#include "daikin_s21_queries.h"
#include "daikin_s21_serial.h"
#include "daikin_s21_types.h"

namespace esphome::daikin_s21 {

class DaikinS21 : public PollingComponent {
 public:
  DaikinS21(DaikinSerial * const serial);

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  void set_debug(bool set) { this->debug = set; }

  // external command action
  void set_climate_settings(const DaikinClimateSettings &settings);

  enum ReadoutRequest {
    // binary sensor
    ReadoutUnitStateBits,
    ReadoutSystemStateBits,
    // climate
    ReadoutPresets,
    // sensor
    ReadoutTemperatureTarget,
    ReadoutTemperatureOutside,
    ReadoutTemperatureCoil,
    ReadoutFanSpeed,
    ReadoutSwingAngle,
    ReadoutCompressorFrequency,
    ReadoutHumidity,
    ReadoutDemand,
    ReadoutIRCounter,
    ReadoutPowerConsumption,
    ReadoutOutdoorCapacity,
    // just for bitset sizing
    ReadoutCount,
  };
  void request_readout(const ReadoutRequest request) {
    this->readout_requests.set(request);
  }

  void add_debug_query(std::string_view query_str);

  // callbacks called when a query cycle is complete
  CallbackManager<void(void)> update_callbacks{};

  // value accessors
  bool is_ready() { return this->ready.all(); }
  const DaikinClimateSettings& get_climate_settings() { return this->current.climate; }
  auto get_climate_mode() { return this->current.climate.mode; }
  auto get_climate_action() { return this->current.action; }
  auto get_temp_setpoint() { return this->current.climate.setpoint; }
  auto get_temp_inside() { return this->temp_inside; }
  auto get_temp_target() { return this->temp_target; }
  auto get_temp_outside() { return this->temp_outside; }
  auto get_temp_coil() { return this->temp_coil; }
  auto get_fan_rpm_setpoint() { return this->current.fan_rpm_setpoint; }
  auto get_fan_rpm() { return this->current.fan_rpm; }
  auto get_swing_vertical_angle_setpoint() { return this->current.swing_vertical_angle_setpoint; }
  auto get_swing_vertical_angle() { return this->current.swing_vertical_angle; }
  auto get_ir_counter() { return this->current.ir_counter; }
  auto get_power_consumption() { return this->current.power_consumption; }
  auto get_outdoor_capacity() { return this->current.outdoor_capacity; }
  auto get_compressor_frequency() { return this->compressor_rpm; }
  auto get_humidity() { return this->humidity; }
  auto get_demand() { return this->demand; }
  auto get_unit_state() { return this->current.unit_state; }
  auto get_system_state() { return this->current.system_state; }
  auto get_software_version() { return this->software_version.data(); }
  bool is_active() { return this->current.active; }
  std::span<const uint8_t> get_query_result(std::string_view query_str);

  // callbacks for serial events
  void handle_serial_result(DaikinSerial::Result result, std::span<const uint8_t> response = {});
  void handle_serial_idle();

 protected:
  DaikinSerial &serial;

  // communication state
  void dump_state();
  bool is_free_run() const { return this->get_update_interval() == 0; }
  void trigger_cycle();
  void start_cycle();
  enum ReadyCommand : uint8_t {
    ReadyProtocolDetection,
    ReadyOptionalFeatures,
    ReadySensorReadout,
    ReadyModelDetection,
    ReadyActiveSource,
    ReadyPowerfulSource,
    ReadyCount, // just for bitset sizing
  };
  std::bitset<ReadyCount> ready{};
  void ready_state_machine();
  void check_ready_protocol_detection();
  void check_ready_optional_features();
  void check_ready_sensor_readout();
  void check_ready_model_detection();
  void check_ready_active_source();
  void check_ready_powerful_source();
  void send_command(std::string_view command, std::span<const uint8_t> payload);
  bool cycle_triggered{};
  bool cycle_active{};
  std::bitset<ReadoutCount> readout_requests{};
  std::string_view current_command{};
  std::vector<DaikinQuery> queries{}; // pool of possible queries, can't be touched during a query cycle
  decltype(queries)::iterator active_query{queries.begin()};  // current active query, valid for lifetime of query cycle
  void reset_queries();
  DaikinQuery& get_query(std::string_view query_str);
  void enable_query(std::string_view query_str);

  // query handlers
  void handle_nop(std::span<const uint8_t> payload) {}
  void handle_state_basic(std::span<const uint8_t> payload);
  void handle_state_swing_or_humidity(std::span<const uint8_t> payload);
  void handle_state_special_modes(std::span<const uint8_t> payload);
  void handle_state_demand_and_econo(std::span<const uint8_t> payload);
  void handle_state_inside_outside_temperature(std::span<const uint8_t> payload);
  void handle_state_model_code_v2(std::span<const uint8_t> payload);
  void handle_state_ir_counter(std::span<const uint8_t> payload);
  void handle_state_power_consumption(std::span<const uint8_t> payload);
  void handle_state_outdoor_capacity(std::span<const uint8_t> payload);
  void handle_env_power_on_off(std::span<const uint8_t> payload);
  void handle_env_indoor_unit_mode(std::span<const uint8_t> payload);
  void handle_env_temperature_setpoint(std::span<const uint8_t> payload);
  void handle_env_swing_mode(std::span<const uint8_t> payload);
  void handle_env_fan_mode(std::span<const uint8_t> payload);
  void handle_env_inside_temperature(std::span<const uint8_t> payload);
  void handle_env_liquid_temperature(std::span<const uint8_t> payload);
  void handle_env_fan_speed_setpoint(std::span<const uint8_t> payload);
  void handle_env_fan_speed(std::span<const uint8_t> payload);
  void handle_env_vertical_swing_angle_setpoint(std::span<const uint8_t> payload);
  void handle_env_vertical_swing_angle(std::span<const uint8_t> payload);
  void handle_env_target_temperature(std::span<const uint8_t> payload);
  void handle_env_outside_temperature(std::span<const uint8_t> payload);
  void handle_env_indoor_frequency_command_signal(std::span<const uint8_t> payload);
  void handle_env_compressor_frequency(std::span<const uint8_t> payload);
  void handle_env_indoor_humidity(std::span<const uint8_t> payload);
  void handle_env_compressor_on_off(std::span<const uint8_t> payload);
  void handle_env_unit_state(std::span<const uint8_t> payload);
  void handle_env_system_state(std::span<const uint8_t> payload);
  void handle_misc_model_v0(std::span<const uint8_t> payload);
  void handle_misc_software_version(std::span<const uint8_t> payload);

  // debugging support
  bool debug{};
  uint32_t last_state_dump_ms{};
  uint32_t cycle_time_start_ms{};
  uint32_t cycle_time_ms{};

  // settings
  struct {
    DaikinClimateSettings climate{};
    climate::ClimateAction action_reported = climate::CLIMATE_ACTION_OFF; // raw readout
    climate::ClimateAction action = climate::CLIMATE_ACTION_OFF; // corrected at end of cycle
    uint16_t fan_rpm_setpoint{};
    uint16_t fan_rpm{};
    int16_t swing_vertical_angle_setpoint{};
    int16_t swing_vertical_angle{};
    uint16_t ir_counter{};
    uint16_t power_consumption{};
    uint8_t outdoor_capacity{};
    DaikinUnitState unit_state{};
    DaikinSystemState system_state{};
    // modifiers
    bool active{};      // actively using the compressor
    bool quiet{};       // outdoor unit fan/compressor limit
    bool econo{};       // limits demand for power consumption
    bool powerful{};    // maximum output (20 minute timeout), mutaully exclusive with quiet and econo
    bool comfort{};     // fan angle depends on heating/cooling action
    bool streamer{};    // electron emitter decontamination?
    bool sensor{};      // "intelligent eye" PIR occupancy setpoint offset
    bool sensor_led{};  // the sensor LED is on
  } current{};

  struct {
    DaikinClimateSettings climate{ .mode = climate::CLIMATE_MODE_AUTO }; // unsupported sentinel value, see set_climate_settings
    bool activate_climate{};
    bool activate_swing_mode{};
    bool activate_preset{};
  } pending{};

  // current values
  DaikinC10 temp_inside{};
  DaikinC10 temp_target{};
  DaikinC10 temp_outside{};
  DaikinC10 temp_coil{};
  uint16_t compressor_rpm{};
  uint8_t humidity{50};
  uint8_t demand{};

  // protocol support
  ProtocolVersion protocol_version{ProtocolUndetected};
  DaikinModel modelV0{ModelUnknown};
  DaikinModel modelV2{ModelUnknown};
  std::array<char, 8+1> software_version{};

  struct {
    // for alternate readout
    bool fan_mode_query{};
    bool inside_temperature_query{};
    bool outside_temperature_query{};
    bool humidity_query{};
    bool unit_system_state_queries{};
    ActiveSource active_source{ActiveSource::Unknown};
    PowerfulSource powerful_source{PowerfulSource::Unknown};
    // supported
    bool fan{};
    bool swing{};
    bool horiz_swing{};
    char model_info{'?'};
    bool humidify{};
    bool dehumidify{};
    bool dry{};
    bool demand{};
    bool powerful{};
    bool econo{};
    bool streamer{};
    // unsupported
    bool ac_led{};
    bool laundry{};
    bool elec{};
    bool temp_range{};
    bool motion_detect{};
    bool ac_japan{};
  } support;
};

} // namespace esphome::daikin_s21
