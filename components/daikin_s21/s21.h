#pragma once

#include <bitset>
#include <ranges>
#include <span>
#include <string_view>
#include <vector>
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
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
  void set_debug(const bool set) { this->debug = set; }

  // external command action
  void set_climate_settings(DaikinClimateSettings climate);
  void set_swing_mode(climate::ClimateSwingMode swing);
  void set_humidity_mode(DaikinHumidityMode humidity);
  void set_mode(DaikinMode mode, bool enable);
  void set_demand_control(uint8_t percent);
  void set_vertical_swing_mode(DaikinVerticalSwingMode swing);

  enum ReadoutRequest {
    // binary sensor
    ReadoutErrorStatus,
    ReadoutUnitStateBits,
    ReadoutSystemStateBits,
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
    ReadoutEnergyConsumptionTotal,
    ReadoutOutdoorCapacity,
    ReadoutEnergyConsumptionClimateModes,
    // select
    ReadoutVerticalSwingMode,
    // multiple components
    ReadoutPowerful,
    ReadoutSwingHumidty,
    ReadoutSpecialModes,
    ReadoutDemandAndEcono,
    ReadoutCount, // just for bitset sizing
  };
  void request_readout(const ReadoutRequest request) {
    this->readout_requests.set(request);
  }

  void add_debug_query(std::string_view query_str);

  // callbacks called when a query cycle is complete
  CallbackManager<void(void)> update_callbacks{};

  // value accessors
  bool is_ready() { return this->ready.all(); }
  auto get_climate() const { return this->climate.value(); }
  auto get_climate_action() const { return this->action; }
  auto get_swing_mode() const { return this->swing_humidity.value().swing; }
  auto get_humidity_mode() const { return this->swing_humidity.value().humidity; }
  auto get_demand_control() const { return this->demand_econo.value().demand; }
  auto get_temp_setpoint() const { return this->climate.value().setpoint; }
  auto get_temp_inside() const { return this->temp_inside; }
  auto get_temp_target() const { return this->temp_target; }
  auto get_temp_outside() const { return this->temp_outside; }
  auto get_temp_coil() const { return this->temp_coil; }
  auto get_fan_rpm_setpoint() const { return this->fan_rpm_setpoint; }
  auto get_fan_rpm() const { return this->fan_rpm; }
  auto get_swing_vertical_angle_setpoint() const { return this->swing_vertical_angle_setpoint; }
  auto get_swing_vertical_angle() const { return this->swing_vertical_angle; }
  auto get_ir_counter() const { return this->ir_counter; }
  auto get_energy_consumption_total() const { return this->energy_consumption_total; }
  auto get_energy_consumption_cooling() const { return this->energy_consumption_cooling; }
  auto get_energy_consumption_heating() const { return this->energy_consumption_heating; }
  auto get_vertical_swing_mode() const { return this->vertical_swing_mode.value(); }
  auto get_outdoor_capacity() const { return this->outdoor_capacity; }
  auto get_compressor_frequency() const { return this->compressor_hz; }
  auto get_humidity() const { return this->humidity; }
  auto get_demand_pull() const { return this->demand_pull; }
  auto get_unit_state() const { return this->unit_state; }
  auto get_system_state() const { return this->system_state; }
  auto get_software_version() const { return this->software_version.data(); }
  auto get_software_revision() const { return this->software_revision.data(); }
  auto get_model_name() const { return this->model_name.data(); }
  bool get_active() const { return this->active; }
  bool get_serial_error() const { return this->serial_error; }
  bool get_mode(DaikinMode mode) const;
  std::span<const uint8_t> get_query_result(std::string_view query_str);
  auto get_cycle_interval_ms() const { return std::max(this->get_update_interval(), this->cycle_time_ms); }

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
  void handle_state_error_status(std::span<const uint8_t> payload);
  void handle_state_swing_humidity_modes(std::span<const uint8_t> payload);
  void handle_state_special_modes(std::span<const uint8_t> payload);
  void handle_state_demand_and_econo(std::span<const uint8_t> payload);
  void handle_state_inside_outside_temperature(std::span<const uint8_t> payload);
  void handle_state_model_code_v2(std::span<const uint8_t> payload);
  void handle_state_ir_counter(std::span<const uint8_t> payload);
  void handle_state_energy_consumption_total(std::span<const uint8_t> payload);
  void handle_vertical_swing_mode(std::span<const uint8_t> payload);
  void handle_state_outdoor_capacity(std::span<const uint8_t> payload);
  void handle_state_model_name(std::span<const uint8_t> payload);
  void handle_state_energy_consumption_climate_modes(std::span<const uint8_t> payload);
  void handle_state_software_revision(std::span<const uint8_t> payload);
  void handle_state_model_v3(std::span<const uint8_t> payload);
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
  uint32_t next_state_dump_ms{};
  uint32_t cycle_time_start_ms{};
  uint32_t cycle_time_ms{};

  // settings
  CommandState<DaikinClimateSettings> climate{};
  CommandState<DaikinSwingHumiditySettings> swing_humidity{};
  CommandState<DaikinSpecialModes> special_modes{};
  CommandState<DaikinDemandEcono> demand_econo{};
  CommandState<DaikinVerticalSwingMode> vertical_swing_mode{};

  // current values
  uint32_t energy_consumption_cooling{};
  uint32_t energy_consumption_heating{};
  DaikinC10 temp_inside{};
  DaikinC10 temp_target{};
  DaikinC10 temp_outside{};
  DaikinC10 temp_coil{};
  uint16_t fan_rpm_setpoint{};  // not supported
  uint16_t fan_rpm{};
  uint16_t compressor_hz{};
  int16_t swing_vertical_angle_setpoint{};  // not supported
  int16_t swing_vertical_angle{};
  uint16_t ir_counter{};
  uint16_t energy_consumption_total{};
  uint8_t humidity{50};
  uint8_t demand_pull{};
  climate::ClimateAction action_reported = climate::CLIMATE_ACTION_OFF; // raw readout
  climate::ClimateAction action = climate::CLIMATE_ACTION_OFF; // corrected at end of cycle
  DaikinUnitState unit_state{};
  DaikinSystemState system_state{};
  bool active{};      // actively using the compressor
  bool serial_error{};

  // protocol support and other static values
  ProtocolVersion protocol_version{ProtocolUndetected};
  DaikinModel modelV0{ModelUnknown};
  DaikinModel modelV2{ModelUnknown};
  DaikinModel modelV3{ModelUnknown};
  std::array<char, 8+1> software_version{"unknown"};
  std::array<char, 8+1> software_revision{"unknown"};
  std::array<char, 22+1> model_name{"unknown"};
  uint8_t outdoor_capacity{};

  struct {
    // for alternate readout
    bool fan_mode_query{};
    bool inside_temperature_query{};
    bool outside_temperature_query{};
    bool humidity_query{};
    bool unit_system_state_queries{};
    ActiveSource active_source{ActiveSourceUnknown};
    PowerfulSource powerful_source{PowerfulSourceUnknown};
    // supported
    bool fan{};
    bool swing{};
    bool horiz_swing{};
    char model_info{'?'};
    bool dry{};
    bool demand{};
    bool powerful{};
    bool econo{};
    bool streamer{};
    uint8_t s_humd{}; // bitfield of supported humidity modes, see protocol docs
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
