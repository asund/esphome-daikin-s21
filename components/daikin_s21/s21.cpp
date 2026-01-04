#include <cinttypes>
#include <numeric>
#include <ranges>
#include "daikin_s21_queries.h"
#include "s21.h"
#include "utils.h"

using namespace esphome;

namespace esphome::daikin_s21 {

static const char *const TAG = "daikin_s21";

constexpr uint8_t climate_mode_to_s21(const climate::ClimateMode mode) {
  switch (mode) {
      break;
    case climate::CLIMATE_MODE_HEAT:
      return '4';
    case climate::CLIMATE_MODE_COOL:
      return '3';
    case climate::CLIMATE_MODE_FAN_ONLY:
      return '6';
    case climate::CLIMATE_MODE_DRY:
      return '2';
    case climate::CLIMATE_MODE_OFF:
    case climate::CLIMATE_MODE_HEAT_COOL:
    case climate::CLIMATE_MODE_AUTO:
    default:
      return '1';
  }
}

constexpr climate::ClimateMode s21_to_climate_mode(const uint8_t mode) {
  switch (mode) {
    case '2': // dry
      return climate::CLIMATE_MODE_DRY;
    case '3': // cool
      return climate::CLIMATE_MODE_COOL;
    case '4': // heat
      return climate::CLIMATE_MODE_HEAT;
    case '6': // fan_only
      return climate::CLIMATE_MODE_FAN_ONLY;
    case '0': // heat_cool cooling
    case '1': // heat_cool setting
    case '7': // heat_cool heating
    default:
      return climate::CLIMATE_MODE_HEAT_COOL;
  }
}

constexpr DaikinFanMode s21_to_fan_mode(const uint8_t mode) {
  const auto iter = std::ranges::find(supported_daikin_fan_modes, mode, [](const auto &elem){ return std::get<uint8_t>(elem); });
  if (iter != std::ranges::end(supported_daikin_fan_modes)) {
    return static_cast<DaikinFanMode>(std::ranges::distance(std::begin(supported_daikin_fan_modes), iter));
  }
  return DaikinFanAuto;
}

constexpr uint8_t fan_mode_to_s21(const DaikinFanMode mode) {
  return std::get<uint8_t>(supported_daikin_fan_modes[mode]);
}

constexpr climate::ClimateAction s21_to_climate_action(const uint8_t action) {
  switch (action) {
    case '2': // dry
      return climate::CLIMATE_ACTION_DRYING;
    case '0': // heat_cool cooling
    case '3': // cool
      return climate::CLIMATE_ACTION_COOLING;
    case '4': // heat
    case '7': // heat_cool heating
      return climate::CLIMATE_ACTION_HEATING;
    case '6': // fan_only
      return climate::CLIMATE_ACTION_FAN;
    case '1': // heat_cool
    default:
      return climate::CLIMATE_ACTION_IDLE;
  }
}

static constexpr std::array<uint8_t, climate::CLIMATE_SWING_HORIZONTAL + 1> climate_swing_encodings = {{
  '0',  // CLIMATE_SWING_OFF
  '7',  // CLIMATE_SWING_BOTH
  '1',  // CLIMATE_SWING_VERTICAL
  '2',  // CLIMATE_SWING_HORIZONTAL
}};
constexpr auto s21_to_climate_swing_mode = encoding_to_enum<climate::ClimateSwingMode, climate::CLIMATE_SWING_HORIZONTAL + 1, climate_swing_encodings>;
constexpr auto climate_swing_mode_to_s21 = enum_to_encoding_checked<climate::ClimateSwingMode, climate::CLIMATE_SWING_HORIZONTAL + 1, climate_swing_encodings>;

static constexpr std::array<uint8_t, DaikinHumidityModeCount> humidity_mode_encodings = {{
  '0',
  0x3A,
  0x3B,
  0x3C,
  0xFF,
}};
constexpr auto s21_to_humidity_mode = encoding_to_enum<DaikinHumidityMode, DaikinHumidityModeCount, humidity_mode_encodings>;

static constexpr std::array<uint8_t, DaikinVerticalSwingModeCount> vertical_swing_mode_encodings = {{
  '0',
  '1',
  '2',
  '3',
  '4',
  '5',
  '?',
}};
constexpr auto s21_to_vertical_swing_mode = encoding_to_enum<DaikinVerticalSwingMode, DaikinVerticalSwingModeCount, vertical_swing_mode_encodings>;

static constexpr std::array<const char *, ActiveSourceCount> active_source_strings = {{
  "Rg",
  "RzB2",
  "assumed on",
  "undetected",
}};

static constexpr std::array<const char *, PowerfulSourceCount> powerful_source_strings = {{
  "G6",
  "RzB2",
  "disabled",
  "undetected",
}};

/**
 * Apply the characteristics of a new vertical swing mode to a climate swing mode
 */
void apply_vertical_swing_mode(const DaikinVerticalSwingMode vertical_swing, climate::ClimateSwingMode &swing) {
  if (vertical_swing == DaikinVerticalSwingOn) {
    if (swing == climate::CLIMATE_SWING_OFF) {
      swing = climate::CLIMATE_SWING_VERTICAL;
    } else if (swing == climate::CLIMATE_SWING_HORIZONTAL) {
      swing = climate::CLIMATE_SWING_BOTH;
    }
  } else {
    if (swing == climate::CLIMATE_SWING_VERTICAL) {
      swing = climate::CLIMATE_SWING_OFF;
    } else if (swing == climate::CLIMATE_SWING_BOTH) {
      swing = climate::CLIMATE_SWING_HORIZONTAL;
    }
  }
}

/**
 * Apply the characteristics of a new climate swing mode to a vertical swing mode
 */
void apply_swing_mode(const climate::ClimateSwingMode swing, DaikinVerticalSwingMode &vertical_swing) {
  switch (swing) {
    case climate::CLIMATE_SWING_OFF:
    case climate::CLIMATE_SWING_HORIZONTAL:
      if (vertical_swing == DaikinVerticalSwingOn) { // don't overwrite discrete steps
        vertical_swing = DaikinVerticalSwingOff;
      }
      break;
    case climate::CLIMATE_SWING_VERTICAL:
    case climate::CLIMATE_SWING_BOTH:
      vertical_swing = DaikinVerticalSwingOn;
      break;
    default:
      break;
  }
}

/**
 * Convert reversed ASCII number to an integer
 *
 * @param bytes ASCII bytes of format <ones><tens><hundreds[><neg/pos>,<thousands>]
 * @param base base used for conversion
 * @return int16_t integer representation of string
 */
int16_t bytes_to_num(std::span<const uint8_t> bytes, const int base = 10) {
  std::array<char, 4+1> buffer{};
  std::ranges::reverse_copy(bytes, buffer.begin());
  return std::strtoul(buffer.data(), nullptr, base);
}

DaikinS21::DaikinS21(DaikinSerial * const serial)
  : serial(*serial) { // serial required in config, non-null
  // populate supported queries
  // this is done in the constructor so debug queries are only ever added to this list
  // see https://github.com/revk/ESP32-Faikout/wiki/S21-Protocol for documentation
  this->queries = {
    {StateQuery::Basic, &DaikinS21::handle_state_basic, 4},
    {StateQuery::OptionalFeatures, &DaikinS21::handle_nop, 4, true},
    // {StateQuery::OnOffTimer, &DaikinS21::handle_nop, 4},  // unused, use home assistant for scheduling
    {StateQuery::ErrorStatus, &DaikinS21::handle_state_error_status, 4},
    {StateQuery::SwingHumidityModes, &DaikinS21::handle_state_swing_humidity_modes, 4},
    {StateQuery::SpecialModes, &DaikinS21::handle_state_special_modes, 4},
    {StateQuery::DemandAndEcono, &DaikinS21::handle_state_demand_and_econo, 4},
    {StateQuery::OldProtocol, &DaikinS21::handle_nop, 4, true},  // protocol version detect
    {StateQuery::InsideOutsideTemperatures, &DaikinS21::handle_state_inside_outside_temperature, 4},
    // {StateQuery::FB, &DaikinS21::handle_nop, 4}, // unknown
    {StateQuery::ModelCode, &DaikinS21::handle_state_model_code_v2, 4, true},
    {StateQuery::IRCounter, &DaikinS21::handle_state_ir_counter, 4},
    {StateQuery::V2OptionalFeatures, &DaikinS21::handle_nop, 4, true},
    {StateQuery::PowerConsumption, &DaikinS21::handle_state_power_consumption, 4},
    // {StateQuery::ITELC, &DaikinS21::handle_nop, 4},  // unknown, daikin intelligent touch controller?
    // {StateQuery::FP, &DaikinS21::handle_nop, 4}, // unknown
    // {StateQuery::FQ, &DaikinS21::handle_nop, 4}, // unknown
    {StateQuery::VerticalSwingMode, &DaikinS21::handle_vertical_swing_mode, 4},
    // {StateQuery::FS, &DaikinS21::handle_nop, 4}, // unknown
    {StateQuery::OutdoorCapacity, &DaikinS21::handle_state_outdoor_capacity, 4},
    {StateQuery::V3OptionalFeatures, &DaikinS21::handle_nop, 32, true},
    {StateQuery::ModelName, &DaikinS21::handle_state_model_name, 32, true},
    // {StateQuery::FV, &DaikinS21::handle_nop}, // unknown
    {StateQuery::NewProtocol, &DaikinS21::handle_nop, 4, true},  // protocol version detect
    {StateQuery::SoftwareRevision, &DaikinS21::handle_state_software_revision, 32, true},
    {StateQuery::V3Model, &DaikinS21::handle_state_model_v3, 4, true}, // unknown/unconfirmed
    // {EnvironmentQuery::PowerOnOff, &DaikinS21::handle_env_power_on_off, 1}, // redundant
    // {EnvironmentQuery::IndoorUnitMode, &DaikinS21::handle_env_indoor_unit_mode, 1}, // redundant
    // {EnvironmentQuery::TemperatureSetpoint, &DaikinS21::handle_env_temperature_setpoint, 4},  // redundant
    // {EnvironmentQuery::OnTimerSetting, &DaikinS21::handle_nop}, // unused, unsupported
    // {EnvironmentQuery::OffTimerSetting, &DaikinS21::handle_nop},  // unused, unsupported
    // {EnvironmentQuery::SwingMode, &DaikinS21::handle_env_swing_mode, 2},  // redundant
    {EnvironmentQuery::FanMode, &DaikinS21::handle_env_fan_mode, 1},
    {EnvironmentQuery::InsideTemperature, &DaikinS21::handle_env_inside_temperature, 4},
    {EnvironmentQuery::LiquidTemperature, &DaikinS21::handle_env_liquid_temperature, 4},
    // {EnvironmentQuery::FanSpeedSetpoint, &DaikinS21::handle_env_fan_speed_setpoint, 3},  // not supported yet, can translate DaikinFanMode to RPM
    {EnvironmentQuery::FanSpeed, &DaikinS21::handle_env_fan_speed, 3},
    // {EnvironmentQuery::LouvreAngleSetpoint, &DaikinS21::handle_env_vertical_swing_angle_setpoint, 4},  // not supported yet
    {EnvironmentQuery::VerticalSwingAngle, &DaikinS21::handle_env_vertical_swing_angle, 4},
    // {EnvironmentQuery::RW, &DaikinS21::handle_nop, 2},  // unknown, "00" for me
    {EnvironmentQuery::TargetTemperature, &DaikinS21::handle_env_target_temperature, 4},
    {EnvironmentQuery::OutsideTemperature, &DaikinS21::handle_env_outside_temperature, 4},
    {EnvironmentQuery::IndoorFrequencyCommandSignal, &DaikinS21::handle_env_indoor_frequency_command_signal, 3},
    {EnvironmentQuery::CompressorFrequency, &DaikinS21::handle_env_compressor_frequency, 3},
    {EnvironmentQuery::IndoorHumidity, &DaikinS21::handle_env_indoor_humidity, 3},
    {EnvironmentQuery::CompressorOnOff, &DaikinS21::handle_env_compressor_on_off, 1},
    {EnvironmentQuery::UnitState, &DaikinS21::handle_env_unit_state, 2},
    {EnvironmentQuery::SystemState, &DaikinS21::handle_env_system_state, 2},
    // {EnvironmentQuery::Rz52, &DaikinS21::handle_nop, 0},  // unknown, "40"for me
    // {EnvironmentQuery::Rz72, &DaikinS21::handle_nop, 0},  // unknown, "23" for me
    {MiscQuery::Model, &DaikinS21::handle_misc_model_v0, 0, true}, // some sort of model? always "3E53" for me, regardless of head unit -- outdoor unit?
    {MiscQuery::Version, &DaikinS21::handle_nop, 0, true}, // purportedly another version, always "00C0" for me
    {MiscQuery::SoftwareVersion, &DaikinS21::handle_misc_software_version, 0, true},
  };
}

void DaikinS21::setup() {
  this->reset_queries();
  this->ready.reset();
  this->start_poller(); // for reinit
  this->disable_loop();
}

/**
 * Component update loop.
 *
 * Used for deferred work. Printing too much in the timer callback context causes warnings about blocking for too long.
 */
void DaikinS21::loop() {
  this->dump_state(); // use Component::defer if more work items are added
  this->disable_loop();
}

/**
 * PollingComponent update loop.
 *
 * Disable the polling loop (this function) if configured to free run.
 * It executes once on startup
 *
 * Trigger a cycle.
 */
void DaikinS21::update() {
  if (this->is_free_run()) {
    this->stop_poller();
  }
  this->trigger_cycle();
}

void DaikinS21::dump_config() {
  ESP_LOGCONFIG(TAG, "DaikinS21:");
  ESP_LOGCONFIG(TAG, "  Update interval: %" PRIu32, this->get_update_interval());
  // todo basic detect output
}

/**
 * Set the climate settings bundle and trigger a write to the unit.
 */
void DaikinS21::set_climate_settings(const DaikinClimateSettings climate) {
  if (this->get_climate() != climate) {
    ESP_LOGD(TAG, "Mode: %s  Setpoint: %.1f  Fan: %s",
      LOG_STR_ARG(climate::climate_mode_to_string(climate.mode)),
      climate.setpoint.f_degc(),
      LOG_STR_ARG(daikin_fan_mode_to_cstr(climate.fan)));
    this->climate.stage(climate);
    this->trigger_cycle();
  }
}

void DaikinS21::set_swing_mode(const climate::ClimateSwingMode swing) {
  if (this->get_swing_mode() != swing) {
    this->swing_humidity.stage({ swing, this->get_humidity_mode() }); // shares D6 with humidity, stage complete command
    this->trigger_cycle();
  }
}

void DaikinS21::set_humidity_mode(const DaikinHumidityMode humidity) {
  if (this->get_humidity_mode() != humidity) {
    this->swing_humidity.stage({ this->get_swing_mode(), humidity }); // shares D6 with swing, stage complete command
    this->trigger_cycle();
  }
}

/**
 * Set the enable value of the specified mode and trigger a write to the unit if it changed.
 */
void DaikinS21::set_mode(const DaikinMode mode, const bool enable) {
  if (mode < DaikinSpecialModesCount) {
    DaikinSpecialModes current = this->special_modes.value(); // local copy to modify
    if (current[mode] != enable) {
      current[mode] = enable;
      this->special_modes.stage(current);
      this->trigger_cycle();
    }
  } else if (mode == ModeEcono) {
    if (this->demand_econo.value().econo != enable) {
      this->demand_econo.stage({ this->get_demand_control(), enable });  // shares D7 with demand control, stage complete command
      this->trigger_cycle();
    }
  }
}

/**
 * Set the demand control percentage and trigger a write to the unit if it changed.
 */
void DaikinS21::set_demand_control(const uint8_t percent) {
  if (this->get_demand_control() != percent) {
    this->demand_econo.stage({ percent, this->get_mode(ModeEcono) });  // shares D7 with econo, stage complete command
    this->trigger_cycle();
  }
}

/**
 * Set the vertical swing mode and trigger a write to the unit if it changed.
 */
void DaikinS21::set_vertical_swing_mode(DaikinVerticalSwingMode swing) {
  if (this->get_vertical_swing_mode() != swing) {
    this->vertical_swing_mode.stage(swing);
    this->trigger_cycle();
  }
}

/**
 * Add debug query to the list, flagging them as such
 *
 * @note important to only call during setup phase so as not to invalidate the query iterator
 */
void DaikinS21::add_debug_query(std::string_view query_str) {
  const auto iter = std::ranges::find(this->queries, query_str, DaikinQuery::GetCommand);
  if (iter == this->queries.end()) {
    auto &query = this->queries.emplace_back(query_str, &DaikinS21::handle_nop);
    query.is_debug = true;  // add unkown query
  } else {
    iter->is_debug = true;  // could exist but not enabled during init
  }
}

/**
 * Get the enable value of the specified mode
 */
bool DaikinS21::get_mode(const DaikinMode mode) const {
  if (mode < DaikinSpecialModesCount) {
    return this->special_modes.value()[mode];
  } else if (mode == ModeEcono) {
    return this->demand_econo.value().econo;
  }
  return false;
}

/**
 * Get the result value of a query for external access
 *
 * @param query_str the query value to fetch
 * @return std::span<const uint8_t> the response to that query
 */
std::span<const uint8_t> DaikinS21::get_query_result(std::string_view query_str) {
  return this->get_query(query_str).value();
}

/**
 * Trigger a query cycle
 *
 * If not active, kick off a cycle.
 * If already active, set a flag to run another when it finishes.
 */
void DaikinS21::trigger_cycle() {
  if (this->cycle_active == false) {
    this->start_cycle();
  } else {
    this->cycle_triggered = true;
  }
}

/**
 * Start a query cycle, tramsitting the first.
 */
void DaikinS21::start_cycle() {
  this->cycle_active = true;
  this->cycle_triggered = this->is_free_run();
  this->cycle_time_start_ms = millis();
  this->active_query = std::ranges::find_if(this->queries, DaikinQuery::IsEnabled);
  this->serial.send_frame(this->active_query->command);
}

/**
 * Initialize the state of the query pool
 */
void DaikinS21::reset_queries() {
  for (auto &query : this->queries) {
    query.clear();
    // enable initial protocol detection queries
    if ((query.command == StateQuery::OldProtocol) || (query.command == StateQuery::NewProtocol)) {
      query.enabled = true;
    }
  }
}

/**
 * Get the result of a query.
 */
DaikinQuery& DaikinS21::get_query(std::string_view query_str) {
  const auto iter = std::ranges::find(this->queries, query_str, DaikinQuery::GetCommand);
  if (iter != this->queries.end()) {
    return *iter;
  }
  // This isn't necessary but will prevent a segfault if I add a bug
  ESP_LOGW(TAG, "Query not found %" PRI_SV, PRI_SV_ARGS(query_str));
  static DaikinQuery dummy{};
  return dummy; // prevent crash in caller when an unknown query requested
}

void DaikinS21::enable_query(std::string_view query_str) {
  this->get_query(query_str).enabled = true;
}

/**
 * Refine the pool of polling queries, adding or removing them as we learn about the unit.
 */
void DaikinS21::ready_state_machine() {
  if (this->ready[ReadyProtocolDetection] == false) {
    this->check_ready_protocol_detection();
  }

  if (this->ready[ReadyProtocolDetection]) {
    if (this->ready[ReadyOptionalFeatures] == false) {
      this->check_ready_optional_features();
    }

    if (this->ready[ReadyOptionalFeatures]) {
      // Enable alternate sensor readout if primary queries are unsupported
      if (this->ready[ReadySensorReadout] == false) {
        this->check_ready_sensor_readout();
      }
    }

    // Detect the model and handle model-specific quirks
    if (this->ready[ReadyModelDetection] == false) {
      this->check_ready_model_detection();
    }

    if (this->ready[ReadyModelDetection]) {
      if (this->ready[ReadyActiveSource] == false) {
        this->check_ready_active_source();
      }

      // Select the source of the powerful flag
      if (this->ready[ReadyPowerfulSource] == false) {
        this->check_ready_powerful_source();
      }
    }
  }

  // Finally, all queries should be scheduled and important ones read out
  if (this->is_ready()) {
    // Populate pending state caches with current values so change detection on future commands works
    this->climate.reset();
    this->swing_humidity.reset();
    this->special_modes.reset();
    this->demand_econo.reset();
    this->vertical_swing_mode.reset();

    // Schedule any user specified debug queries, done last so as to not duplicate automatically added queries
    for (auto &query : this->queries | std::views::filter(DaikinQuery::IsDebug)) {
      query.enabled = true;
    }
  }
}

void DaikinS21::check_ready_protocol_detection() {
  static constexpr uint8_t old_version_0[] = {'0',0,0,0};
  static constexpr uint8_t old_version_1[] = {'0','1',0,0};
  static constexpr uint8_t old_version_2or3[] = {'0','2',0,0};
  static constexpr uint8_t old_version_31plus[] = {'0','2','0','0'};
  const auto &old_proto = this->get_query(StateQuery::OldProtocol);
  const auto &new_proto = this->get_query(StateQuery::NewProtocol);

  this->protocol_version = ProtocolUndetected;
  // Check availability first, both protocol indicators were enabled on init so skip to the chase
  if (old_proto.success() && new_proto.failed()) {
    if (std::ranges::equal(old_proto.value(), old_version_0)) {
      this->protocol_version = ProtocolVersion(0);
    } else if (std::ranges::equal(old_proto.value(), old_version_1)) {
      this->protocol_version = ProtocolVersion(1);
    } else if (std::ranges::equal(old_proto.value(), old_version_2or3) || std::ranges::equal(old_proto.value(), old_version_31plus)) {
      this->protocol_version = ProtocolVersion(2); // NAK for NewProtocol rules out 3.0
    } else {
      this->protocol_version = ProtocolUnknown;
    }
  } else if (old_proto.ready() && new_proto.success()) {
    const uint16_t raw_version = bytes_to_num(new_proto.value());
    this->protocol_version = {static_cast<uint8_t>(raw_version / 100), static_cast<uint8_t>(raw_version % 100)};
    // fixup on the 2 / 3.0 protocol border
    if (this->protocol_version == ProtocolVersion(3,0)) {
      if (old_proto.success() && std::ranges::equal(old_proto.value(), old_version_31plus)) {
        this->protocol_version = ProtocolVersion(3,10);
      }
    }
  } else if (old_proto.failed() && new_proto.failed()) {
    // both nak'd, even though we're talking to the unit?
    this->protocol_version = ProtocolUnknown;
  }

  // Print some info if we're falling back to version 0
  if (this->protocol_version == ProtocolUnknown) {
    ESP_LOGE(TAG, "Unable to detect protocol version! Old: %s New: %s",
      str_repr(old_proto.value()).c_str(),
      str_repr(new_proto.value()).c_str());
  }

  // check if complete and handle results if so
  this->ready[ReadyProtocolDetection] = this->protocol_version != ProtocolUndetected;
  if (this->ready[ReadyProtocolDetection]) {
    ESP_LOGD(TAG, "Protocol version %" PRIu8 ".%" PRIu8 " detected", this->protocol_version.major, this->protocol_version.minor);
    // >= ProtocolVersion(0)
    this->enable_query(StateQuery::Basic);
    this->enable_query(StateQuery::OptionalFeatures);
    if (this->readout_requests[ReadoutErrorStatus]) {
      this->enable_query(StateQuery::ErrorStatus);
    }
    if (this->readout_requests[ReadoutSwingHumidty]) {
      this->enable_query(StateQuery::SwingHumidityModes);
    }
    this->enable_query(EnvironmentQuery::FanMode);
    if (this->readout_requests[ReadoutFanSpeed]) {
      this->enable_query(EnvironmentQuery::FanSpeed);
    }
    if (this->readout_requests[ReadoutSwingAngle]) {
      this->enable_query(EnvironmentQuery::VerticalSwingAngle);
    }
    if (this->readout_requests[ReadoutHumidity]) {
      this->enable_query(EnvironmentQuery::IndoorHumidity);
    }
    this->enable_query(EnvironmentQuery::CompressorOnOff);
    this->enable_query(MiscQuery::SoftwareVersion);
    if (this->protocol_version <= ProtocolVersion(2)) {
      this->enable_query(MiscQuery::Model);
      this->enable_query(MiscQuery::Version);
    }
    if (this->protocol_version >= ProtocolVersion(2)) {
      if (this->readout_requests[ReadoutPowerful] || this->readout_requests[ReadoutSpecialModes]) {
        this->enable_query(StateQuery::SpecialModes);
      }
      if (this->readout_requests[ReadoutDemandAndEcono]) {
        this->enable_query(StateQuery::DemandAndEcono);
      }
      this->enable_query(StateQuery::ModelCode);
      if (this->readout_requests[ReadoutIRCounter]) {
        this->enable_query(StateQuery::IRCounter);
      }
      this->enable_query(StateQuery::V2OptionalFeatures);
      if (this->readout_requests[ReadoutPowerConsumption]) {
        this->enable_query(StateQuery::PowerConsumption);
      }
      if (this->readout_requests[DaikinS21::ReadoutVerticalSwingMode]) {
        this->enable_query(StateQuery::VerticalSwingMode);
      }
      if (this->readout_requests[ReadoutOutdoorCapacity]) {
        this->enable_query(StateQuery::OutdoorCapacity);
      }
    }
    if (this->protocol_version >= ProtocolVersion(3)) {
      this->enable_query(StateQuery::V3OptionalFeatures);
      this->enable_query(StateQuery::SoftwareRevision);
      this->enable_query(StateQuery::V3Model);
    }
    if (this->protocol_version >= ProtocolVersion(3,40)) {
      this->enable_query(StateQuery::ModelName);
    }
    this->enable_query(EnvironmentQuery::InsideTemperature);
    if (this->readout_requests[ReadoutTemperatureCoil]) {
      this->enable_query(EnvironmentQuery::LiquidTemperature);
    }
    if (this->readout_requests[ReadoutTemperatureTarget]) {
      this->enable_query(EnvironmentQuery::TargetTemperature);
    }
    if (this->readout_requests[ReadoutTemperatureOutside]) {
      this->enable_query(EnvironmentQuery::OutsideTemperature);
    }
    if (this->readout_requests[ReadoutDemand]) {
      this->enable_query(EnvironmentQuery::IndoorFrequencyCommandSignal);
    }
    if (this->readout_requests[ReadoutCompressorFrequency]) {
      this->enable_query(EnvironmentQuery::CompressorFrequency);
    }
  }
}

/**
 * Detect and handle optional features
 */
void DaikinS21::check_ready_optional_features() {
  const auto &v0_features = this->get_query(StateQuery::OptionalFeatures);
  const auto &v2_features = this->get_query(StateQuery::V2OptionalFeatures);
  const auto &v3_features = this->get_query(StateQuery::V3OptionalFeatures);
  // check if complete and handle results if so
  this->ready[ReadyOptionalFeatures] = (v0_features.ready() && v2_features.ready() && v3_features.ready());
  if (this->ready[ReadyOptionalFeatures]) {
    if (v0_features.success()) {
      this->support.fan =           true;
      this->support.swing =         (v0_features[0] & 0b0100);
      this->support.horiz_swing =   (v0_features[0] & 0b1000);
      this->support.model_info =    (v0_features[1] & 0b1000) ? 'N': 'C';
      if (v0_features[3] & 0b0010) {
        this->support.s_humd |=     0xA5;
      }
      if (v0_features[3] & 0b1000) {
        this->support.s_humd |=     0x92;
      }
    }
    if (v2_features.success()) {
      this->support.ac_led =        (v2_features[0] & 0b00000001);
      this->support.laundry =       (v2_features[0] & 0b00001000);
      this->support.elec =          (v2_features[1] & 0b00000001);
      this->support.temp_range =    (v2_features[1] & 0b00000100);
      this->support.motion_detect = (v2_features[1] & 0b00001000);
      this->support.ac_japan =      (v2_features[2] & 0b00000001) == 0;
      if ((v2_features[2] & 0b00000010)) { // v2 gates v0
        this->support.s_humd =      0x10; // "simple humidify mode available", unknown what this means to us
      }
      if ((v2_features[2] & 0b00000100) == false) { // v2 gates v0
        this->support.fan =         false;
        this->support.swing =       false;
        this->support.horiz_swing = false;
      }
      this->support.dry =           (v2_features[2] & 0b00001000);
      if (this->protocol_version > ProtocolVersion(2)) {
        this->support.demand =      (v2_features[3] & 0b00000001);
      }
    }
    if (v3_features.success()) {
      this->support.powerful =      (v3_features[0] == '3');
      this->support.econo =         (v3_features[1] == '3');
      this->support.streamer =      (v3_features[5] == '3');
    }

    ESP_LOGD(TAG, "Optional features detected");
  }
}

/**
 * Enable alternate sensor readout if primary queries are unsupported
 */
void DaikinS21::check_ready_sensor_readout() {
  const auto &fan_mode = this->get_query(EnvironmentQuery::FanMode);
  const auto &inside = this->get_query(EnvironmentQuery::InsideTemperature);
  const auto &outside = this->get_query(EnvironmentQuery::OutsideTemperature);
  const auto &humidity = this->get_query(EnvironmentQuery::IndoorHumidity);
  // check if complete and handle results if so
  this->ready[ReadySensorReadout] = fan_mode.ready() &&
                                    inside.ready() &&
                                    ((this->readout_requests[ReadoutTemperatureOutside] == false) || outside.ready()) &&
                                    ((this->readout_requests[ReadoutHumidity] == false) || humidity.ready());
  if (this->ready[ReadySensorReadout]) {
    this->support.fan_mode_query = fan_mode.success();
    this->support.inside_temperature_query = inside.success();
    this->support.outside_temperature_query = outside.success();
    this->support.humidity_query = humidity.success();
    // enable coarse fallback query if any sensor query failed
    const bool alt = inside.failed() ||
                    (this->readout_requests[ReadoutTemperatureOutside] && outside.failed()) ||
                    (this->readout_requests[ReadoutHumidity] && humidity.failed());
    if (alt) {
      this->enable_query(StateQuery::InsideOutsideTemperatures);
    }
    ESP_LOGD(TAG, "%s sensor readout selected", alt ? "Alternate" : "Dedicated");
  }
}

/**
 * Detect the model and handle model-specific quirks
 */
void DaikinS21::check_ready_model_detection() {
  const auto &model_v0 = this->get_query(MiscQuery::Model);
  const auto &model_v2 = this->get_query(StateQuery::ModelCode);
  // check if complete and handle results if so
  this->ready[ReadyModelDetection] = (model_v0.ready() && model_v2.ready());
  if (this->ready[ReadyModelDetection]) {
    // Unit and system state aren't always reliable, blacklist models here
    switch (this->modelV0) {
      case ModelRXB35C2V1B:
      case ModelRXC24AXVJU:
        this->support.unit_system_state_queries = false;
        break;
      case ModelUnknown:
      default:
        this->support.unit_system_state_queries = true;
        break;
    }
    if (this->support.unit_system_state_queries) {
      if (this->readout_requests[ReadoutUnitStateBits]) {
        this->enable_query(EnvironmentQuery::UnitState);
      }
      if (this->readout_requests[ReadoutSystemStateBits]) {
        this->enable_query(EnvironmentQuery::SystemState);
      }
    }
    // no v2 handling required yet
    ESP_LOGD(TAG, "Model %04" PRIX16 " %04" PRIX16 " detected", this->modelV0, this->modelV2);
  }
}

/**
 * Select the source of the active flag
 */
void DaikinS21::check_ready_active_source() {
  const auto &compressor_on_off = this->get_query(EnvironmentQuery::CompressorOnOff); // always enabled
  if (compressor_on_off.success()) {
    this->support.active_source = ActiveSourceCompressorOnOff;
  } else if (compressor_on_off.failed()) {
    auto &unit_state = this->get_query(EnvironmentQuery::UnitState);
    if (unit_state.success()) {
      this->support.active_source = ActiveSourceUnitState;
    } else if (unit_state.failed()) {
      this->support.active_source = ActiveSourceUnsupported;
    } else if (unit_state.enabled == false) {
      if (this->support.unit_system_state_queries) {
        this->readout_requests.set(ReadoutUnitStateBits);
        unit_state.enabled = true;
      } else {
        this->support.active_source = ActiveSourceUnsupported;
      }
    }
    if (this->support.active_source == ActiveSourceUnsupported) {
      this->support.unit_system_state_queries = false;  // unit state is assumed to be supported, if it isn't then correct the record
      this->active = true;  // always active, reported action will follow unit
    }
  }
  // check if complete and handle results if so
  this->ready[ReadyActiveSource] = (this->support.active_source != ActiveSourceUnknown);
  if (this->ready[ReadyActiveSource]) {
    ESP_LOGD(TAG, "Active source is %s", LOG_STR_ARG(active_source_strings[this->support.active_source]));
  }
}

/**
 * Select the source of the powerful flag
 */
void DaikinS21::check_ready_powerful_source() {
  if (this->readout_requests[ReadoutPowerful]) {
    const auto &special_modes = this->get_query(StateQuery::SpecialModes);  // enabled when requested and protocol >= 2
    if (special_modes.success()) {
      this->support.powerful_source = PowerfulSourceSpecialModes;
    } else if (special_modes.failed() || (this->protocol_version < ProtocolVersion(2))) {
      auto &unit_state = this->get_query(EnvironmentQuery::UnitState);
      if (unit_state.success()) {
        this->support.powerful_source = PowerfulSourceUnitState;
      } else if (unit_state.failed()) {
        this->support.powerful_source = PowerfulSourceDisabled;
      } else if (unit_state.enabled == false) {
        if (this->support.unit_system_state_queries) {
          this->readout_requests.set(ReadoutUnitStateBits);
          unit_state.enabled = true;
        } else {
          this->support.powerful_source = PowerfulSourceDisabled;
        }
      }
      if (this->support.powerful_source == PowerfulSourceDisabled) {
        this->support.unit_system_state_queries = false;  // unit state is assumed to be supported, if it isn't then correct the record
      }
    }
  } else {
    this->support.powerful_source = PowerfulSourceDisabled;
  }
  // check if complete and handle results if so
  this->ready[ReadyPowerfulSource] = (this->support.powerful_source != PowerfulSourceUnknown);
  if (this->ready[ReadyPowerfulSource]) {
    ESP_LOGD(TAG, "Powerful source is %s", LOG_STR_ARG(powerful_source_strings[this->support.powerful_source]));
  }
}

/**
 * Send a command with the accompanying payload
 */
void DaikinS21::send_command(const std::string_view command, const std::span<const uint8_t> payload) {
  this->current_command = command;
  this->serial.send_frame(this->current_command, payload);
}

/**
 * Perform the next action when the communication state is idle
 *
 * Select the next command to issue to the unit. If none, continue with the query cycle.
 *
 * Handle the results of the cycle if the last query was already issued:
 * - Process and report the results if the cycle was just completed
 * - Start the next cycle if free running or triggered in polling mode
 */
void DaikinS21::handle_serial_idle() {
  std::array<uint8_t, 4U> payload = {'0','0','0','0'};  // all command payloads here are 4 bytes long for now

  // Apply any pending settings
  const auto cycle_interval = this->get_cycle_interval_ms();
  if (this->climate.staged()) {
    payload[0] = (this->climate.pending.mode == climate::CLIMATE_MODE_OFF) ? '0' : '1'; // power
    payload[1] = climate_mode_to_s21(this->climate.pending.mode);
    if (this->climate.pending.setpoint == TEMPERATURE_INVALID) {
      payload[2] = 0x80;
    } else {
      payload[2] = (static_cast<int16_t>(this->climate.pending.setpoint) / 5) + 28;
    }
    payload[3] = fan_mode_to_s21(this->climate.pending.fan);
    this->send_command(StateCommand::PowerModeTempFan, payload);
    this->climate.set_confirm_ms(cycle_interval, 10*1000);  // longer timeout needed for mode changes
    return;
  }

  if (this->swing_humidity.staged()) {
    payload[0] = climate_swing_mode_to_s21(this->swing_humidity.pending.swing);
    if (this->swing_humidity.pending.swing != climate::CLIMATE_SWING_OFF) {
      payload[1] = '?';
    }
    payload[2] = humidity_mode_encodings[this->swing_humidity.pending.humidity];
    this->send_command(StateCommand::SwingHumidityModes, payload);
    this->swing_humidity.set_confirm_ms(cycle_interval);
    // keep vertical swing mode in sync
    this->vertical_swing_mode.pending = this->get_vertical_swing_mode();
    apply_swing_mode(this->swing_humidity.pending.swing, this->vertical_swing_mode.pending);
    this->vertical_swing_mode.set_confirm_ms(cycle_interval);
    return;
  }

  if (this->special_modes.staged()) {
    if (this->special_modes.value()[ModePowerful]) {
      payload[0] |= 0b00000010;
    }
    if (this->special_modes.value()[ModeComfort]) {
      payload[0] |= 0b01000000;
    }
    if (this->special_modes.value()[ModeQuiet]) {
      payload[0] |= 0b10000000;
    }
    if (this->special_modes.value()[ModeStreamer]) {
      payload[1] |= 0b10000000;
    }
    if (this->special_modes.value()[ModeSensorLED]) {
      payload[3] |= 0b00000100;
    }
    if (this->special_modes.value()[ModeMotionSensor]) {
      payload[3] |= 0b00001000;
    }
    this->send_command(StateCommand::SpecialModes, payload);
    this->special_modes.set_confirm_ms(cycle_interval);
    return;
  }

  if (this->demand_econo.staged()) {
    payload[0] += 100 - this->demand_econo.pending.demand;
    if (this->demand_econo.pending.econo) {
      payload[1] |= 0b00000010;
    }
    this->send_command(StateCommand::DemandAndEcono, payload);
    this->demand_econo.set_confirm_ms(cycle_interval);
    return;
  }

  if (this->vertical_swing_mode.staged()) {
    payload[0] = vertical_swing_mode_encodings[this->vertical_swing_mode.pending];
    this->send_command(StateCommand::VerticalSwingMode, payload);
    this->vertical_swing_mode.set_confirm_ms(cycle_interval);
    // keep regular swing mode state in sync
    this->swing_humidity.pending.swing = this->get_swing_mode();
    apply_vertical_swing_mode(this->vertical_swing_mode.pending, this->swing_humidity.pending.swing);
    this->swing_humidity.set_confirm_ms(cycle_interval);
    return;
  }

  // Periodic query cycle
  if (this->active_query < this->queries.end()) {
    this->serial.send_frame(this->active_query->command);  // query cycle underway, continue
    return;
  }

  // Query cycle complete
  const auto now = millis();
  this->cycle_active = false;
  this->cycle_time_ms = now - this->cycle_time_start_ms;
  if (this->is_ready() == false) {
    this->ready_state_machine();
  } else {
    // resolve action
    if (this->unit_state.defrost() && (this->action_reported == climate::CLIMATE_ACTION_HEATING)) {
      this->action = climate::CLIMATE_ACTION_COOLING; // report cooling during defrost
    } else if (this->active || (this->action_reported == climate::CLIMATE_ACTION_FAN)) {
      this->action = this->action_reported; // trust the unit when active or in fan only
    } else {
      this->action = climate::CLIMATE_ACTION_IDLE;
    }

    // resolve pending commands
    this->climate.check_confirm();
    this->swing_humidity.check_confirm();
    this->special_modes.check_confirm();
    this->demand_econo.check_confirm();
    this->vertical_swing_mode.check_confirm();

    // signal there's fresh data to consumers
    this->update_callbacks.call();
  }

  if (timestamp_passed(now, this->next_state_dump_ms)) {
    this->next_state_dump_ms = now + (60 * 1000); // every minute
    this->enable_loop_soon_any_context();  // dump state in foreground, blocks for too long here
  }

  // Start fresh polling query cycle (triggered never cleared in free run)
  if (this->cycle_triggered) {
    this->start_cycle();
  }
}

void DaikinS21::handle_state_basic(const std::span<const uint8_t> payload) {
  if (payload[0] == '0') {
    this->climate.active.mode = climate::CLIMATE_MODE_OFF;
    this->action_reported = climate::CLIMATE_ACTION_OFF;
  } else {
    this->climate.active.mode = s21_to_climate_mode(payload[1]);
    this->action_reported = s21_to_climate_action(payload[1]);
  }
  if (payload[2] == 0x80) {
    this->climate.active.setpoint = TEMPERATURE_INVALID;
  } else {
    this->climate.active.setpoint = (payload[2] - 28) * 5;  // Celsius * 10
  }
  // silent fan mode not reported here so prefer RG if present
  if (this->support.fan_mode_query == false) {
    this->climate.active.fan = s21_to_fan_mode(payload[3]);
  }
}

void DaikinS21::handle_state_error_status(const std::span<const uint8_t> payload) {
  this->serial_error = (payload[2] & 0b00010000);
}

void DaikinS21::handle_state_swing_humidity_modes(const std::span<const uint8_t> payload) {
  this->swing_humidity.active.swing = s21_to_climate_swing_mode(payload[0]);
  this->swing_humidity.active.humidity = s21_to_humidity_mode(payload[2]);
  // keep vertical swing mode in sync
  apply_swing_mode(this->swing_humidity.active.swing, this->vertical_swing_mode.active);
}

void DaikinS21::handle_state_special_modes(const std::span<const uint8_t> payload) {
  this->special_modes.active[ModePowerful] =     (payload[0] & 0b00000010);  // highest precedence, if this query is working there's no need to check powerful_source
  this->special_modes.active[ModeComfort] =      (payload[0] & 0b01000000);
  this->special_modes.active[ModeQuiet] =        (payload[0] & 0b10000000);
  this->special_modes.active[ModeStreamer] =     (payload[1] & 0b10000000);
  this->special_modes.active[ModeSensorLED] =    (payload[3] & 0b00000100);
  this->special_modes.active[ModeMotionSensor] = (payload[3] & 0b00001000);
}

void DaikinS21::handle_state_demand_and_econo(const std::span<const uint8_t> payload) {
  this->demand_econo.active.demand = 100 - (payload[0] - '0');
  this->demand_econo.active.econo = (payload[1] == '2');
}

/** Coarser than EnvironmentQuery::InsideTemperature and EnvironmentQuery::OutsideTemperature. Added if those queries fail. */
void DaikinS21::handle_state_inside_outside_temperature(const std::span<const uint8_t> payload) {
  if (this->support.inside_temperature_query == false) {
    this->temp_inside = (payload[0] - 128) * 5;  // 1 degree
  }
  if ((this->support.outside_temperature_query == false) && (payload[1] != 0xFF)) { // danijelt reports 0xFF when unsupported
    this->temp_outside = (payload[1] - 128) * 5; // 1 degree
  }
  if ((this->support.humidity_query == false) && ((payload[2] - '0') <= 100)) {  // Some units report 0xFF when unsupported
    this->humidity = payload[2] - '0';  // 5% granularity
  }
}

void DaikinS21::handle_state_model_code_v2(const std::span<const uint8_t> payload) {
  this->modelV2 = bytes_to_num(payload, 16);
}

void DaikinS21::handle_state_ir_counter(const std::span<const uint8_t> payload) {
  this->ir_counter = bytes_to_num(payload); // format unknown
}

void DaikinS21::handle_state_power_consumption(const std::span<const uint8_t> payload) {
  this->power_consumption = bytes_to_num(payload, 16);
}

void DaikinS21::handle_vertical_swing_mode(const std::span<const uint8_t> payload) {
  this->vertical_swing_mode.active = s21_to_vertical_swing_mode(payload[0]);
  // keep regular swing mode in sync
  apply_vertical_swing_mode(this->vertical_swing_mode.active, this->swing_humidity.active.swing);
}

void DaikinS21::handle_state_outdoor_capacity(const std::span<const uint8_t> payload) {
  this->outdoor_capacity = bytes_to_num(payload);
}

void DaikinS21::handle_state_model_name(std::span<const uint8_t> payload) {
  payload = payload.first(22);
  const auto end = std::ranges::find(payload, ' ');
  payload = { payload.begin(), end };
  std::ranges::copy(payload, this->model_name.begin());
  this->model_name[payload.size()] = 0;
}

void DaikinS21::handle_state_software_revision(std::span<const uint8_t> payload) {
  std::ranges::copy_n(payload.begin(), 8, this->software_revision.begin());
  this->software_revision[8] = 0;
}

void DaikinS21::handle_state_model_v3(std::span<const uint8_t> payload) {
  this->modelV3 = bytes_to_num(payload, 16);
}

/** Inferior to StateQuery::Basic */
void DaikinS21::handle_env_power_on_off(const std::span<const uint8_t> payload) {
  const bool active = payload[0] == '1';
}

/** Same info as StateQuery::Basic */
void DaikinS21::handle_env_indoor_unit_mode(const std::span<const uint8_t> payload) {
  if (payload[0] == '0') {
    this->climate.active.mode = climate::CLIMATE_MODE_OFF;
    this->action_reported = climate::CLIMATE_ACTION_OFF;
  } else {
    this->climate.active.mode = s21_to_climate_mode(payload[1]);
    this->action_reported = s21_to_climate_action(payload[1]);
  }
}

/** Same info as StateQuery::Basic */
void DaikinS21::handle_env_temperature_setpoint(const std::span<const uint8_t> payload) {
  this->climate.active.setpoint = bytes_to_num(payload) * 10;  // whole degrees C
}

/** Same info as StateQuery::SwingHumidityModes */
void DaikinS21::handle_env_swing_mode(const std::span<const uint8_t> payload) {
  this->swing_humidity.active.swing = s21_to_climate_swing_mode(payload[0]);
  // keep vertical swing mode in sync
  apply_swing_mode(this->swing_humidity.active.swing, this->vertical_swing_mode.active);
}

/** Better info than StateQuery::Basic (reports silent) */
void DaikinS21::handle_env_fan_mode(const std::span<const uint8_t> payload) {
  this->climate.active.fan = s21_to_fan_mode(payload[0]);
}

void DaikinS21::handle_env_inside_temperature(const std::span<const uint8_t> payload) {
  this->temp_inside = bytes_to_num(payload);
}

void DaikinS21::handle_env_liquid_temperature(const std::span<const uint8_t> payload) {
  this->temp_coil = bytes_to_num(payload);
}

void DaikinS21::handle_env_fan_speed_setpoint(const std::span<const uint8_t> payload) {
  this->fan_rpm_setpoint = bytes_to_num(payload) * 10;
}

void DaikinS21::handle_env_fan_speed(const std::span<const uint8_t> payload) {
  this->fan_rpm = bytes_to_num(payload) * 10;
}

void DaikinS21::handle_env_vertical_swing_angle_setpoint(const std::span<const uint8_t> payload) {
  this->swing_vertical_angle_setpoint = bytes_to_num(payload);
}

void DaikinS21::handle_env_vertical_swing_angle(const std::span<const uint8_t> payload) {
  this->swing_vertical_angle = bytes_to_num(payload);
}

void DaikinS21::handle_env_target_temperature(const std::span<const uint8_t> payload) {
  this->temp_target = bytes_to_num(payload); // Internal control loop target temperature
}

void DaikinS21::handle_env_outside_temperature(const std::span<const uint8_t> payload) {
  this->temp_outside = bytes_to_num(payload);
}

void DaikinS21::handle_env_indoor_frequency_command_signal(const std::span<const uint8_t> payload) {
  this->demand_pull = bytes_to_num(payload);  // Demand, 0-15
}

void DaikinS21::handle_env_compressor_frequency(const std::span<const uint8_t> payload) {
  this->compressor_rpm = bytes_to_num(payload) * 10;
  if (this->compressor_rpm == 9990) {
    this->compressor_rpm = 0;  // reported by danijelt
  }
}

void DaikinS21::handle_env_indoor_humidity(const std::span<const uint8_t> payload) {
  this->humidity = bytes_to_num(payload);
}

void DaikinS21::handle_env_compressor_on_off(const std::span<const uint8_t> payload) {
  this->active = (payload[0] == '1'); // highest precedence, if this query is working there's no need to check active_source
}

void DaikinS21::handle_env_unit_state(const std::span<const uint8_t> payload) {
  this->unit_state = bytes_to_num(payload, 16);
  if (this->support.active_source == ActiveSourceUnitState) {
    this->active = this->unit_state.active(); // used to refine climate action
  }
  if (this->support.powerful_source == PowerfulSourceUnitState) {
    this->special_modes.active[ModePowerful] = this->unit_state.powerful();  // if G6 is unsupported we can still read out powerful set by remote
  }
}

void DaikinS21::handle_env_system_state(const std::span<const uint8_t> payload) {
  this->system_state = bytes_to_num(payload, 16);
}

void DaikinS21::handle_misc_model_v0(const std::span<const uint8_t> payload) {
  this->modelV0 = bytes_to_num(payload, 16);
}

void DaikinS21::handle_misc_software_version(std::span<const uint8_t> payload) {
  // these rely on query string being included in the payload, as it is when
  // the calling code can't trim it off because it's an irregular response
  if ((payload.size() == 5) && (payload[0] == 'V')) {
    // protocol 0 returns VXXXX MiscQuery::Version response
    payload = payload.subspan(1);
  } else if ((payload.size() == 16) && (payload[0] == 'V') && (payload[1] == 'S')) {
    // protocol >=2 returns VSXXXXXXXXM00000 response
    payload = payload.subspan(2, 8);
  }
  if (payload.size() <= (this->software_version.size()-1)) {
    std::ranges::reverse_copy(payload, this->software_version.begin());
    this->software_version[payload.size()] = 0;
  }
}

void DaikinS21::handle_serial_result(const DaikinSerial::Result result, const std::span<const uint8_t> response /*= {}*/) {
  const bool is_query = this->current_command.empty();
  const std::string_view tx_str = is_query ? this->active_query->command : this->current_command;

  // Clip off the echoed query when it's echoed back. Most responses have a different leading character so ignore it.
  std::span<const uint8_t> payload{};
  if ((tx_str.size() <= response.size()) && std::equal(tx_str.begin() + 1, tx_str.end(), response.begin() + 1)) {
    payload = { response.begin() + tx_str.size(), response.end() };
  } else {
    payload = response; // Some queries have parameters that might not be echoed back, like VS000M -> V for protocol 0
  }

  // Add commands to this array to debug their output. empty string is just a placeholder to compile
  static constexpr std::array debug_commands{""};
  const bool is_debug = this->debug && (std::ranges::find(debug_commands, tx_str) != debug_commands.end());

  switch (result) {
    case DaikinSerial::Result::Ack:
        // debug logging
      if (/*this->debug ||*/ is_debug) {  // uncomment to debug all, not just debug_commands
        ESP_LOGD(TAG, "ACK: %" PRI_SV " -> %s %s",
                  PRI_SV_ARGS(tx_str),
                  str_repr(payload).c_str(),
                  hex_repr(payload).c_str());
      }
      if (is_query) {
        // print changed values
        if ((/*this->debug ||*/ is_debug) &&  // uncomment to debug all, not just debug_commands
            (std::ranges::equal(this->active_query->value(), payload) == false)) {
          ESP_LOGI(TAG, "%" PRI_SV " changed: %s %s -> %s %s",
                    PRI_SV_ARGS(this->active_query->command),
                    str_repr(this->active_query->value()).c_str(),
                    hex_repr(this->active_query->value()).c_str(),
                    str_repr(payload).c_str(),
                    hex_repr(payload).c_str());
        }
        // decode payload
        if ((this->active_query->response_length != 0) && (payload.size() != this->active_query->response_length)) {
          ESP_LOGW(TAG, "Unexpected payload length for %" PRI_SV " (%s)", this->active_query->command, hex_repr(payload).c_str());
          this->active_query->nak(payload);
        } else {
          if (this->active_query->handler == nullptr) {
            ESP_LOGI(TAG, "Unhandled command: %s", hex_repr(response).c_str());
          } else {
            std::invoke(this->active_query->handler, this, payload);
          }
          // save a copy of the payload
          this->active_query->ack(payload);
        }
      } else {
        // nothing yet to do when a command is accepted
      }
      break;

    case DaikinSerial::Result::Nak:
      ESP_LOGW(TAG, "NAK for %" PRI_SV, PRI_SV_ARGS(tx_str));
      if (is_query) {
        this->active_query->nak();
      } else {
        // nothing yet to do when a command is rejected
      }
      break;

    case DaikinSerial::Result::Timeout:
      ESP_LOGW(TAG, "Timeout waiting for response to %" PRI_SV, PRI_SV_ARGS(tx_str));
      if (is_query) {
        // It's possible some unsupported queries don't respond at all
        // Treat these as NAKs if we've established communication
        // Otherwise, a disconnected or unpowered HVAC unit will quickly cause all queries to fail
        if (this->ready[ReadyProtocolDetection]) {
          this->active_query->nak();
        }
      } else {
        // command will be retried, let's hope the code that generates it is bug free
      }
      break;

    case DaikinSerial::Result::Error:
      ESP_LOGE(TAG, "Error with %" PRI_SV, PRI_SV_ARGS(tx_str));
      break;
    default:
      break;
  }

  // update local state for next action
  if (result == DaikinSerial::Result::Error) {
    // something went terribly wrong, try to reinitialize communications
    this->climate.reset();
    this->swing_humidity.reset();
    this->special_modes.reset();
    this->demand_econo.reset();
    this->vertical_swing_mode.reset();
    this->current_command = {};
    this->start_cycle();
  } else {
    if (is_query) {
      // if communication established and all queries are disabled we had comms then they were lost
      if (this->ready[ReadyProtocolDetection] && (std::ranges::count_if(this->queries, DaikinQuery::IsEnabled) == 0)) {
        this->setup();  // reinitialize in order to prepare for the HVAC unit being reconnected
      } else {
        // advance to next query
        this->active_query = std::ranges::find_if(this->active_query + 1, this->queries.end(), DaikinQuery::IsEnabled);
      }
    } else {
      this->current_command = {};
    }
  }
}

void DaikinS21::dump_state() {
  ESP_LOGD(TAG, "Ready: %lX  Protocol: %" PRIu8 ".%" PRIu8 "  ModelV0: %04" PRIX16 "  ModelV2: %04" PRIX16 "  ModelV3: %04" PRIX16,
      this->ready.to_ulong(),
      this->protocol_version.major,
      this->protocol_version.minor,
      this->modelV0,
      this->modelV2,
      this->modelV3);
  if (this->debug) {
    const auto &old_proto = this->get_query(StateQuery::OldProtocol);
    const auto &new_proto = this->get_query(StateQuery::NewProtocol);
    const auto &misc_version = this->get_query(MiscQuery::Version);
    ESP_LOGD(TAG, " G8: %s  GY00: %s  Ver: %s  Rev: %s",
        str_repr(old_proto.value()).c_str(),
        str_repr(new_proto.value()).c_str(),
        this->software_version.data(),
        this->software_revision.data());
  }
  ESP_LOGD(TAG, "  Fan: %c  VSwing: %c  HSwing: %c  MI: %c  Humidify: %02" PRIX8,
      this->support.fan ? 'Y' : 'N',
      this->support.swing ? 'Y' : 'N',
      this->support.horiz_swing ? 'Y' : 'N',
      this->support.model_info,
      this->support.s_humd);
  ESP_LOGD(TAG, "  Dry: %c  Demand: %c  Powerful: %c  Econo: %c  Streamer: %c",
      this->support.dry ? 'Y' : 'N',
      this->support.demand ? 'Y' : 'N',
      this->support.powerful ? 'Y' : 'N',
      this->support.econo ? 'Y' : 'N',
      this->support.streamer ? 'Y' : 'N');
  if (this->debug) {
    const auto &v0_features = this->get_query(StateQuery::OptionalFeatures);
    const auto &v2_features = this->get_query(StateQuery::V2OptionalFeatures);
    const auto &v3_features = this->get_query(StateQuery::V3OptionalFeatures);
    auto v3_features_value = v3_features.value();
    if (v3_features_value.size() >= 5) {
      v3_features_value = v3_features_value.first(5);
    }
    ESP_LOGD(TAG, " G2: %s  GK: %s  GU00: %s  ActiveSrc: %s  PowerfulSrc: %s",
        (v0_features.success() ? hex_repr : str_repr)(v0_features.value()).c_str(),
        (v2_features.success() ? hex_repr : str_repr)(v2_features.value()).c_str(),
        (v3_features.success() ? hex_repr : str_repr)(v3_features_value).c_str(),
        LOG_STR_ARG(active_source_strings[this->support.active_source]),
        LOG_STR_ARG(powerful_source_strings[this->support.powerful_source]));
  }
  ESP_LOGD(TAG, "Mode: %s  Action: %s  Setpoint: %.1fC  Target: %.1fC  Inside: %.1fC  Coil: %.1fC",
      LOG_STR_ARG(climate::climate_mode_to_string(this->get_climate().mode)),
      LOG_STR_ARG(climate::climate_action_to_string(this->get_climate_action())),
      this->get_climate().setpoint.f_degc(),
      this->get_temp_target().f_degc(),
      this->get_temp_inside().f_degc(),
      this->get_temp_coil().f_degc());
  ESP_LOGD(TAG, "Cycle Time: %" PRIu32 "ms  UnitState: %" PRIX8 "  SysState: %02" PRIX8,
      this->cycle_time_ms,
      this->unit_state.raw,
      this->system_state.raw);
  if (this->debug) {
    const auto comma_join = [](auto&& queries) {
      std::string str;
      for (const auto &q : queries) {
        str += q;
        if (q != queries.back()) {
          str += ",";
        }
      }
      return str;
    };
    ESP_LOGD(TAG, "Enabled: %s", LOG_STR_ARG(comma_join(this->queries | std::views::filter(DaikinQuery::IsEnabled) | std::views::transform(DaikinQuery::GetCommand)).c_str()));
    ESP_LOGD(TAG, "  Nak'd: %s", LOG_STR_ARG(comma_join(this->queries | std::views::filter(DaikinQuery::IsFailed) | std::views::transform(DaikinQuery::GetCommand)).c_str()));
    ESP_LOGD(TAG, " Static: %s", LOG_STR_ARG(comma_join(this->queries | std::views::filter(DaikinQuery::IsAckedStatic) | std::views::transform(DaikinQuery::GetCommand)).c_str()));
  }
}

} // namespace esphome::daikin_s21
