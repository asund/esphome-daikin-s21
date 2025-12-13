#include <cinttypes>
#include <numeric>
#include <ranges>
#include "daikin_s21_queries.h"
#include "s21.h"
#include "utils.h"

using namespace esphome;

namespace esphome::daikin_s21 {

static const char *const TAG = "daikin_s21";

uint8_t climate_mode_to_daikin(const climate::ClimateMode mode) {
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

climate::ClimateMode daikin_to_climate_mode(const uint8_t mode) {
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

climate::ClimateAction daikin_to_climate_action(const uint8_t action) {
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

climate::ClimateSwingMode daikin_to_climate_swing_mode(const uint8_t mode) {
  switch (mode) {
    case '1':
      return climate::CLIMATE_SWING_VERTICAL;
    case '2':
      return climate::CLIMATE_SWING_HORIZONTAL;
    case '7':
      return climate::CLIMATE_SWING_BOTH;
    default:
      return climate::CLIMATE_SWING_OFF;
  }
}

uint8_t climate_swing_mode_to_daikin(const climate::ClimateSwingMode mode) {
  switch (mode) {
    case climate::CLIMATE_SWING_BOTH:
      return '7';
    case climate::CLIMATE_SWING_VERTICAL:
      return '1';
    case climate::CLIMATE_SWING_HORIZONTAL:
      return '2';
    case climate::CLIMATE_SWING_OFF:
    default:
      return '0';
  }
}

const char * active_source_to_string(const ActiveSource source) {
  switch (source) {
    case ActiveSource::CompressorOnOff:
      return "Rg";
    case ActiveSource::UnitState:
      return "unit state";
    case ActiveSource::Unsupported:
      return "assumed on";
    case ActiveSource::Unknown:
    default:
      return "undetected";
  }
}

const char * powerful_source_to_string(const PowerfulSource source) {
  switch (source) {
    case PowerfulSource::SpecialModes:
      return "G6";
    case PowerfulSource::UnitState:
      return "unit state";
    case PowerfulSource::Disabled:
      return "disabled";
    case PowerfulSource::Unknown:
    default:
      return "undetected";
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
    {StateQuery::Basic, &DaikinS21::handle_state_basic},
    {StateQuery::OptionalFeatures, &DaikinS21::handle_nop, true},
    // {StateQuery::OnOffTimer},  // unused, use home assistant for scheduling
    {StateQuery::ErrorStatus, &DaikinS21::handle_state_error_status},
    {StateQuery::SwingOrHumidity, &DaikinS21::handle_state_swing_or_humidity},
    {StateQuery::SpecialModes, &DaikinS21::handle_state_special_modes},
    {StateQuery::DemandAndEcono, &DaikinS21::handle_state_demand_and_econo},
    {StateQuery::OldProtocol, &DaikinS21::handle_nop, true},  // protocol version detect
    {StateQuery::InsideOutsideTemperatures, &DaikinS21::handle_state_inside_outside_temperature},
    // {StateQuery::FB, &DaikinS21::handle_nop}, // unknown
    {StateQuery::ModelCode, &DaikinS21::handle_state_model_code_v2, true},
    {StateQuery::IRCounter, &DaikinS21::handle_state_ir_counter},
    {StateQuery::V2OptionalFeatures, &DaikinS21::handle_nop, true},
    {StateQuery::PowerConsumption, &DaikinS21::handle_state_power_consumption},
    // {StateQuery::ITELC, &DaikinS21::handle_nop},  // unknown, daikin intelligent touch controller?
    // {StateQuery::FP, &DaikinS21::handle_nop}, // unknown
    // {StateQuery::FQ, &DaikinS21::handle_nop}, // unknown
    // {StateQuery::FS, &DaikinS21::handle_nop}, // unknown
    {StateQuery::OutdoorCapacity, &DaikinS21::handle_state_outdoor_capacity}, // unconfirmed, outdoor unit capacity?
    {StateQuery::V3OptionalFeatures, &DaikinS21::handle_nop, true},
    // {StateQuery::FV, &DaikinS21::handle_nop}, // unknown
    {StateQuery::NewProtocol, &DaikinS21::handle_nop, true},  // protocol version detect
    // {EnvironmentQuery::PowerOnOff, &DaikinS21::handle_env_power_on_off}, // redundant
    // {EnvironmentQuery::IndoorUnitMode, &DaikinS21::handle_env_indoor_unit_mode}, // redundant
    // {EnvironmentQuery::TemperatureSetpoint, &DaikinS21::handle_env_temperature_setpoint},  // redundant
    // {EnvironmentQuery::OnTimerSetting}, // unused, unsupported
    // {EnvironmentQuery::OffTimerSetting},  // unused, unsupported
    // {EnvironmentQuery::SwingMode, &DaikinS21::handle_env_swing_mode},  // redundant
    {EnvironmentQuery::FanMode, &DaikinS21::handle_env_fan_mode},
    {EnvironmentQuery::InsideTemperature, &DaikinS21::handle_env_inside_temperature},
    {EnvironmentQuery::LiquidTemperature, &DaikinS21::handle_env_liquid_temperature},
    // {EnvironmentQuery::FanSpeedSetpoint, &DaikinS21::handle_env_fan_speed_setpoint},  // not supported yet, can translate DaikinFanMode to RPM
    {EnvironmentQuery::FanSpeed, &DaikinS21::handle_env_fan_speed},
    // {EnvironmentQuery::LouvreAngleSetpoint, &DaikinS21::handle_env_vertical_swing_angle_setpoint},  // not supported yet
    {EnvironmentQuery::VerticalSwingAngle, &DaikinS21::handle_env_vertical_swing_angle},
    // {EnvironmentQuery::RW, &DaikinS21::handle_nop},  // unknown, "00" for me
    {EnvironmentQuery::TargetTemperature, &DaikinS21::handle_env_target_temperature},
    {EnvironmentQuery::OutsideTemperature, &DaikinS21::handle_env_outside_temperature},
    {EnvironmentQuery::IndoorFrequencyCommandSignal, &DaikinS21::handle_env_indoor_frequency_command_signal},
    {EnvironmentQuery::CompressorFrequency, &DaikinS21::handle_env_compressor_frequency},
    {EnvironmentQuery::IndoorHumidity, &DaikinS21::handle_env_indoor_humidity},
    {EnvironmentQuery::CompressorOnOff, &DaikinS21::handle_env_compressor_on_off},
    // {EnvironmentQuery::CompressorOnOff}, // redundant
    {EnvironmentQuery::UnitState, &DaikinS21::handle_env_unit_state},
    {EnvironmentQuery::SystemState, &DaikinS21::handle_env_system_state},
    // {EnvironmentQuery::Rz52, &DaikinS21::handle_nop},  // unknown, "40"for me
    // {EnvironmentQuery::Rz72, &DaikinS21::handle_nop},  // unknown, "23" for me
    {MiscQuery::Model, &DaikinS21::handle_misc_model_v0, true}, // some sort of model? always "3E53" for me, regardless of head unit -- outdoor unit?
    {MiscQuery::Version, &DaikinS21::handle_nop, true}, // purportedly another version, always "00C0" for me
    {MiscQuery::SoftwareVersion, &DaikinS21::handle_misc_software_version, true},
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
void DaikinS21::set_climate_settings(const DaikinClimateSettings &settings) {
  if ((this->pending.climate.mode != settings.mode) ||
      (this->pending.climate.setpoint != settings.setpoint) ||
      (this->pending.climate.fan != settings.fan)) {
    this->pending.activate_climate = true;
    this->trigger_cycle();
  }

  if (this->pending.climate.swing != settings.swing) {
    this->pending.activate_swing_mode = true;
    this->trigger_cycle();
  }

  if (this->pending.climate.preset != settings.preset) {
    this->pending.activate_preset = true;
    this->trigger_cycle();
  }

  this->pending.climate = settings;
}

/**
 * Set the enable value of the specified mode and trigger a write to the unit if it changed.
 */
void DaikinS21::set_mode(const DaikinMode mode, const bool enable) {
  if (mode < DaikinModeCount) {
    if (this->pending.modes[mode] != enable) {
      this->pending.modes[mode] = enable;
      this->pending.activate_modes[mode] = true;
      this->trigger_cycle();
    }
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
  if (mode < DaikinModeCount) {
    return this->current.modes[mode];
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
    start_cycle();
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
    this->pending.climate = this->current.climate;
    this->pending.modes = this->current.modes;

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
  static constexpr uint8_t new_version_unsupported[] = {'G',0,0,0,0,0}; // can be returned when unsupported, query string included in the payload
  const auto &old_proto = this->get_query(StateQuery::OldProtocol);
  const auto &new_proto = this->get_query(StateQuery::NewProtocol);
  const bool new_proto_red_herring = new_proto.success() && std::ranges::equal(new_proto.value(), new_version_unsupported);

  this->protocol_version = ProtocolUndetected;
  // Check availability first, both protocol indicators were enabled on init so skip to the chase
  if (old_proto.success() && (new_proto.failed() || new_proto_red_herring)) {
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
    this->enable_query(StateQuery::ErrorStatus);
    this->enable_query(StateQuery::SwingOrHumidity);
    this->enable_query(EnvironmentQuery::CompressorOnOff);
    this->enable_query(MiscQuery::SoftwareVersion);
    if (this->protocol_version <= ProtocolVersion(2)) {
      this->enable_query(MiscQuery::Model);
      this->enable_query(MiscQuery::Version);
    }
    if (this->protocol_version >= ProtocolVersion(2)) {
      if (this->readout_requests[ReadoutSpecialModes]) {
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
      if (this->readout_requests[ReadoutOutdoorCapacity]) {
        this->enable_query(StateQuery::OutdoorCapacity);
      }
    }
    if (this->protocol_version >= ProtocolVersion(3)) {
      this->enable_query(StateQuery::V3OptionalFeatures);
      // todo many more
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
      this->support.humidify =      (v0_features[3] & 0b0010);
      this->support.dehumidify =    (v0_features[3] & 0b1000);
    }
    if (v2_features.success()) {
      this->support.ac_led =        (v2_features[0] & 0b00000001);
      this->support.laundry =       (v2_features[0] & 0b00001000);
      this->support.elec =          (v2_features[1] & 0b00000001);
      this->support.temp_range =    (v2_features[1] & 0b00000100);
      this->support.motion_detect = (v2_features[1] & 0b00001000);
      this->support.ac_japan =      (v2_features[2] & 0b00000001) == 0;
      if ((v2_features[2] & 0b00000010)) { // v2 gates v0
        // "simple humidify mode available", unknown what this means to us
        this->support.humidify =    false;
        this->support.dehumidify =  false;
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

    if (this->support.fan) {
      this->enable_query(EnvironmentQuery::FanMode);
      if (this->readout_requests[ReadoutFanSpeed]) {
        this->enable_query(EnvironmentQuery::FanSpeed);
      }
    }
    if (this->support.swing && this->readout_requests[ReadoutSwingAngle]) {
      this->enable_query(EnvironmentQuery::VerticalSwingAngle);
    }
    if (this->support.humidify || this->support.dehumidify) {
      // unknown if this is (de)humidify mode or humidity sensor, potentially hides the sensor if it's just the mode
      if (this->readout_requests[ReadoutHumidity]) {
        this->enable_query(EnvironmentQuery::IndoorHumidity);
      }
    }

    // todo climate traits and sensor config check -- let user know they can pare down their yaml
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
                    (this->readout_requests[ReadoutHumidity] && (this->support.humidify || this->support.dehumidify) && humidity.failed()); // only enable for humidity's sake if declared to be supported
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
  const auto &compressor_on_off = this->get_query(EnvironmentQuery::CompressorOnOff);
  if (compressor_on_off.success()) {
    this->support.active_source = ActiveSource::CompressorOnOff;
  } else if (compressor_on_off.failed()) {
    auto &unit_state = this->get_query(EnvironmentQuery::UnitState);
    if (unit_state.success()) {
      this->support.active_source = ActiveSource::UnitState;
    } else if (unit_state.failed()) {
      this->support.active_source = ActiveSource::Unsupported;
    } else if (unit_state.enabled == false) {
      if (this->support.unit_system_state_queries) {
        this->readout_requests.set(ReadoutUnitStateBits);
        unit_state.enabled = true;
      } else {
        this->support.active_source = ActiveSource::Unsupported;
      }
    }
    if (this->support.active_source == ActiveSource::Unsupported) {
      this->support.unit_system_state_queries = false;  // unit state is assumed to be supported, if it isn't then correct the record
      this->current.active = true;  // always active, reported action will follow unit
    }
  }
  // check if complete and handle results if so
  this->ready[ReadyActiveSource] = (this->support.active_source != ActiveSource::Unknown);
  if (this->ready[ReadyActiveSource]) {
    ESP_LOGD(TAG, "Active source is %s", active_source_to_string(this->support.active_source));
  }
}

/**
 * Select the source of the powerful flag
 */
void DaikinS21::check_ready_powerful_source() {
  if (this->readout_requests[ReadoutSpecialModes]) {
    const auto &special_modes = this->get_query(StateQuery::SpecialModes);
    if (special_modes.ready()) {
      if (special_modes.success()) {
        this->support.powerful_source = PowerfulSource::SpecialModes;
      } else {
        auto &unit_state = this->get_query(EnvironmentQuery::UnitState);
        if (unit_state.success()) {
          this->support.powerful_source = PowerfulSource::UnitState;
        } else if (unit_state.failed()) {
          this->support.powerful_source = PowerfulSource::Disabled;
        } else if (unit_state.enabled == false) {
          if (this->support.unit_system_state_queries) {
            this->readout_requests.set(ReadoutUnitStateBits);
            unit_state.enabled = true;
          } else {
            this->support.powerful_source = PowerfulSource::Disabled;
          }
        }
        if (this->support.powerful_source == PowerfulSource::Disabled) {
          this->support.unit_system_state_queries = false;  // unit state is assumed to be supported, if it isn't then correct the record
        }
      }
    }
  } else {
    this->support.powerful_source = PowerfulSource::Disabled;
  }
  // check if complete and handle results if so
  this->ready[ReadyPowerfulSource] = (this->support.powerful_source != PowerfulSource::Unknown);
  if (this->ready[ReadyPowerfulSource]) {
    ESP_LOGD(TAG, "Powerful source is %s", powerful_source_to_string(this->support.powerful_source));
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
  // Important to clear the activate flag here as another command can be queued while waiting for this one to complete
  if (this->pending.activate_climate) {
    this->pending.activate_climate = false;
    payload[0] = (this->pending.climate.mode == climate::CLIMATE_MODE_OFF) ? '0' : '1'; // power
    payload[1] = climate_mode_to_daikin(this->pending.climate.mode);
    payload[2] = (static_cast<int16_t>(this->pending.climate.setpoint) / 5) + 28;
    payload[3] = static_cast<char>(this->pending.climate.fan);
    this->send_command(StateCommand::PowerModeTempFan, payload);
    return;
  }

  if (this->pending.activate_swing_mode) {
    this->pending.activate_swing_mode = false;
    payload[0] = climate_swing_mode_to_daikin(this->pending.climate.swing);
    if (this->pending.climate.swing != climate::CLIMATE_SWING_OFF) {
      payload[1] = '?';
    }
    this->send_command(StateCommand::LouvreSwingMode, payload);
    return;
  }

  if (this->pending.activate_modes.any()) {
    // value to send is the pending value of any modes being activated and the current value of any modes not being activated
    const std::bitset<DaikinModeCount> send_value = (this->pending.activate_modes & this->pending.modes) | (~this->pending.activate_modes & this->current.modes);
    if (this->pending.activate_modes[ModeEcono] == false) {
      if (send_value[ModePowerful]) {
        payload[0] |= 0b00000010;
      }
      if (send_value[ModeComfort]) {
        payload[0] |= 0b01000000;
      }
      if (send_value[ModeQuiet]) {
        payload[0] |= 0b10000000;
      }
      if (send_value[ModeStreamer]) {
        payload[1] |= 0b10000000;
      }
      if (send_value[ModeMotionSensor]) {
        payload[3] |= 0b10000000;
      }
      this->send_command(StateCommand::SpecialModes, payload);
      this->pending.activate_modes &= std::bitset<DaikinModeCount>().set(ModeEcono);
    } else {
      if (send_value[ModeEcono]) {
        payload[1] |= 0b00000010;
      }
      this->send_command(StateCommand::DemandAndEcono, payload);
      this->pending.activate_modes.reset(ModeEcono);
    }
    return;
  }

  if (this->pending.activate_preset) {
    // potentially a two stage operation -- first disabling the old, then enabling the new
    climate::ClimatePreset preset;
    bool enable;
    if ((this->current.climate.preset != this->pending.climate.preset) && (this->current.climate.preset != climate::CLIMATE_PRESET_NONE)) {
      preset = this->current.climate.preset;
      enable = false;
      if (this->pending.climate.preset == climate::CLIMATE_PRESET_NONE) {
        this->pending.activate_preset = false;  // don't execute again if we're just disabling the current preset
      }
    } else {
      preset = this->pending.climate.preset;
      enable = true;
      this->pending.activate_preset = false;  // don't execute again if we're setting the desired preset
    }
    switch (preset) {
      case climate::CLIMATE_PRESET_BOOST:
        payload[0] = enable ? '2' : '0';
        payload[1] = '0';
        payload[2] = '0';
        payload[3] = '0';
        this->send_command(StateCommand::SpecialModes, payload);
        return;
      case climate::CLIMATE_PRESET_ECO:
        payload[0] = '0';
        payload[1] = enable ? '2' : '0';
        payload[2] = '0';
        payload[3] = '0';
        this->send_command(StateCommand::DemandAndEcono, payload);
        return;
      case climate::CLIMATE_PRESET_NONE:
      default:
        break;
    }
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
    if (this->current.unit_state.defrost() && (this->current.action_reported == climate::CLIMATE_ACTION_HEATING)) {
      this->current.action = climate::CLIMATE_ACTION_COOLING; // report cooling during defrost
    } else if (this->current.active || (this->current.action_reported == climate::CLIMATE_ACTION_FAN)) {
      this->current.action = this->current.action_reported; // trust the unit when active or in fan only
    } else {
      this->current.action = climate::CLIMATE_ACTION_IDLE;
    }
    // resolve presets
    if (this->current.modes[ModePowerful]) {
      this->current.climate.preset = climate::CLIMATE_PRESET_BOOST;
    } else if (this->current.modes[ModeEcono]) {
      this->current.climate.preset = climate::CLIMATE_PRESET_ECO;
    } else {
      this->current.climate.preset = climate::CLIMATE_PRESET_NONE;
    }
    // signal there's fresh data to consumers
    this->update_callbacks.call();
  }

  if ((now - last_state_dump_ms) > (60 * 1000)) { // every minute
    last_state_dump_ms = now;
    this->enable_loop_soon_any_context();  // dump state in foreground, blocks for too long here
  }

  // Start fresh polling query cycle (triggered never cleared in free run)
  if (this->cycle_triggered) {
    this->start_cycle();
  }
}

void DaikinS21::handle_state_basic(const std::span<const uint8_t> payload) {
  if (payload[0] == '0') {
    this->current.climate.mode = climate::CLIMATE_MODE_OFF;
    this->current.action_reported = climate::CLIMATE_ACTION_OFF;
  } else {
    this->current.climate.mode = daikin_to_climate_mode(payload[1]);
    this->current.action_reported = daikin_to_climate_action(payload[1]);
  }
  this->current.climate.setpoint = (payload[2] - 28) * 5;  // Celsius * 10
  // silent fan mode not reported here so prefer RG if present
  if (this->support.fan_mode_query == false) {
    this->current.climate.fan = static_cast<daikin_s21::DaikinFanMode>(payload[3]);
  }
}

void DaikinS21::handle_state_error_status(const std::span<const uint8_t> payload) {
  this->current.serial_error = (payload[2] & 0b00010000);
}

void DaikinS21::handle_state_swing_or_humidity(const std::span<const uint8_t> payload) {
  this->current.climate.swing = daikin_to_climate_swing_mode(payload[0]);
}

void DaikinS21::handle_state_special_modes(const std::span<const uint8_t> payload) {
  this->current.modes[ModePowerful] =     (payload[0] & 0b00000010);  // highest precedence, if this query is working there's no need to check powerful_source
  this->current.modes[ModeComfort] =      (payload[0] & 0b01000000);
  this->current.modes[ModeQuiet] =        (payload[0] & 0b10000000);
  this->current.modes[ModeStreamer] =     (payload[1] & 0b10000000);
  this->current.modes[ModeMotionSensor] = (payload[3] & 0b00001000);
  this->current.sensor_led =              (payload[3] & 0b00001100) != 0b00001100;
}

void DaikinS21::handle_state_demand_and_econo(const std::span<const uint8_t> payload) {
  this->current.modes[ModeEcono] =        (payload[1] == '2');
}

/** Coarser than EnvironmentQuery::InsideTemperature and EnvironmentQuery::OutsideTemperature. Added if those queries fail. */
void DaikinS21::handle_state_inside_outside_temperature(const std::span<const uint8_t> payload) {
  if (this->support.inside_temperature_query == false) {
    this->temp_inside = (payload[0] - 128) * 5;  // 1 degree
  }
  if ((this->support.outside_temperature_query == false) && (payload[1] != 0xFF)) { // danijelt reports 0xFF when unsupported
    this->temp_outside = (payload[1] - 128) * 5; // 1 degree
  }
  if ((this->support.humidity_query == false) && (this->support.humidify || this->support.dehumidify) && ((payload[2] - '0') <= 100)) {  // Some units report 0xFF when unsupported
    this->humidity = payload[2] - '0';  // 5% granularity
  }
}

void DaikinS21::handle_state_model_code_v2(const std::span<const uint8_t> payload) {
  this->modelV2 = bytes_to_num(payload, 16);
}

void DaikinS21::handle_state_ir_counter(const std::span<const uint8_t> payload) {
  this->current.ir_counter = bytes_to_num(payload); // format unknown
}

void DaikinS21::handle_state_power_consumption(const std::span<const uint8_t> payload) {
  this->current.power_consumption = bytes_to_num(payload, 16);
}

void DaikinS21::handle_state_outdoor_capacity(const std::span<const uint8_t> payload) {
  this->current.outdoor_capacity = bytes_to_num(payload);
}

/** Vastly inferior to StateQuery::Basic */
void DaikinS21::handle_env_power_on_off(const std::span<const uint8_t> payload) {
  const bool active = payload[0] == '1';
}

/** Same info as StateQuery::Basic */
void DaikinS21::handle_env_indoor_unit_mode(const std::span<const uint8_t> payload) {
  if (payload[0] == '0') {
    this->current.climate.mode = climate::CLIMATE_MODE_OFF;
    this->current.action_reported = climate::CLIMATE_ACTION_OFF;
  } else {
    this->current.climate.mode = daikin_to_climate_mode(payload[1]);
    this->current.action_reported = daikin_to_climate_action(payload[1]);
  }
}

/** Same info as StateQuery::Basic */
void DaikinS21::handle_env_temperature_setpoint(const std::span<const uint8_t> payload) {
  this->current.climate.setpoint = bytes_to_num(payload) * 10;  // whole degrees C
}

/** Same info as StateQuery::SwingOrHumidity */
void DaikinS21::handle_env_swing_mode(const std::span<const uint8_t> payload) {
  this->current.climate.swing = daikin_to_climate_swing_mode(payload[0]);
}

/** Better info than StateQuery::Basic (reports silent) */
void DaikinS21::handle_env_fan_mode(const std::span<const uint8_t> payload) {
  this->current.climate.fan = static_cast<daikin_s21::DaikinFanMode>(payload[0]);
}

void DaikinS21::handle_env_inside_temperature(const std::span<const uint8_t> payload) {
  this->temp_inside = bytes_to_num(payload);
}

void DaikinS21::handle_env_liquid_temperature(const std::span<const uint8_t> payload) {
  this->temp_coil = bytes_to_num(payload);
}

void DaikinS21::handle_env_fan_speed_setpoint(const std::span<const uint8_t> payload) {
  this->current.fan_rpm_setpoint = bytes_to_num(payload) * 10;
}

void DaikinS21::handle_env_fan_speed(const std::span<const uint8_t> payload) {
  this->current.fan_rpm = bytes_to_num(payload) * 10;
}

void DaikinS21::handle_env_vertical_swing_angle_setpoint(const std::span<const uint8_t> payload) {
  this->current.swing_vertical_angle_setpoint = bytes_to_num(payload);
}

void DaikinS21::handle_env_vertical_swing_angle(const std::span<const uint8_t> payload) {
  this->current.swing_vertical_angle = bytes_to_num(payload);
}

void DaikinS21::handle_env_target_temperature(const std::span<const uint8_t> payload) {
  this->temp_target = bytes_to_num(payload); // Internal control loop target temperature
}

void DaikinS21::handle_env_outside_temperature(const std::span<const uint8_t> payload) {
  this->temp_outside = bytes_to_num(payload);
}

void DaikinS21::handle_env_indoor_frequency_command_signal(const std::span<const uint8_t> payload) {
  this->demand = bytes_to_num(payload);  // Demand, 0-15
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
  this->current.active = (payload[0] == '1'); // highest precedence, if this query is working there's no need to check active_source
}

void DaikinS21::handle_env_unit_state(const std::span<const uint8_t> payload) {
  this->current.unit_state = bytes_to_num(payload, 16);
  if (this->support.active_source == ActiveSource::UnitState) {
    this->current.active = this->current.unit_state.active(); // used to refine climate action
  }
  if (this->support.powerful_source == PowerfulSource::UnitState) {
    this->current.modes[ModePowerful] = this->current.unit_state.powerful();  // if G6 is unsupported we can still read out powerful set by remote
  }
}

void DaikinS21::handle_env_system_state(const std::span<const uint8_t> payload) {
  this->current.system_state = bytes_to_num(payload, 16);
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
        if (this->active_query->handler != nullptr) {
          std::invoke(this->active_query->handler, this, payload);
        } else {
          ESP_LOGD(TAG, "Unhandled command: %s", hex_repr(response).c_str());
        }
        // save a copy of the payload
        this->active_query->ack(payload);
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
    this->pending.activate_climate = false;
    this->pending.activate_swing_mode = false;
    this->pending.activate_preset = false;
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
  ESP_LOGD(TAG, "Ready: 0x%lX  Protocol: %" PRIu8 ".%" PRIu8 "  ModelV0: %04" PRIX16 "  ModelV2: %04" PRIX16,
      this->ready.to_ulong(),
      this->protocol_version.major,
      this->protocol_version.minor,
      this->modelV0,
      this->modelV2);
  if (this->debug) {
    const auto &old_proto = this->get_query(StateQuery::OldProtocol);
    const auto &new_proto = this->get_query(StateQuery::NewProtocol);
    const auto &misc_version = this->get_query(MiscQuery::Version);
    ESP_LOGD(TAG, " G8: %s  GY00: %s  Ver: %s",
        str_repr(old_proto.value()).c_str(),
        str_repr(new_proto.value()).c_str(),
        this->software_version.data());
  }
  ESP_LOGD(TAG, "  Fan: %c  VSwing: %c  HSwing: %c  MI: %c  Humid: %c  Dehumid: %c",
      this->support.fan ? 'Y' : 'N',
      this->support.swing ? 'Y' : 'N',
      this->support.horiz_swing ? 'Y' : 'N',
      this->support.model_info,
      this->support.humidify ? 'Y' : 'N',
      this->support.dehumidify ? 'Y' : 'N');
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
      v3_features_value = v3_features_value.subspan(0,5);
    }
    ESP_LOGD(TAG, " G2: %s  GK: %s  GU00: %s  ActiveSrc: %s  PowerfulSrc: %s",
        (v0_features.success() ? hex_repr : str_repr)(v0_features.value()).c_str(),
        (v2_features.success() ? hex_repr : str_repr)(v2_features.value()).c_str(),
        (v3_features.success() ? hex_repr : str_repr)(v3_features_value).c_str(),
        active_source_to_string(this->support.active_source),
        powerful_source_to_string(this->support.powerful_source));
  }
  ESP_LOGD(TAG, "Mode: %s  Action: %s  Preset: %s  Demand: %" PRIu8,
      LOG_STR_ARG(climate::climate_mode_to_string(this->current.climate.mode)),
      LOG_STR_ARG(climate::climate_action_to_string(this->get_climate_action())),
      LOG_STR_ARG(climate::climate_preset_to_string(this->current.climate.preset)),
      this->get_demand());
  if (this->support.fan || this->support.swing) {
    ESP_LOGD(TAG, "Fan: %s (%" PRIu16 " RPM)  Swing: %s",
        LOG_STR_ARG(daikin_fan_mode_to_cstr(this->current.climate.fan)),
        this->current.fan_rpm,
        this->support.swing ? LOG_STR_ARG(climate::climate_swing_mode_to_string(this->current.climate.swing)) : "N/A");
  }
  ESP_LOGD(TAG, "Setpoint: %.1fC  Target: %.1fC  Inside: %.1fC  Coil: %.1fC",
      this->get_temp_setpoint().f_degc(),
      this->get_temp_target().f_degc(),
      this->get_temp_inside().f_degc(),
      this->get_temp_coil().f_degc());
  if (this->support.humidify || this->support.dehumidify) {
    ESP_LOGD(TAG, "Humid: %" PRIu8 "%%", this->get_humidity());
  }
  ESP_LOGD(TAG, "Cycle Time: %" PRIu32 "ms  UnitState: %" PRIX8 "  SysState: %02" PRIX8,
      this->cycle_time_ms,
      this->current.unit_state.raw,
      this->current.system_state.raw);
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
