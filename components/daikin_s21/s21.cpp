#include <cinttypes>
#include <numeric>
#include <ranges>
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

int16_t bytes_to_num(std::span<const uint8_t> bytes) {
  // <ones><tens><hundreds[><neg/pos>,<thousands>]
  int16_t val = 0;
  val = bytes[0] - '0';
  val += (bytes[1] - '0') * 10;
  val += (bytes[2] - '0') * 100;
  if (bytes.size() > 3) {
    if (bytes[3] == '-') {
      val *= -1;
    } else if (std::isdigit(bytes[3])) {
      val += (bytes[3] - '0') * 1000;
    }
  }
  return val;
}

void DaikinS21::setup() {
  // populate initial messages to poll
  // clang-format off
  this->queries = {
      // Protocol version detect
      {StateQuery::OldProtocol, &DaikinS21::handle_nop, true},
      {StateQuery::NewProtocol, &DaikinS21::handle_nop, true},
  };
  // clang-format on
  this->failed_queries = {};
  this->static_queries = {};
  this->protocol_version = ProtocolUndetected;
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
}

void DaikinS21::set_climate_settings(const DaikinClimateSettings &settings) {
  if ((this->pending.climate.mode != settings.mode) ||
      (this->pending.climate.setpoint != settings.setpoint) ||
      (this->pending.climate.fan != settings.fan)) {

    // If this is the first time settings are being applied, we need to force
    // all of them to be applied to the unit so our state will be in sync with
    // the unit and thus rely on this change detection to work in the future.
    // The pending settings mode has been initialized in the object constructor
    // to the unsupported (by this project) CLIMATE_MODE_AUTO in order to detect
    // this one time condition here.
    if (this->pending.climate.mode == climate::CLIMATE_MODE_AUTO) {
      this->pending.activate_swing_mode = true;
      this->pending.activate_preset = true;
    }

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
  this->query_index = 0;
  this->serial.send_frame(this->current_query()->command);
}

/**
 * Get the result of a query.
 */
DaikinQueryResult DaikinS21::get_query_result(std::string_view query_str) const {
  // static and therefore ack'd once
  const auto iter_static = std::ranges::find(this->static_queries, query_str, DaikinQueryState::GetCommand);
  if (iter_static != this->static_queries.end()) {
    return { iter_static->value(), true, false };
  }
  // failed and therefore nak'd
  const auto iter_nak = std::ranges::find(this->failed_queries, query_str);
  if (iter_nak != this->failed_queries.end()) {
    static constexpr uint8_t nak_str[] = {'N','A','K'};
    return { nak_str, false, true };
  }
  // active and ack'd
  const auto iter_active = std::ranges::find(this->queries, query_str, DaikinQueryState::GetCommand);
  if (iter_active != this->queries.end()) {
    return { iter_active->value(), iter_active->acked, false };
  }
  // never scheduled, let handler code treat it as nak'd
  return QueryNotScheduled;
}

/**
 * Remove a query from the active pool.
 */
void DaikinS21::prune_query(std::string_view query_str) {
  const auto query = std::ranges::find(this->queries, query_str, DaikinQueryState::GetCommand);
  if (query != this->queries.end()) {
    // adjust index if deleting
    const auto erased_index = std::distance(this->queries.begin(), query);
    this->queries.erase(query);
    if (queries.empty()) {
      this->query_index = 0;
    } else if (this->query_index > erased_index) {
      this->query_index--;
    }
  }
}

/**
 * Refine the pool of polling queries, adding or removing them as we learn about the unit.
 */
void DaikinS21::refine_queries() {
  if (this->comms_detected() == false) {
    if (this->determine_protocol_version() == false) {
      return; // wait for detection
    }

    ESP_LOGD(TAG, "Protocol version %" PRIu8 ".%" PRIu8 " detected", this->protocol_version.major, this->protocol_version.minor);
    // >= Protocol0
    this->queries.insert(this->queries.end(), {
      {StateQuery::Basic, &DaikinS21::handle_state_basic},
      {StateQuery::OptionalFeatures, &DaikinS21::handle_nop, true},
      // {StateQuery::OnOffTimer},  // unused, use home assistant for scheduling
      // {StateQuery::ErrorStatus}, // not handled yet
      {StateQuery::SwingOrHumidity, &DaikinS21::handle_state_swing_or_humidity},
      // {StateQuery::OldProtocol, &DaikinS21::handle_nop, true}, // already added in initial detection
      // {StateQuery::InsideOutsideTemperatures, &DaikinS21::handle_state_inside_outside_temperature}, // added if granular sensors fail
    });
    if (this->protocol_version <= ProtocolVersion(2)) {
      this->queries.insert(this->queries.end(), {
        {EnvironmentQuery::CompressorOnOff, &DaikinS21::handle_env_compressor_on_off},
        {MiscQuery::Model, &DaikinS21::handle_misc_model_v0, true}, // some sort of model? always "3E53" for me, regardless of head unit -- outdoor unit?
        {MiscQuery::Version, &DaikinS21::handle_nop, true}, // purportedly another version, always "00C0" for me
      });
    }
    if (this->protocol_version >= ProtocolVersion(2)) {
      this->queries.insert(this->queries.end(), {
        {StateQuery::SpecialModes, &DaikinS21::handle_state_special_modes},
        {StateQuery::DemandAndEcono, &DaikinS21::handle_state_demand_and_econo},
        // {StateQuery::FB, &DaikinS21::handle_nop}, // unknown
        {StateQuery::ModelCode, &DaikinS21::handle_state_model_code_v2, true},
        {StateQuery::IRCounter, &DaikinS21::handle_state_ir_counter},
        {StateQuery::V2OptionalFeatures, &DaikinS21::handle_nop, true},
        {StateQuery::PowerConsumption, &DaikinS21::handle_state_power_consumption},
        // {StateQuery::ITELC, &DaikinS21::handle_nop},  // unknown, daikin intelligent touch controller?
        // {StateQuery::FP, &DaikinS21::handle_nop}, // unknown
        // {StateQuery::FQ, &DaikinS21::handle_nop}, // unknown
        // {StateQuery::FS, &DaikinS21::handle_nop}, // unknown
        {StateQuery::FT, &DaikinS21::handle_nop, true}, // unknown, outdoor unit capacity?
        // {StateQuery::FV, &DaikinS21::handle_nop}, // unknown
        // {StateQuery::NewProtocol, &DaikinS21::handle_nop, true},  // already added in initial detection
        {MiscQuery::SoftwareVersion, &DaikinS21::handle_misc_software_version, true},
      });
    }
    // todo >= ProtocolVersion(3,0)
    // common sensors
    this->queries.insert(this->queries.end(), {
      // {EnvironmentQuery::PowerOnOff, &DaikinS21::handle_env_power_on_off}, // redundant
      // {EnvironmentQuery::IndoorUnitMode, &DaikinS21::handle_env_indoor_unit_mode}, // redundant
      // {EnvironmentQuery::TemperatureSetpoint, &DaikinS21::handle_env_temperature_setpoint},  // redundant
      // {EnvironmentQuery::OnTimerSetting}, // unused, unsupported
      // {EnvironmentQuery::OffTimerSetting},  // unused, unsupported
      // {EnvironmentQuery::SwingMode, &DaikinS21::handle_env_swing_mode},  // redundant
      // {EnvironmentQuery::FanMode, &DaikinS21::handle_env_fan_mode},  // added in optional feature detection
      {EnvironmentQuery::InsideTemperature, &DaikinS21::handle_env_inside_temperature},
      {EnvironmentQuery::LiquidTemperature, &DaikinS21::handle_env_liquid_temperature},
      // {EnvironmentQuery::FanSpeedSetpoint, &DaikinS21::handle_env_fan_speed_setpoint},  // not supported yet, can translate DaikinFanMode to RPM
      // {EnvironmentQuery::FanSpeed, &DaikinS21::handle_env_fan_speed}, // added in optional feature detection
      // {EnvironmentQuery::LouvreAngleSetpoint, &DaikinS21::handle_env_vertical_swing_angle_setpoint},  // not supported yet
      // {EnvironmentQuery::VerticalSwingAngle, &DaikinS21::handle_env_vertical_swing_angle}, // added in optional feature detection
      // {EnvironmentQuery::RW, &DaikinS21::handle_nop},  // unknown, "00" for me
      {EnvironmentQuery::TargetTemperature, &DaikinS21::handle_env_target_temperature},
      {EnvironmentQuery::OutsideTemperature, &DaikinS21::handle_env_outside_temperature},
      {EnvironmentQuery::IndoorFrequencyCommandSignal, &DaikinS21::handle_env_indoor_frequency_command_signal},
      {EnvironmentQuery::CompressorFrequency, &DaikinS21::handle_env_compressor_frequency},
      // {EnvironmentQuery::IndoorHumidity, &DaikinS21::handle_env_indoor_humidity},  // added in optional feature detection
      // {EnvironmentQuery::CompressorOnOff}, // redundant
      // {EnvironmentQuery::UnitState, &DaikinS21::handle_env_unit_state}, // added in model detection
      // {EnvironmentQuery::SystemState, &DaikinS21::handle_env_system_state}, // added in model detection
      // {EnvironmentQuery::Rz52, &DaikinS21::handle_nop},  // unknown, "40"for me
      // {EnvironmentQuery::Rz72, &DaikinS21::handle_nop},  // unknown, "23" for me
    });
  }

  // Protocol detected and initial queries scheduled
  if (this->ready.all()) {
    return; // nothing to refine
  }

  // Detect and handle optional features
  if (this->ready[ReadyOptionalFeatures] == false) {
    const auto features = this->get_query_result(StateQuery::OptionalFeatures);
    const auto v2_features = this->get_query_result(StateQuery::V2OptionalFeatures);
    // done if all queries have been resolved
    this->ready[ReadyOptionalFeatures] = (features && v2_features);
    // handle results
    if (this->ready[ReadyOptionalFeatures]) {
      // v2 GK info gates base G2 support if present
      if (v2_features.nak || (v2_features.ack && (v2_features.value)[2] & 0b00000100)) {
        // swing
        this->support.swing = (features.ack && (features.value[0] & 0b0100));
        if (this->support.swing) {
          this->support.horizontal_swing = (features.ack && (features.value[0] & 0b1000));
          this->queries.emplace_back(EnvironmentQuery::VerticalSwingAngle, &DaikinS21::handle_env_vertical_swing_angle);
        }

        // fan
        this->support.fan = true;
        this->queries.insert(this->queries.end(), {
          {EnvironmentQuery::FanMode, &DaikinS21::handle_env_fan_mode},
          {EnvironmentQuery::FanSpeed, &DaikinS21::handle_env_fan_speed},
        });
      }

      // humidity
      if (v2_features.nak || (v2_features.ack && (v2_features.value[2] & 0b00000010))) {
        this->support.humidity = (features.ack && (features.value[0] & 0b0010));
        if (this->support.humidity) {
          // unknown if this is a dehumidify mode or humidity sensor. potentially hides the sensor if it's just the mode. my unit supports neither over S21 interface.
          this->queries.emplace_back(EnvironmentQuery::IndoorHumidity, &DaikinS21::handle_env_indoor_humidity);
        }
      }

      // standalone, info only
      if (features.ack) {
        this->support.model_info =    (features.value[1] & 0b1000) ? 'N': 'C';
      }
      if (v2_features.ack) {
        this->support.ac_led =        (v2_features.value[0] & 0b00000001);
        this->support.laundry =       (v2_features.value[0] & 0b00001000);
        this->support.elec =          (v2_features.value[1] & 0b00000001);
        this->support.temp_range =    (v2_features.value[1] & 0b00000100);
        this->support.motion_detect = (v2_features.value[1] & 0b00001000);
        this->support.ac_japan =      (v2_features.value[2] & 0b00000001) == 0;
        this->support.dry =           (v2_features.value[2] & 0b00001000);
        this->support.demand =        (v2_features.value[3] & 0b00000001);
      }

      // todo climate traits and sensor config check -- let user know they can pare down their yaml
      ESP_LOGD(TAG, "Optional features detected");
    }
  }

  // Enable alternate sensor readout if primary queries are unsupported. Depends on optional features having been detected.
  if (this->ready[ReadyOptionalFeatures] && (this->ready[ReadySensorReadout] == false)) {
    const auto fan_mode = this->get_query_result(EnvironmentQuery::FanMode);
    const auto inside = this->get_query_result(EnvironmentQuery::InsideTemperature);
    const auto outside = this->get_query_result(EnvironmentQuery::OutsideTemperature);
    const auto humidity = this->get_query_result(EnvironmentQuery::IndoorHumidity);
    // done if all queries have been resolved
    this->ready[ReadySensorReadout] = (fan_mode && inside && outside && humidity);
    // handle results
    if (this->ready[ReadySensorReadout]) {
      this->support.fan_mode_query = fan_mode.ack;
      this->support.inside_temperature_query = inside.ack;
      this->support.outside_temperature_query = outside.ack;
      this->support.humidity_query = humidity.ack;
      // enable coarse fallback query if any sensor query failed
      const bool alt = (inside.nak || outside.nak || (this->support.humidity && humidity.nak)); // only enable for humidity's sake if declared to be supported
      if (alt) {
        this->queries.emplace_back(StateQuery::InsideOutsideTemperatures, &DaikinS21::handle_state_inside_outside_temperature);
      }
      ESP_LOGD(TAG, "%s sensor readout selected", alt ? "Alternate" : "Dedicated");
    }
  }

  // Detect the model and handle model-specific quirks
  if (this->ready[ReadyModelDetection] == false) {
    const auto model_v0 = this->get_query_result(MiscQuery::Model);
    const auto model_v2 = this->get_query_result(StateQuery::ModelCode);
    // done if all queries have been resolved
    this->ready[ReadyModelDetection] = (model_v0 && model_v2);
    // handle results
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
        this->queries.insert(this->queries.end(), {
          {EnvironmentQuery::UnitState, &DaikinS21::handle_env_unit_state},
          {EnvironmentQuery::SystemState, &DaikinS21::handle_env_system_state},
        });
      }
      // no v2 handling required yet
      ESP_LOGD(TAG, "Model %04" PRIX16 " %04" PRIX16 " detected", this->modelV0, this->modelV2);
    }
  }

  // Select the source of the active flag. Depends on the model having been detected.
  if (this->ready[ReadyModelDetection] && (this->ready[ReadyActiveSource] == false)) {
    const auto compressor_on_off = this->get_query_result(EnvironmentQuery::CompressorOnOff);
    if (compressor_on_off) {
      if (compressor_on_off.ack) {
        this->support.active_source = ActiveSource::CompressorOnOff;
      } else {
        const auto unit_state = this->get_query_result(EnvironmentQuery::UnitState);
        if (unit_state) {
          if (unit_state.ack) {
            this->support.active_source = ActiveSource::UnitState;
          }
          if (unit_state.nak) {
            this->support.unit_system_state_queries = false;  // unit state is assumed to be supported, if it isn't then correct the record
            this->support.active_source = ActiveSource::Unsupported;
            this->current.active = true;  // always active, reported action will follow unit
          }
        }
      }
    }
    // handle results
    this->ready[ReadyActiveSource] = (this->support.active_source != ActiveSource::Unknown);
    if (this->ready[ReadyActiveSource]) {
      ESP_LOGD(TAG, "Active source is %s", active_source_to_string(this->support.active_source));
    }
  }

  // Finally, schedule any user specified debug queries
  // Done last so as to not duplicate automatically added queries
  if (this->ready.all()) {
    for (const auto debug_query : this->debug_queries) {
      const auto existing = get_query_result(debug_query);
      if (existing == QueryNotScheduled) {
        ESP_LOGD(TAG, "Adding query %" PRI_SV, PRI_SV_ARGS(debug_query));
        this->queries.emplace_back(debug_query, &DaikinS21::handle_nop);
      }
    }
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
  std::array<uint8_t, 4U> payload;  // all command payloads here are 4 bytes long for now

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
    payload[1] = (this->pending.climate.swing != climate::CLIMATE_SWING_OFF) ? '?' : '0';
    payload[2] = '0';
    payload[3] = '0';
    this->send_command(StateCommand::LouvreSwingMode, payload);
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
        this->send_command(StateCommand::Powerful, payload);
        return;
      case climate::CLIMATE_PRESET_ECO:
        payload[0] = '0';
        payload[1] = enable ? '2' : '0';
        payload[2] = '0';
        payload[3] = '0';
        this->send_command(StateCommand::Econo, payload);
        return;
      case climate::CLIMATE_PRESET_NONE:
      default:
        break;
    }
  }

  // Periodic query cycle
  if (this->query_index < this->queries.size()) {
    this->serial.send_frame(this->current_query()->command);  // query cycle underway, continue
    return;
  }

  // Query cycle complete
  const auto now = millis();
  this->cycle_active = false;
  this->refine_queries();
  this->cycle_time_ms = now - this->cycle_time_start_ms;
  if (this->ready.all()) {
    // resolve action
    if (this->current.unit_state.defrost() && (this->current.action_reported == climate::CLIMATE_ACTION_HEATING)) {
      this->current.action = climate::CLIMATE_ACTION_COOLING; // report cooling during defrost
    } else if (this->current.active || (this->current.action_reported == climate::CLIMATE_ACTION_FAN)) {
      this->current.action = this->current.action_reported; // trust the unit when active or in fan only
    } else {
      this->current.action = climate::CLIMATE_ACTION_IDLE;
    }
    // resolve presets
    if (this->current.powerful) {
      this->current.climate.preset = climate::CLIMATE_PRESET_BOOST;
    } else if (this->current.econo) {
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

void DaikinS21::handle_state_basic(std::span<uint8_t> &payload) {
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
  this->ready.set(ReadyBasic);
}

void DaikinS21::handle_state_swing_or_humidity(std::span<uint8_t> &payload) {
  this->current.climate.swing = daikin_to_climate_swing_mode(payload[0]);
}

void DaikinS21::handle_state_special_modes(std::span<uint8_t> &payload) {
  this->current.powerful =    (payload[0] & 0b00000010);
  this->current.comfort =     (payload[0] & 0b01000000);
  this->current.quiet =       (payload[0] & 0b10000000);
  this->current.streamer =    (payload[1] & 0b10000000);
  this->current.sensor =      (payload[3] & 0b00001000);
  this->current.sensor_led =  (payload[3] & 0b00001100) == 0b00001100;
}

void DaikinS21::handle_state_demand_and_econo(std::span<uint8_t> &payload) {
  this->current.econo =       (payload[1] == '2');
}

/** Coarser than EnvironmentQuery::InsideTemperature and EnvironmentQuery::OutsideTemperature. Added if those queries fail. */
void DaikinS21::handle_state_inside_outside_temperature(std::span<uint8_t> &payload) {
  if (this->support.inside_temperature_query == false) {
    this->temp_inside = (payload[0] - 128) * 5;  // 1 degree
  }
  if ((this->support.outside_temperature_query == false) && (payload[1] != 0xFF)) { // danijelt reports 0xFF when unsupported
    this->temp_outside = (payload[1] - 128) * 5; // 1 degree
  }
  if (this->support.humidity && (this->support.humidity_query == false) && ((payload[2] - '0') <= 100)) {  // Some units report 0xFF when unsupported
    this->humidity = payload[2] - '0';  // 5% granularity
  }
}

void DaikinS21::handle_state_model_code_v2(std::span<uint8_t> &payload) {
  this->modelV2 = ahex_u16_le(payload);
}

void DaikinS21::handle_state_ir_counter(std::span<uint8_t> &payload) {
  this->current.ir_counter = bytes_to_num(payload); // format unknown
}

void DaikinS21::handle_state_power_consumption(std::span<uint8_t> &payload) {
  this->current.power_consumption = ahex_u16_le(payload);
}

/** Vastly inferior to StateQuery::Basic */
void DaikinS21::handle_env_power_on_off(std::span<uint8_t> &payload) {
  const bool active = payload[0] == '1';
}

/** Same info as StateQuery::Basic */
void DaikinS21::handle_env_indoor_unit_mode(std::span<uint8_t> &payload) {
  if (payload[0] == '0') {
    this->current.climate.mode = climate::CLIMATE_MODE_OFF;
    this->current.action_reported = climate::CLIMATE_ACTION_OFF;
  } else {
    this->current.climate.mode = daikin_to_climate_mode(payload[1]);
    this->current.action_reported = daikin_to_climate_action(payload[1]);
  }
}

/** Same info as StateQuery::Basic */
void DaikinS21::handle_env_temperature_setpoint(std::span<uint8_t> &payload) {
  this->current.climate.setpoint = bytes_to_num(payload) * 10;  // whole degrees C
}

/** Same info as StateQuery::SwingOrHumidity */
void DaikinS21::handle_env_swing_mode(std::span<uint8_t> &payload) {
  this->current.climate.swing = daikin_to_climate_swing_mode(payload[0]);
}

/** Better info than StateQuery::Basic (reports quiet) */
void DaikinS21::handle_env_fan_mode(std::span<uint8_t> &payload) {
  this->current.climate.fan = static_cast<daikin_s21::DaikinFanMode>(payload[0]);
}

void DaikinS21::handle_env_inside_temperature(std::span<uint8_t> &payload) {
  this->temp_inside = bytes_to_num(payload);
}

void DaikinS21::handle_env_liquid_temperature(std::span<uint8_t> &payload) {
  this->temp_coil = bytes_to_num(payload);
}

void DaikinS21::handle_env_fan_speed_setpoint(std::span<uint8_t> &payload) {
  this->current.fan_rpm_setpoint = bytes_to_num(payload) * 10;
}

void DaikinS21::handle_env_fan_speed(std::span<uint8_t> &payload) {
  this->current.fan_rpm = bytes_to_num(payload) * 10;
}

void DaikinS21::handle_env_vertical_swing_angle_setpoint(std::span<uint8_t> &payload) {
  this->current.swing_vertical_angle_setpoint = bytes_to_num(payload);
}

void DaikinS21::handle_env_vertical_swing_angle(std::span<uint8_t> &payload) {
  this->current.swing_vertical_angle = bytes_to_num(payload);
}

void DaikinS21::handle_env_target_temperature(std::span<uint8_t> &payload) {
  this->temp_target = bytes_to_num(payload); // Internal control loop target temperature
}

void DaikinS21::handle_env_outside_temperature(std::span<uint8_t> &payload) {
  this->temp_outside = bytes_to_num(payload);
}

void DaikinS21::handle_env_indoor_frequency_command_signal(std::span<uint8_t> &payload) {
  this->demand = bytes_to_num(payload);  // Demand, 0-15
}

void DaikinS21::handle_env_compressor_frequency(std::span<uint8_t> &payload) {
  this->compressor_hz = bytes_to_num(payload);
  if (this->compressor_hz == 999) {
    this->compressor_hz = 0;  // reported by danijelt
  }
}

void DaikinS21::handle_env_indoor_humidity(std::span<uint8_t> &payload) {
  this->humidity = bytes_to_num(payload);
}

void DaikinS21::handle_env_compressor_on_off(std::span<uint8_t> &payload) {
  this->current.active = (payload[0] == '1'); // highest precedence active source, if this query is working there's no need to check active_source
}

void DaikinS21::handle_env_unit_state(std::span<uint8_t> &payload) {
  this->current.unit_state = ahex_digit(payload[0]);
  if (this->support.active_source == ActiveSource::UnitState) {
    this->current.active = this->current.unit_state.active(); // used to refine climate action
  }
  this->current.powerful = this->current.unit_state.powerful();  // if G6 is unsupported we can still read out powerful set by remote
}

void DaikinS21::handle_env_system_state(std::span<uint8_t> &payload) {
  this->current.system_state = ahex_u8_le(payload[0], payload[1]);
}

void DaikinS21::handle_misc_model_v0(std::span<uint8_t> &payload) {
  this->modelV0 = ahex_u16_le(payload);
}

void DaikinS21::handle_misc_software_version(std::span<uint8_t> &payload) {
  payload = payload.subspan(0, DaikinQueryValue::software_version_length);  // odd command, trailing characters are "00000M"
  std::ranges::reverse(payload);  // reversed ascii
}

/**
 * Determine the protocol version according to Faikin documentation.
 *
 * @note This is mostly untested, I own only one Daikin system.
 *
 * @return true protocl version was detected
 * @return false protocol version wasn't detected
 */
bool DaikinS21::determine_protocol_version() {
  static constexpr uint8_t old_version_0[4] = {'0',0,0,0};
  static constexpr uint8_t old_version_1[4] = {'0','1',0,0};
  static constexpr uint8_t old_version_2or3[4] = {'0','2',0,0};
  static constexpr uint8_t old_version_31plus[4] = {'0','2','0','0'};

  // both protocol indicators should have been scheduled on init
  const auto old_proto = this->get_query_result(StateQuery::OldProtocol);
  const auto new_proto = this->get_query_result(StateQuery::NewProtocol);

  // Check availability first
  if (old_proto.ack && new_proto.nak) {
    if (std::ranges::equal(old_proto.value, old_version_0)) {
      this->protocol_version = ProtocolVersion(0);
    } else if (std::ranges::equal(old_proto.value, old_version_1)) {
      this->protocol_version = ProtocolVersion(1);
    } else if (std::ranges::equal(old_proto.value, old_version_2or3) || std::ranges::equal(old_proto.value, old_version_31plus)) {
      this->protocol_version = ProtocolVersion(2); // NAK for NewProtocol rules out 3.0
    } else {
      this->protocol_version = ProtocolUnknown;
    }
  } else if (old_proto && new_proto.ack) {
    const uint16_t raw_version = bytes_to_num(new_proto.value);
    this->protocol_version = {static_cast<uint8_t>(raw_version / 100), static_cast<uint8_t>(raw_version % 100)};
    // fixup on the 2 / 3.0 protocol border
    if (this->protocol_version == ProtocolVersion(3,0)) {
      if (old_proto.ack && std::ranges::equal(old_proto.value, old_version_31plus)) {
        this->protocol_version = ProtocolVersion(3,10);
      }
    }
  } else if (old_proto.nak && new_proto.nak) {
    // both nak'd, even though we're talking to the unit?
    this->protocol_version = ProtocolUnknown;
  } else {
    ESP_LOGV(TAG, "Protocol version not ready yet");
  }

  // Print some info if we're falling back to version 0
  if (this->protocol_version == ProtocolUnknown) {
    ESP_LOGE(TAG, "Unable to detect protocol version! Old: %s New: %s",
      str_repr(old_proto.value).c_str(),
      str_repr(new_proto.value).c_str());
  }

  return this->protocol_version != daikin_s21::ProtocolUndetected;
}

void DaikinS21::handle_serial_result(const DaikinSerial::Result result, const std::span<uint8_t> response /*= {}*/) {
  const bool is_query = this->current_command.empty();
  const std::string_view tx_str = is_query ? this->current_query()->command : this->current_command;
  std::span<uint8_t> payload = { response.begin() + tx_str.size(), response.end() };

  // add commands to this array to debug their output. empty string is just a placeholder to compile
  static constexpr std::array debug_commands{""};
  const bool is_debug = this->debug && (std::ranges::find(debug_commands, tx_str) != debug_commands.end());

  // debug logging
  switch (result) {
    case DaikinSerial::Result::Ack:
      if (/*this->debug ||*/ is_debug) {  // uncomment to debug all, not just debug_commands
        ESP_LOGD(TAG, "ACK: %" PRI_SV " -> %s %s",
                  PRI_SV_ARGS(tx_str),
                  str_repr(payload).c_str(),
                  hex_repr(payload).c_str());
      }
      break;
    case DaikinSerial::Result::Nak:
      ESP_LOGW(TAG, "NAK for %" PRI_SV, PRI_SV_ARGS(tx_str));
      break;
    case DaikinSerial::Result::Timeout:
      ESP_LOGW(TAG, "Timeout waiting for response to %" PRI_SV, PRI_SV_ARGS(tx_str));
      break;
    case DaikinSerial::Result::Error:
      ESP_LOGE(TAG, "Error with %" PRI_SV, PRI_SV_ARGS(tx_str));
      break;
    default:
      break;
  }

  // handle serial result
  switch (result) {
    case DaikinSerial::Result::Ack:
      if (is_query) {
        // mark as acked
        this->current_query()->acked = true;
        this->current_query()->naks = 0;
        // print changed values
        if ((/*this->debug ||*/ is_debug) &&  // uncomment to debug all, not just debug_commands
            (std::ranges::equal(this->current_query()->value(), payload) == false)) {
          ESP_LOGI(TAG, "%" PRI_SV " changed: %s %s -> %s %s",
                    PRI_SV_ARGS(this->current_query()->command),
                    str_repr(this->current_query()->value()).c_str(),
                    hex_repr(this->current_query()->value()).c_str(),
                    str_repr(payload).c_str(),
                    hex_repr(payload).c_str());
        }
        // decode payload
        if (this->current_query()->handler != nullptr) {
          std::invoke(this->current_query()->handler, this, payload);
        } else {
          ESP_LOGI(TAG, "Unhandled command: %s", hex_repr(response).c_str());
        }
        // save a copy of the payload
        this->current_query()->set_value(payload);
        if (this->current_query()->is_static) {
          this->static_queries.emplace_back(*this->current_query());
        }
      } else {
        // nothing yet to do when a command is accepted
      }
      break;

    case DaikinSerial::Result::Nak:
      if (is_query) {
        this->current_query()->naks++;
      } else {
        // nothing yet to do when a command is rejected
      }
      break;

    case DaikinSerial::Result::Timeout:
      if (is_query) {
        // It's possible some unsupported queries don't respond at all
        // Treat these as NAKs if we've established communication
        // Otherwise, a disconnected or unpowered HVAC unit will quickly cause all queries to fail
        if (this->comms_detected()) {
          this->current_query()->naks++;
        }
      } else {
        // command will be retried, let's hope the code that generates it is bug free
      }
      break;

    case DaikinSerial::Result::Error:
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
      if ((result == DaikinSerial::Result::Ack) && this->current_query()->is_static) {
        // got the query result and know it won't change, remove
        this->prune_query(this->current_query()->command);
      } else if (this->current_query()->naks >= 3) {
        // query failed, remove
        ESP_LOGW(TAG, "removing %" PRI_SV " from query pool as unsupported", PRI_SV_ARGS(tx_str));
        this->failed_queries.emplace_back(this->current_query()->command);
        this->prune_query(this->current_query()->command);
      } else {
        // next query
        this->query_index++;
      }
      // if communication established and all queries have failed we had comms then they were lost
      if (this->comms_detected() && this->queries.empty()) {
        this->setup();  // reinitialize in order to prepare for the HVAC unit being reconnected
      }
    } else {
      this->current_command = {};
    }
  }
}

void DaikinS21::dump_state() {
  ESP_LOGD(TAG, "Protocol: %" PRIu8 ".%" PRIu8 "  ModelV0: %04" PRIX16 "  ModelV2: %04" PRIX16,
      this->protocol_version.major,
      this->protocol_version.minor,
      this->modelV0,
      this->modelV2);
  if (this->debug) {
    const auto old_proto = this->get_query_result(StateQuery::OldProtocol);
    const auto new_proto = this->get_query_result(StateQuery::NewProtocol);
    const auto misc_version = this->get_query_result(MiscQuery::Version);
    const auto software_version = this->get_query_result(MiscQuery::SoftwareVersion);
    ESP_LOGD(TAG, " G8: %s  GY00: %s  V: %s  VS000M: %s",
        str_repr(old_proto.value).c_str(),
        str_repr(new_proto.value).c_str(),
        str_repr(misc_version.value).c_str(),
        str_repr(software_version.value).c_str());
  }
  ESP_LOGD(TAG, "ModelInfo: %c  VSwing: %c  HSwing: %c  Humid: %c  Fan: %c",
      this->support.model_info,
      this->support.swing ? 'Y' : 'N',
      this->support.horizontal_swing ? 'Y' : 'N',
      this->support.humidity ? 'Y' : 'N',
      this->support.fan ? 'Y' : 'N');
  if (this->debug) {
    const auto features = this->get_query_result(StateQuery::OptionalFeatures);
    const auto v2_features = this->get_query_result(StateQuery::V2OptionalFeatures);
    const auto ft_capacity = this->get_query_result(StateQuery::FT);
    ESP_LOGD(TAG, " G2: %s  GK: %s  GT: %s  ActiveSrc: %s",
        features.ack ? hex_repr(features.value).c_str() : str_repr(features.value).c_str(),
        v2_features.ack ? hex_repr(v2_features.value).c_str() : str_repr(v2_features.value).c_str(),
        ft_capacity.ack ? hex_repr(ft_capacity.value).c_str() : str_repr(ft_capacity.value).c_str(),
        active_source_to_string(this->support.active_source));
  }
  ESP_LOGD(TAG, "Mode: %s  Action: %s  Preset: %s  Demand: %" PRIu8,
      LOG_STR_ARG(climate::climate_mode_to_string(this->current.climate.mode)),
      LOG_STR_ARG(climate::climate_action_to_string(this->get_climate_action())),
      LOG_STR_ARG(climate::climate_preset_to_string(this->current.climate.preset)),
      this->get_demand());
  if (this->support.fan || this->support.swing) {
    ESP_LOGD(TAG, "Fan: %" PRI_SV " (%" PRIu16 " RPM)  Swing: %s",
        PRI_SV_ARGS(daikin_fan_mode_to_string_view(this->current.climate.fan)),
        this->current.fan_rpm,
        this->support.swing ? LOG_STR_ARG(climate::climate_swing_mode_to_string(this->current.climate.swing)) : "N/A");
  }
  ESP_LOGD(TAG, "Setpoint: %.1fC  Target: %.1fC  Inside: %.1fC  Coil: %.1fC",
      this->get_temp_setpoint().f_degc(),
      this->get_temp_target().f_degc(),
      this->get_temp_inside().f_degc(),
      this->get_temp_coil().f_degc());
  if (this->support.humidity) {
    ESP_LOGD(TAG, "Humid: %" PRIu8 "%%", this->get_humidity());
  }
  ESP_LOGD(TAG, "Cycle Time: %" PRIu32 "ms  UnitState: %" PRIX8 "  SysState: %02" PRIX8,
      this->cycle_time_ms,
      this->current.unit_state.raw,
      this->current.system_state.raw);
  if (this->debug) {
    const auto comma_join = [](const auto& queries) {
      std::string str;
      for (const auto &q : queries) {
        str += q;
        if (q != queries.back()) {
          str += ",";
        }
      }
      return str;
    };
    ESP_LOGD(TAG, " Active: %s", LOG_STR_ARG(comma_join(std::views::transform(this->queries, DaikinQueryState::GetCommand)).c_str()));
    ESP_LOGD(TAG, "  Nak'd: %s", LOG_STR_ARG(comma_join(this->failed_queries).c_str()));
    ESP_LOGD(TAG, " Static: %s", LOG_STR_ARG(comma_join(std::views::transform(this->static_queries, DaikinQueryState::GetCommand)).c_str()));
  }
}

} // namespace esphome::daikin_s21
