#include <cinttypes>
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "daikin_s21_climate.h"
#include "../daikin_s21_fan_modes.h"

using namespace esphome;

namespace esphome {
namespace daikin_s21 {

constexpr float SETPOINT_STEP = 0.5F; // ESPHome thermostat calculations -- unit's internal support and visual step may differ

static const char *const TAG = "daikin_s21.climate";

void DaikinS21Climate::setup() {
  uint32_t h = this->get_object_id_hash();
  this->auto_setpoint_pref = global_preferences->make_preference<int16_t>(h + 1);
  this->cool_setpoint_pref = global_preferences->make_preference<int16_t>(h + 2);
  this->heat_setpoint_pref = global_preferences->make_preference<int16_t>(h + 3);
  // populate default traits
  this->traits_.set_supports_action(true);
  this->traits_.set_supports_current_temperature(true);
  this->traits_.set_visual_min_temperature(10.0F);
  this->traits_.set_visual_max_temperature(32.0F);
  this->traits_.set_visual_current_temperature_step(0.5F);
  this->traits_.set_visual_target_temperature_step(1.0F);
  this->traits_.set_supported_modes({
      climate::CLIMATE_MODE_OFF,
      climate::CLIMATE_MODE_HEAT_COOL,
      climate::CLIMATE_MODE_COOL,
      climate::CLIMATE_MODE_HEAT,
      climate::CLIMATE_MODE_FAN_ONLY,
      climate::CLIMATE_MODE_DRY,
  });
  std::array<std::string, std::size(supported_daikin_fan_modes)> supported_fan_mode_strings;
  std::ranges::transform(supported_daikin_fan_modes, std::begin(supported_fan_mode_strings), [](const auto &arg){ return daikin_fan_mode_to_string_view(arg); } );
  this->traits_.set_supported_custom_fan_modes({std::begin(supported_fan_mode_strings), std::end(supported_fan_mode_strings)});
  this->traits_.set_supported_swing_modes({
      climate::CLIMATE_SWING_OFF,
      climate::CLIMATE_SWING_BOTH,
      climate::CLIMATE_SWING_VERTICAL,
      climate::CLIMATE_SWING_HORIZONTAL,
  });
  // ensure optionals are populated with defaults
  this->set_custom_fan_mode(commanded.fan);
  this->preset = commanded.preset;
  // enable event driven updates
  this->get_parent()->add_climate_callback(std::bind(&DaikinS21Climate::update_handler, this)); // enable update events from DaikinS21
  this->set_command_timeout(0);  // schedule an immediate update to capture the current state (change detection on update requires a "previous" state)
  this->disable_loop(); // wait for updates
}
/**
 * Command timeout handler
 * 
 * Called when a command times out without seeming to take effect.
 * Trigger a publish of the current state.
 */
void DaikinS21Climate::command_timeout_handler() {
  command_active = false;
  update_handler();
}

/**
 * Update handler
 * 
 * Update climate state if a previous command isn't in progress.
 * Publish any state changes to Home Assistant.
 *
 * Called by DaikinS21 on every complete system state update.
 */
void DaikinS21Climate::update_handler() {
  const auto& reported = this->get_parent()->get_climate_settings();

  // If an active command took effect, cancel the timeout and lift the publication ban
  if (command_active && (this->commanded == reported)) {
    this->command_active = false;
    this->cancel_timeout(command_timeout_name);
  }
  
  // Don't publish current state if a command is in progress -- avoids UI glitches
  if (command_active == false) {
    // Allowed to publish, determine if we should
    bool do_publish = false;

    // Detect and integrate new sensor values into component state
    if ((this->action != this->get_parent()->get_climate_action()) ||
        (this->current_temperature != this->get_effective_current_temperature()) ||
        (this->current_humidity != this->get_parent()->get_humidity())) {
      this->action = this->get_parent()->get_climate_action();
      this->current_temperature = this->get_effective_current_temperature();
      this->current_humidity = this->get_parent()->get_humidity();
      do_publish = true;
    }

    // Detect and integrate new settings values into component state
    if (this->should_check_setpoint()) {
      this->last_setpoint_check_ms = millis();

      if (this->use_room_sensor()) {
        ESP_LOGD(TAG, "Room temp from external sensor: %.1f %s (%.1f °C)",
                this->room_sensor_->get_state(),
                this->room_sensor_->get_unit_of_measurement().c_str(),
                this->room_sensor_degc());
        ESP_LOGD(TAG, "  Offset: %.1f", this->get_room_temp_offset());
      }

      // Target temperature is stored by climate class, and is used to represent
      // the user's desired temperature. This is distinct from the HVAC unit's
      // setpoint because we may be using an external sensor. So we only update
      // the target temperature here if it appears uninitialized.
      float unexpected_diff = abs(this->commanded.setpoint.f_degc() - reported.setpoint.f_degc());
      if (this->target_temperature == 0.0F || isnanf(this->target_temperature)) {
        // Use stored setpoint for mode, or fall back to use s21's setpoint.
        this->target_temperature = this->load_setpoint().value_or(reported.setpoint.f_degc());
        this->set_s21_climate();
      } else if (unexpected_diff >= SETPOINT_STEP) {
        // User probably set temp via IR remote -- so try to honor their wish by
        // matching controller's target value to what they sent via remote.
        ESP_LOGI(TAG, "S21 setpoint changed outside controller, updating target temp (expected %.1f, found %.1f)",
            this->commanded.setpoint.f_degc(), reported.setpoint.f_degc());
        this->target_temperature = reported.setpoint.f_degc();
        this->set_s21_climate();
      } else if (this->s21_setpoint_variance() >= SETPOINT_STEP) {
        // Room temperature offset has probably changed, so we need to adjust
        // the s21 setpoint based on the new difference.
        this->set_s21_climate();
        ESP_LOGI(TAG, "S21 setpoint updated to %.1f", this->commanded.setpoint.f_degc());
      }
    }

    // If the resulting commanded settings of the above are not being reported we should publish them
    if (this->commanded != reported) {
      this->mode = reported.mode;
      this->swing_mode = reported.swing;
      if (this->commanded.fan != reported.fan) {
        this->set_custom_fan_mode(reported.fan);   // avoid custom string operations until there's a change that requires publishing
      }
      this->preset = reported.preset;
      this->commanded = reported;
      do_publish = true;
    }

    // Only publish when state changed
    if (do_publish) {
      this->publish_state();
    }
  }
}

void DaikinS21Climate::dump_config() {
  ESP_LOGCONFIG(TAG, "DaikinS21Climate:");
  if (this->room_sensor_ != nullptr) {
    if (!this->room_sensor_unit_is_valid()) {
      ESP_LOGCONFIG(TAG, "  ROOM SENSOR: INVALID UNIT '%s' (must be °C or °F)",
                    this->room_sensor_->get_unit_of_measurement().c_str());
    } else {
      ESP_LOGCONFIG(TAG, "  Room sensor: %s",
                    this->room_sensor_->get_name().c_str());
      ESP_LOGCONFIG(TAG, "  Setpoint interval: %" PRIu16 "s", this->setpoint_interval_s);
    }
  }
  this->dump_traits_(TAG);
}

/**
 * Override supported modes
 *
 * @note Modifies traits, call during setup only
 * 
 * @param modes modes to support
 */
void DaikinS21Climate::set_supported_modes_override(std::set<climate::ClimateMode> modes) {
  this->traits_.set_supported_modes(modes);
  this->traits_.add_supported_mode(climate::CLIMATE_MODE_OFF);   // Always available
  this->traits_.add_supported_mode(climate::CLIMATE_MODE_HEAT_COOL);  // Always available
}

/**
 * Override supported presets
 *
 * @note Modifies traits, call during setup only
 * 
 * @param presets presets to support
 */
void DaikinS21Climate::set_supported_presets_override(std::set<climate::ClimatePreset> presets) {
  this->traits_.set_supported_presets(presets);
  this->traits_.add_supported_preset(climate::CLIMATE_PRESET_NONE);
}

/**
 * Configure to report humidity
 *
 * @note Modifies traits, call during setup only
 * 
 * @param supports_current_humidity true to report humidity, false to disable
 */
void DaikinS21Climate::set_supports_current_humidity(const bool supports_current_humidity) {
  this->traits_.set_supports_current_humidity(supports_current_humidity);
}

void DaikinS21Climate::set_custom_fan_mode(const DaikinFanMode mode) {
  this->commanded.fan = mode;
  this->custom_fan_mode = static_cast<std::string>(daikin_fan_mode_to_string_view(mode)); 
}

bool DaikinS21Climate::use_room_sensor() {
  return this->room_sensor_unit_is_valid() && this->room_sensor_->has_state() &&
         !isnanf(this->room_sensor_->get_state());
}

bool DaikinS21Climate::room_sensor_unit_is_valid() {
  if (this->room_sensor_ != nullptr) {
    auto u = this->room_sensor_->get_unit_of_measurement();
    return u == "°C" || u == "°F";
  }
  return false;
}

float DaikinS21Climate::room_sensor_degc() {
  float temp = this->room_sensor_->get_state();
  if (this->room_sensor_->get_unit_of_measurement() == "°F") {
    temp = fahrenheit_to_celsius(temp);
  }
  return temp;
}

float DaikinS21Climate::get_effective_current_temperature() {
  if (this->use_room_sensor()) {
    return this->room_sensor_degc();
  }
  return this->get_parent()->get_temp_inside();
}

float DaikinS21Climate::get_room_temp_offset() {
  if (!this->use_room_sensor()) {
    return 0.0F;
  }
  float room_val = this->room_sensor_degc();
  float s21_val = this->get_parent()->get_temp_inside();
  return s21_val - room_val;
}

float nearest_step(float temp) {
  return std::round(temp / SETPOINT_STEP) * SETPOINT_STEP;
}

// What setpoint should be sent to s21, acconting for external room sensor.
float DaikinS21Climate::calc_s21_setpoint() {
  float offset_target = this->target_temperature + this->get_room_temp_offset();
  return nearest_step(offset_target);
}

// How far from desired setpoint is the current S21 setpoint?
float DaikinS21Climate::s21_setpoint_variance() {
  return abs(this->get_parent()->get_setpoint() - this->calc_s21_setpoint());
}

void DaikinS21Climate::save_setpoint(float value, ESPPreferenceObject &pref) {
  int16_t stored_val = static_cast<int16_t>(value * 10.0F);
  pref.save(&stored_val);
}

void DaikinS21Climate::save_setpoint(float value) {
  optional<float> prev = this->load_setpoint();
  // Only save if value is different from what's already saved.
  if (abs(value - prev.value_or(0.0F)) >= SETPOINT_STEP) {
    switch (this->mode) {
      case climate::CLIMATE_MODE_HEAT_COOL:
        this->save_setpoint(value, this->auto_setpoint_pref);
        break;
      case climate::CLIMATE_MODE_COOL:
        this->save_setpoint(value, this->cool_setpoint_pref);
        break;
      case climate::CLIMATE_MODE_HEAT:
        this->save_setpoint(value, this->heat_setpoint_pref);
        break;
      default:
        break;
    }
  }
}

optional<float> DaikinS21Climate::load_setpoint(ESPPreferenceObject &pref) {
  int16_t stored_val = 0;
  if (!pref.load(&stored_val)) {
    return {};
  }
  return static_cast<float>(stored_val) / 10.0F;
}

optional<float> DaikinS21Climate::load_setpoint() {
  optional<float> loaded;
  switch (this->mode) {
    case climate::CLIMATE_MODE_HEAT_COOL:
      loaded = this->load_setpoint(this->auto_setpoint_pref);
      break;
    case climate::CLIMATE_MODE_COOL:
      loaded = this->load_setpoint(this->cool_setpoint_pref);
      break;
    case climate::CLIMATE_MODE_HEAT:
      loaded = this->load_setpoint(this->heat_setpoint_pref);
      break;
    default:
      break;
  }
  return loaded;
}

bool DaikinS21Climate::should_check_setpoint() {
  bool mode_uses_setpoint = (this->mode == climate::CLIMATE_MODE_HEAT_COOL) ||
                            (this->mode == climate::CLIMATE_MODE_COOL) ||
                            (this->mode == climate::CLIMATE_MODE_HEAT);
  bool time_elapsed = (millis() - this->last_setpoint_check_ms) > (this->setpoint_interval_s * 1000);
  return mode_uses_setpoint && time_elapsed;
}

/**
 * Inherited ESPHome climate control call handler.
 * 
 * Populates internal state with contained arguments then applies to the unit.
 *
 * @param call the call to process
 */
void DaikinS21Climate::control(const climate::ClimateCall &call) {
  if (call.get_mode().has_value()) {
    this->mode = call.get_mode().value();
    if (!call.get_target_temperature().has_value()) {
      // If call does not include target, then try to use saved target.
      optional<float> sp = this->load_setpoint();
      if (sp.has_value()) {
        this->target_temperature = nearest_step(sp.value());
      }
    }
  }
  if (call.get_target_temperature().has_value()) {
    this->target_temperature = nearest_step(call.get_target_temperature().value());
  }
  if (call.get_swing_mode().has_value()) {
    this->swing_mode = call.get_swing_mode().value();
  }
  if (call.get_custom_fan_mode().has_value()) {
    this->custom_fan_mode = call.get_custom_fan_mode().value();
  }
  if (call.get_preset().has_value()) {
    this->preset = call.get_preset().value();
  }
  this->set_s21_climate();
  this->publish_state();
}

/**
 * Apply ESPHome Climate state to the unit.
 *
 * Converts to internal settings format and forwards to DaikinS21 component to apply.
 */
void DaikinS21Climate::set_s21_climate() {
  // Set and command new settings
  this->commanded.mode = this->mode;
  this->commanded.setpoint = this->calc_s21_setpoint();
  this->commanded.swing = this->swing_mode;
  this->commanded.fan = string_to_daikin_fan_mode(this->custom_fan_mode.value());
  this->commanded.preset = this->preset.value();
  this->get_parent()->set_climate_settings(this->commanded);
  this->set_command_timeout();

  ESP_LOGI(TAG, "Controlling S21 climate:");
  ESP_LOGI(TAG, "  Mode: %s", LOG_STR_ARG(climate::climate_mode_to_string(this->commanded.mode)));
  ESP_LOGI(TAG, "  Setpoint: %.1f (s21: %.1f)", this->target_temperature, this->commanded.setpoint.f_degc());
  ESP_LOGI(TAG, "  Fan: %" PRI_SV, PRI_SV_ARGS(daikin_fan_mode_to_string_view(this->commanded.fan)));
  ESP_LOGI(TAG, "  Swing: %s", LOG_STR_ARG(climate::climate_swing_mode_to_string(this->commanded.swing)));
  ESP_LOGI(TAG, "  Preset: %s", LOG_STR_ARG(climate::climate_preset_to_string(this->commanded.preset)));

  this->save_setpoint(this->target_temperature);
}

/**
 * Set the command timeout.
 *
 * HVAC unit takes a few seconds to begin reporting settings changes back to
 * the controller, so when modifying don't publish current state until it
 * takes effect or is given enough time to take effect or we get a jumpy UI
 * in Home Assistant as stale state is published over the commanded state
 * followed by the active state.
 * 
 * If the timeout expires the state will be published regardless.
 */
void DaikinS21Climate::set_command_timeout(const uint32_t delay_ms /*= state_publication_timeout_ms*/) {
  command_active = true;
  this->set_timeout(command_timeout_name, delay_ms, std::bind(&DaikinS21Climate::command_timeout_handler, this));
}

}  // namespace daikin_s21
}  // namespace esphome
