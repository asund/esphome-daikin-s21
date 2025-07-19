#include <cinttypes>
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "daikin_s21_climate.h"
#include "../daikin_s21_fan_modes.h"

using namespace esphome;

namespace esphome {
namespace daikin_s21 {

constexpr float SETPOINT_STEP = 0.5F; // ESPHome thermostat calculations -- unit's internal support and visual step may differ

static const char *const TAG = "daikin_s21.climate";

void DaikinS21Climate::setup() {
  uint32_t h = this->get_object_id_hash();
  auto_setpoint_pref = global_preferences->make_preference<int16_t>(h + 1);
  cool_setpoint_pref = global_preferences->make_preference<int16_t>(h + 2);
  heat_setpoint_pref = global_preferences->make_preference<int16_t>(h + 3);
  this->set_custom_fan_mode(commanded.fan); // ensure custom fan mode string is populated with a default
}

/**
 * Component polling loop.
 * 
 * Check the DaikinS21 component to see if it's updated.
 * Update climate state if a previous command isn't in progress.
 * Publish any state changes to Home Assistant.
 */
void DaikinS21Climate::loop() {
  if (this->s21->climate_updated) {
    const auto& reported = this->s21->get_climate_settings();

    // New state, determine if we can publish
    const bool can_publish = (static_cast<int32_t>(millis() - this->command_timeout_end_ms) >= 0) ||  // command not in progress
                             (this->commanded == reported);                         // command took effect, no longer in progress
    if (can_publish) {
      // Clear the updated flag only if there's no command in progress.
      // A command timeout could occur before the next update and we'll want to publish the current state before then
      this->s21->climate_updated = false;
      this->command_timeout_end_ms = millis(); // move forward to avoid underflow in subsequent expiry calculations

      // Allowed to publish, determine if we should
      bool do_publish = false;

      // Detect and integrate new sensor values into component state
      if ((this->action != this->s21->get_climate_action()) ||
          (this->current_temperature != this->get_effective_current_temperature()) ||
          (this->current_humidity != this->s21->get_humidity())) {
        this->action = this->s21->get_climate_action();
        this->current_temperature = this->get_effective_current_temperature();
        this->current_humidity = this->s21->get_humidity();
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
        if (this->commanded.fan != reported.fan) {
          this->set_custom_fan_mode(reported.fan);   // avoid custom string operations until there's a change that requires publishing
        }
        this->mode = reported.mode;
        this->swing_mode = reported.swing;
        this->commanded = reported;
        do_publish = true;
      }

      // Only publish when state changed
      if (do_publish) {
        this->publish_state();
      }
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
      ESP_LOGCONFIG(TAG, "  Setpoint interval: %ds", this->setpoint_interval_s);
    }
  }
  this->dump_traits_(TAG);
}

climate::ClimateTraits DaikinS21Climate::traits() {
  auto traits = climate::ClimateTraits();
  // Base
  traits.set_supports_action(true);
  traits.set_supports_current_temperature(true);
  traits.set_visual_min_temperature(10.0F);
  traits.set_visual_max_temperature(32.0F);
  traits.set_visual_current_temperature_step(0.5F);
  traits.set_visual_target_temperature_step(1.0F);
  traits.set_supported_modes({
      climate::CLIMATE_MODE_OFF,
      climate::CLIMATE_MODE_HEAT_COOL,
      climate::CLIMATE_MODE_COOL,
      climate::CLIMATE_MODE_HEAT,
      climate::CLIMATE_MODE_FAN_ONLY,
      climate::CLIMATE_MODE_DRY,
  });
  std::array<std::string, std::size(supported_daikin_fan_modes)> supported_fan_mode_strings;
  std::ranges::transform(supported_daikin_fan_modes, std::begin(supported_fan_mode_strings), [](const auto &arg){ return daikin_fan_mode_to_string_view(arg); } );
  traits.set_supported_custom_fan_modes({std::begin(supported_fan_mode_strings), std::end(supported_fan_mode_strings)});
  traits.set_supported_swing_modes({
      climate::CLIMATE_SWING_OFF,
      climate::CLIMATE_SWING_BOTH,
      climate::CLIMATE_SWING_VERTICAL,
      climate::CLIMATE_SWING_HORIZONTAL,
  });
  // Overrides
  if (this->supports_current_humidity_) {
    traits.set_supports_current_humidity(this->supports_current_humidity_);
  }
  if (this->supported_modes_override_.has_value()) {
    traits.set_supported_modes(this->supported_modes_override_.value());
    traits.add_supported_mode(climate::CLIMATE_MODE_OFF);   // Always available
    traits.add_supported_mode(climate::CLIMATE_MODE_HEAT_COOL);  // Always available
  }
  return traits;
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
  return this->s21->get_temp_inside();
}

float DaikinS21Climate::get_room_temp_offset() {
  if (!this->use_room_sensor()) {
    return 0.0F;
  }
  float room_val = this->room_sensor_degc();
  float s21_val = this->s21->get_temp_inside();
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
  return abs(this->s21->get_setpoint() - this->calc_s21_setpoint());
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
  this->commanded.fan = string_to_daikin_fan_mode(this->custom_fan_mode.value());
  this->commanded.swing = this->swing_mode;
  this->s21->set_climate_settings(this->commanded);

  ESP_LOGI(TAG, "Controlling S21 climate:");
  ESP_LOGI(TAG, "  Mode: %s", LOG_STR_ARG(climate::climate_mode_to_string(this->commanded.mode)));
  ESP_LOGI(TAG, "  Setpoint: %.1f (s21: %.1f)", this->target_temperature, this->commanded.setpoint.f_degc());
  ESP_LOGI(TAG, "  Fan: %" PRI_SV, PRI_SV_ARGS(daikin_fan_mode_to_string_view(this->commanded.fan)));
  ESP_LOGI(TAG, "  Swing: %s", LOG_STR_ARG(climate::climate_swing_mode_to_string(this->commanded.swing)));

  // HVAC unit takes a few seconds to begin reporting settings changes back to
  // the controller, so when modifying don't publish current state until it
  // takes effect or is given enough time to take effect or we get a jumpy UI
  // in Home Assistant as stale state is published over the commanded state
  // followed by the active state.
  this->command_timeout_end_ms = millis() + state_publication_timeout_ms;
  this->save_setpoint(this->target_temperature);
}

}  // namespace daikin_s21
}  // namespace esphome
