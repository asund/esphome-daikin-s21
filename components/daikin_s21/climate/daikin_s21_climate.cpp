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
}

void DaikinS21Climate::dump_config() {
  ESP_LOGCONFIG(TAG, "DaikinS21Climate:");
  ESP_LOGCONFIG(TAG, "  Update interval: %" PRIu32, this->get_update_interval());
  if (this->room_sensor_ != nullptr) {
    if (!this->room_sensor_unit_is_valid()) {
      ESP_LOGCONFIG(TAG, "  ROOM SENSOR: INVALID UNIT '%s' (must be °C or °F)",
                    this->room_sensor_->get_unit_of_measurement().c_str());
    } else {
      ESP_LOGCONFIG(TAG, "  Room sensor: %s",
                    this->room_sensor_->get_name().c_str());
      ESP_LOGCONFIG(TAG, "  Setpoint interval: %d", this->setpoint_interval);
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
  std::transform(std::begin(supported_daikin_fan_modes), std::end(supported_daikin_fan_modes), std::begin(supported_fan_mode_strings), [](const auto &arg){ return daikin_fan_mode_to_string_ref(arg).str(); } );
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
    return 0.0;
  }
  float room_val = this->room_sensor_degc();
  float s21_val = this->s21->get_temp_inside();
  return s21_val - room_val;
}

float nearest_step(float temp) {
  return std::round(temp / SETPOINT_STEP) * SETPOINT_STEP;
}

// What setpoint should be sent to s21, acconting for external room sensor.
float DaikinS21Climate::calc_s21_setpoint(float target) {
  float offset_target = target + this->get_room_temp_offset();
  return nearest_step(offset_target);
}

// How far from desired setpoint is the current S21 setpoint?
float DaikinS21Climate::s21_setpoint_variance() {
  return abs(this->s21->get_setpoint() -
             this->calc_s21_setpoint(this->target_temperature));
}

void DaikinS21Climate::save_setpoint(float value, ESPPreferenceObject &pref) {
  int16_t stored_val = static_cast<int16_t>(value * 10.0);
  pref.save(&stored_val);
}

void DaikinS21Climate::save_setpoint(float value) {
  optional<float> prev = this->load_setpoint();
  // Only save if value is diff from what's already saved.
  if (abs(value - prev.value_or(0.0)) >= SETPOINT_STEP) {
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
  return static_cast<float>(stored_val) / 10.0;
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

bool DaikinS21Climate::should_check_setpoint(climate::ClimateMode mode) {
  bool mode_uses_setpoint = mode == climate::CLIMATE_MODE_HEAT_COOL ||
                            mode == climate::CLIMATE_MODE_COOL ||
                            mode == climate::CLIMATE_MODE_HEAT;
  bool skip_check = false;
  if (this->skip_setpoint_checks > 0) {
    this->skip_setpoint_checks--;
    skip_check = true;
  }
  bool min_passed =
      this->setpoint_interval == 0 || this->last_setpoint_check == 0 ||
      (millis() - this->last_setpoint_check > (this->setpoint_interval * 1000));
  return mode_uses_setpoint && !skip_check && min_passed;
}

void DaikinS21Climate::update() {
  if (this->use_room_sensor()) {
    ESP_LOGD(TAG, "Room temp from external sensor: %.1f %s (%.1f °C)",
             this->room_sensor_->get_state(),
             this->room_sensor_->get_unit_of_measurement().c_str(),
             this->room_sensor_degc());
    ESP_LOGD(TAG, "  Offset: %.1f", this->get_room_temp_offset());
  }
  if (this->s21->is_ready()) {
    this->mode = this->s21->get_climate_mode();
    this->action = this->s21->get_climate_action();
    this->set_custom_fan_mode_(daikin_fan_mode_to_string_ref(this->s21->get_fan_mode()).str());
    this->swing_mode = this->s21->get_swing_mode();
    this->current_temperature = this->get_effective_current_temperature();
    this->current_humidity = this->s21->get_humidity();

    if (this->should_check_setpoint(this->mode)) {
      this->last_setpoint_check = millis();
      // Target temperature is stored by climate class, and is used to represent
      // the user's desired temperature. This is distinct from the HVAC unit's
      // setpoint because we may be using an external sensor. So we only update
      // the target temperature here if it appears uninitialized.
      float current_s21_sp = this->s21->get_setpoint();
      float unexpected_diff = abs(this->expected_s21_setpoint - current_s21_sp);
      if (this->target_temperature == 0.0 || isnanf(this->target_temperature)) {
        // Use stored setpoint for mode, or fall back to use s21's setpoint.
        auto stored = this->load_setpoint();
        this->target_temperature = stored.value_or(current_s21_sp);
        this->set_s21_climate();
      } else if (unexpected_diff >= SETPOINT_STEP) {
        // User probably set temp via IR remote -- so try to honor their wish by
        // matching controller's target value to what they sent via remote.
        ESP_LOGI(TAG, "S21 setpoint changed outside controller");
        ESP_LOGI(TAG, "  Expected: %.1f", this->expected_s21_setpoint);
        ESP_LOGI(TAG, "  Found: %.1f", current_s21_sp);
        this->target_temperature = current_s21_sp;
        ESP_LOGI(TAG, "  Target temp updated to %.1f", current_s21_sp);
        this->set_s21_climate();
      } else if (this->s21_setpoint_variance() >= SETPOINT_STEP) {
        // Room temperature offset has probably changed, so we need to adjust
        // the s21 setpoint based on the new difference.
        this->set_s21_climate();
        ESP_LOGI(TAG, "S21 setpoint updated to %.1f",
                 this->expected_s21_setpoint);
      }
    }

    this->publish_state();
  }
}

void DaikinS21Climate::control(const climate::ClimateCall &call) {
  bool set_basic = false;

  if (call.get_mode().has_value()) {
    climate::ClimateMode climate_mode = call.get_mode().value();
    if (this->mode != climate_mode) {
      this->mode = climate_mode;
      set_basic = true;
      if (!call.get_target_temperature().has_value()) {
        // If call does not include target, then try to use saved target.
        optional<float> sp = this->load_setpoint();
        if (sp.has_value()) {
          this->target_temperature = nearest_step(sp.value());
        }
      }
    }
  }
  if (call.get_target_temperature().has_value()) {
    this->target_temperature =
        nearest_step(call.get_target_temperature().value());
    set_basic = true;
  }
  if (call.get_custom_fan_mode().has_value()) {
    this->custom_fan_mode = call.get_custom_fan_mode().value();
    set_basic = true;
  }

  if (set_basic) {
    this->set_s21_climate();
  }

  if (call.get_swing_mode().has_value()) {
    this->s21->set_swing_settings(call.get_swing_mode().value());
  }

  this->update();
}

void DaikinS21Climate::set_s21_climate() {
  this->expected_s21_setpoint = this->calc_s21_setpoint(this->target_temperature);
  ESP_LOGI(TAG, "Controlling S21 climate:");
  ESP_LOGI(TAG, "  Mode: %s", LOG_STR_ARG(climate::climate_mode_to_string(this->mode)));
  ESP_LOGI(TAG, "  Setpoint: %.1f (s21: %.1f)", this->target_temperature,
          this->expected_s21_setpoint);
  ESP_LOGI(TAG, "  Fan: %s", this->custom_fan_mode.value().c_str());

  this->s21->set_climate_settings(
      this->mode,
      this->expected_s21_setpoint,
      string_to_daikin_fan_mode(this->custom_fan_mode.value()));
  // HVAC unit seems to take a few seconds to begin reporting mode and setpoint
  // changes back to the controller, so when modifying settings, setpoint checks
  // are skipped to avoid unexpected setpoint updates, especially when changing
  // modes.
  this->skip_setpoint_checks = 2;
  this->save_setpoint(this->target_temperature);
}

}  // namespace daikin_s21
}  // namespace esphome
