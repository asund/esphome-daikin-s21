#include <cmath>
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "daikin_s21_climate.h"
#include "../daikin_s21_fan_modes.h"
#include "../s21.h"
#include "../utils.h"

using namespace esphome;

namespace esphome::daikin_s21 {

static const char *const TAG = "daikin_s21.climate";

constexpr bool is_setpoint_mode(const climate::ClimateMode mode) {
  return (mode == climate::CLIMATE_MODE_HEAT_COOL) ||
         (mode == climate::CLIMATE_MODE_COOL) ||
         (mode == climate::CLIMATE_MODE_HEAT);
}

void DaikinS21Climate::setup() {
  uint32_t h = this->get_object_id_hash();
  this->auto_setpoint_pref = global_preferences->make_preference<int16_t>(h + 1);
  this->cool_setpoint_pref = global_preferences->make_preference<int16_t>(h + 2);
  this->heat_setpoint_pref = global_preferences->make_preference<int16_t>(h + 3);
  // populate default traits
  this->traits_.set_supports_action(true);
  this->traits_.set_supports_current_temperature(true);
  if (this->visual_min_temperature_override_.has_value()) {
    this->traits_.set_visual_min_temperature(this->visual_min_temperature_override_.value());
  } else {
    this->traits_.set_visual_min_temperature(this->min_heat_temperature.f_degc());
  }
  if (this->visual_max_temperature_override_.has_value()) {
    this->traits_.set_visual_max_temperature(this->visual_max_temperature_override_.value());
  } else {
    this->traits_.set_visual_max_temperature(this->max_cool_temperature.f_degc());
  }
  if (this->visual_target_temperature_step_override_.has_value()) {
    this->traits_.set_visual_target_temperature_step(this->visual_target_temperature_step_override_.value());
  } else {
    this->traits_.set_visual_target_temperature_step(SETPOINT_STEP.f_degc());
  }
  if (this->visual_current_temperature_step_override_.has_value()) {
    this->traits_.set_visual_current_temperature_step(this->visual_current_temperature_step_override_.value());
  } else {
    this->traits_.set_visual_current_temperature_step(TEMPERATURE_STEP.f_degc());
  }
  std::array<std::string, std::size(supported_daikin_fan_modes)> supported_fan_mode_strings;
  std::ranges::transform(supported_daikin_fan_modes, supported_fan_mode_strings.begin(), [](const auto &arg){ return daikin_fan_mode_to_string_view(arg); } );
  this->traits_.set_supported_custom_fan_modes({supported_fan_mode_strings.begin(), supported_fan_mode_strings.end()});
  this->traits_.set_supported_swing_modes({
      climate::CLIMATE_SWING_OFF,
      climate::CLIMATE_SWING_BOTH,
      climate::CLIMATE_SWING_VERTICAL,
      climate::CLIMATE_SWING_HORIZONTAL,
  });
  // ensure optionals are populated with defaults
  this->set_custom_fan_mode(commanded.fan);
  this->preset = commanded.preset;
  // initialize setpoint, will be loaded from preferences or unit shortly
  this->target_temperature = NAN;
  // enable event driven updates
  this->get_parent()->update_callbacks.add([this](){ this->enable_loop_soon_any_context(); }); // enable update events from DaikinS21
  // allow loop() to execute once to capture the current state (change detection on update requires a "previous" state)
}

/**
 * ESPHome Component loop
 *
 * Deferred work when an update occurs.
 *
 * Update climate state if a previous command isn't in progress.
 * Publish any state changes to Home Assistant.
 */
void DaikinS21Climate::loop() {
  const auto& reported = this->get_parent()->get_climate_settings();

  // If an active command took effect, cancel the timeout and lift the publication ban
  if (this->command_active && (this->commanded == reported)) {
    this->command_active = false;
    this->cancel_timeout(command_timeout_name);
  }

  // Don't publish current state if a command is in progress -- avoids UI glitches
  if (this->command_active == false) {
    // Allowed to publish, determine if we should
    bool do_publish = false;
    bool update_unit_setpoint = false;

    // Detect and integrate new values into component state
    const float current_temperature = this->get_current_temperature().f_degc();
    const float current_humidity = this->get_current_humidity();
    if ((this->mode != reported.mode) ||
        (this->action != this->get_parent()->get_climate_action()) ||
        (this->current_temperature != current_temperature) ||
        (std::isfinite(current_humidity) && (this->current_humidity != current_humidity))||
        (this->swing_mode != reported.swing) ||
        (this->preset != reported.preset)) {
      this->mode = reported.mode;
      this->action = this->get_parent()->get_climate_action();
      this->current_temperature = current_temperature;
      this->current_humidity = current_humidity;
      this->swing_mode = reported.swing;
      this->preset = reported.preset;
      do_publish = true;
    }
    if (this->commanded.fan != reported.fan) {
      this->set_custom_fan_mode(reported.fan);   // avoid custom string operations until there's a change that requires publishing
      do_publish = true;
    }

    // Update component target temperature
    if (is_setpoint_mode(mode)) {
      // Target temperature is stored by climate class, and is used to represent
      // the user's desired temperature. This is distinct from the HVAC unit's
      // setpoint because we may be using an external sensor. So we only update
      // the target temperature here if it appears uninitialized.
      if (std::isnan(this->target_temperature)) {
        // Use stored setpoint for mode, or fall back to use s21's setpoint.
        const auto setpoint = this->load_setpoint();
        this->target_temperature = ((setpoint != TEMPERATURE_INVALID) ? setpoint : reported.setpoint).f_degc();
        do_publish = true;
        update_unit_setpoint = true;
      } else if (this->commanded.setpoint != reported.setpoint) {
        // User probably set temp via IR remote -- so try to honor their wish by
        // matching controller's target value to what they sent via remote.
        // This keeps the external temperature sensor in the loop.
        ESP_LOGI(TAG, "User setpoint changed, updating target temp (was %.1f, now %.1f)",
            this->commanded.setpoint.f_degc(), reported.setpoint.f_degc());
        this->target_temperature = reported.setpoint.f_degc();
        update_unit_setpoint = true;
      } else if (this->check_setpoint) {
        // Periodic adjustment of the Daikin setpoint to reflect changes
        // to the component's reference temperature.
        this->check_setpoint = false;
        if (DaikinC10::diff(reported.setpoint, this->calc_s21_setpoint()) >= SETPOINT_STEP) { // in setpoint mode, calculated won't be invalid
          if (this->use_temperature_sensor()) {
            ESP_LOGD(TAG, "Temperature from external sensor: %.1f %s (%.1f °C)  Offset: %.1f",
                this->temperature_sensor_->get_state(),
                this->temperature_sensor_->get_unit_of_measurement_ref().c_str(),
                current_temperature,
                this->get_parent()->get_temp_inside() - this->temperature_sensor_degc());
          }
          ESP_LOGI(TAG, "Reference offset changed, updating setpoint");
          update_unit_setpoint = true;
        }
      }
    } else {
      // Not a setpoint mode, clear it if set and publish
      if (std::isfinite(this->target_temperature)) {
        this->target_temperature = NAN;
        do_publish = true;
      }
    }

    // If the reported state differs from the last commanded value, update component state and publish an update
    if (this->commanded != reported) {
      this->commanded = reported;
      do_publish = true;
    }

    // Publish when state changed
    if (do_publish) {
      this->publish_state();
    }
    // Command unit when setpoint changed
    if (update_unit_setpoint) {
      this->set_s21_climate();
    }
  }

  this->disable_loop(); // wait for updates
}

/**
 * ESPHome PollingComponent loop
 *
 * Flag to check the setpoint on the next update from the unit
 * when applicable in the current climate mode.
 */
void DaikinS21Climate::update() {
  this->check_setpoint = true;
}

/**
 * Command timeout handler
 *
 * Called when a command times out without seeming to take effect.
 * Trigger a publish of the current state.
 */
void DaikinS21Climate::command_timeout_handler() {
  this->command_active = false;
  this->loop();
}

void DaikinS21Climate::dump_config() {
  ESP_LOGCONFIG(TAG, "DaikinS21Climate:");
  if (this->temperature_sensor_ != nullptr) {
    if (this->temperature_sensor_unit_is_valid() == false) {
      ESP_LOGCONFIG(TAG, "  TEMPERATURE SENSOR: INVALID UNIT '%s' (must be °C or °F)",
                    this->temperature_sensor_->get_unit_of_measurement_ref().c_str());
    } else {
      ESP_LOGCONFIG(TAG, "  Temperature sensor: %s",
                    this->temperature_sensor_->get_name().c_str());
    }
  }
  if (this->humidity_sensor_ != nullptr) {
    if ((this->humidity_sensor_->get_unit_of_measurement_ref() != "%")) {
      ESP_LOGCONFIG(TAG, "  HUMIDITY SENSOR: INVALID UNIT '%s' (must be %)",
                    this->humidity_sensor_->get_unit_of_measurement_ref().c_str());
    } else {
      ESP_LOGCONFIG(TAG, "  Humidity sensor: %s",
                    this->humidity_sensor_->get_name().c_str());
    }
  }
  ESP_LOGCONFIG(TAG, "  Setpoint interval: %" PRIu32 "s", this->get_update_interval() / 1000);
  this->dump_traits_(TAG);
}

/**
 * Override supported modes
 *
 * @note Modifies traits, call during setup only
 */
void DaikinS21Climate::set_supported_modes(std::set<climate::ClimateMode> modes) {
  this->traits_.set_supported_modes(modes);
}

/**
 * Override supported presets
 *
 * @note Modifies traits, call during setup only
 */
void DaikinS21Climate::set_supported_presets(std::set<climate::ClimatePreset> presets) {
  this->traits_.set_supported_presets(presets);
}

/**
 * Set the humidity sensor used to report the climate humidity.
 *
 * @note Modifies traits, call during setup only
 */
void DaikinS21Climate::set_humidity_reference_sensor(sensor::Sensor * const sensor) {
  this->traits_.set_supports_current_humidity(true);
  this->humidity_sensor_ = sensor;
}

void DaikinS21Climate::set_custom_fan_mode(const DaikinFanMode mode) {
  this->commanded.fan = mode;
  this->custom_fan_mode = static_cast<std::string>(daikin_fan_mode_to_string_view(mode));
}

bool DaikinS21Climate::temperature_sensor_unit_is_valid() {
  if (this->temperature_sensor_ != nullptr) {
    auto u = this->temperature_sensor_->get_unit_of_measurement_ref();
    return u == "°C" || u == "°F";
  }
  return false;
}

bool DaikinS21Climate::use_temperature_sensor() {
  return this->temperature_sensor_unit_is_valid() &&
         this->temperature_sensor_->has_state() &&
         std::isfinite(this->temperature_sensor_->get_state());
}

DaikinC10 DaikinS21Climate::temperature_sensor_degc() {
  float temp = this->temperature_sensor_->get_state();
  if (this->temperature_sensor_->get_unit_of_measurement_ref() == "°F") {
    temp = fahrenheit_to_celsius(temp);
  }
  return temp;
}

/**
 * Get the current temperature, either from the external reference or the Daikin unit.
 */
DaikinC10 DaikinS21Climate::get_current_temperature() {
  if (this->use_temperature_sensor()) {
    return this->temperature_sensor_degc();
  }
  return this->get_parent()->get_temp_inside();
}

/**
 * Determine the S21 setpoint value based on the current temperature and target temperature.
 *
 * Applies an offset from the external reference sensor if present.
 */
DaikinC10 DaikinS21Climate::calc_s21_setpoint() {
  // First ensure we're in a setpoint mode
  if (is_setpoint_mode(this->mode) == false) {
    return TEMPERATURE_INVALID;
  }

  const auto current_temperature = this->get_current_temperature();

  // offset the ideal setpoint by the difference between the unit and reference sensor (0 if the same sensor)
  auto s21_setpoint = static_cast<DaikinC10>(this->target_temperature) + (this->get_parent()->get_temp_inside() - current_temperature);

  // Round to Daikin's internal setpoint resolution
  // When the ideal setpoint is between steps force it in the direction of change by controlling rounding. Over time it should oscillate over the ideal setpoint.
  if (s21_setpoint < current_temperature) {
    // commanding the unit lower, round down
  } else if (s21_setpoint > current_temperature) {
    // commanding the unit higher, round up by adding almost a full step
    s21_setpoint = s21_setpoint + (SETPOINT_STEP - 1);
  } else {
    // no difference, round to nearest
    s21_setpoint = ((s21_setpoint * 2) + SETPOINT_STEP) / 2;  //add a half step (scaled for full precision)
  }
  s21_setpoint = (s21_setpoint / SETPOINT_STEP) * SETPOINT_STEP;  // complete round by truncating fractional component of step

  // Ensure it's valid for the unit's current mode.
  // Daikin will clamp internally with a slightly out of range value, but it's faster for the UI to do it here without waiting for comms
  // Also, when a large external temperature offset is used, the value can be so far out of range it will be NAK'd
  return std::clamp(s21_setpoint,
      (this->mode == climate::CLIMATE_MODE_HEAT) ? this->min_heat_temperature : this->min_cool_temperature,
      (this->mode == climate::CLIMATE_MODE_COOL) ? this->max_cool_temperature : this->max_heat_temperature);
}

void DaikinS21Climate::save_setpoint(const DaikinC10 value) {
  // Only save if value is different from what's already saved.
  if (value != this->load_setpoint()) {
    const int16_t save_val = static_cast<int16_t>(value);
    switch (this->mode) {
      case climate::CLIMATE_MODE_HEAT_COOL:
        this->auto_setpoint_pref.save(&save_val);
        break;
      case climate::CLIMATE_MODE_COOL:
        this->cool_setpoint_pref.save(&save_val);
        break;
      case climate::CLIMATE_MODE_HEAT:
        this->heat_setpoint_pref.save(&save_val);
        break;
      default:
        break;
    }
  }
}

DaikinC10 DaikinS21Climate::load_setpoint() {
  int16_t load_val{};
  switch (this->mode) {
    case climate::CLIMATE_MODE_HEAT_COOL:
      (void)this->auto_setpoint_pref.load(&load_val);
      break;
    case climate::CLIMATE_MODE_COOL:
      (void)this->cool_setpoint_pref.load(&load_val);
      break;
    case climate::CLIMATE_MODE_HEAT:
      (void)this->heat_setpoint_pref.load(&load_val);
      break;
    default:
      return TEMPERATURE_INVALID;
  }
  return load_val;
}

/**
 * ESPHome climate control call handler.
 *
 * Populates internal state with contained arguments then applies to the unit.
 */
void DaikinS21Climate::control(const climate::ClimateCall &call) {
  if (call.get_mode().has_value()) {
    this->mode = call.get_mode().value();
    // If call does not include target, then try to use saved target.
    if (call.get_target_temperature().has_value() == false) {
      const auto sp = this->load_setpoint();
      // Also clear setpoint if not in a setpoint mode
      if ((is_setpoint_mode(this->mode) == false) || (sp != TEMPERATURE_INVALID)) {
        this->target_temperature = sp.f_degc();
      }
    }
  }
  if (call.get_target_temperature().has_value()) {
    this->target_temperature = call.get_target_temperature().value();
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
 * Get the current humidity value from the optional sensor
 */
float DaikinS21Climate::get_current_humidity() const {
  if ((this->humidity_sensor_ != nullptr) &&
      (this->humidity_sensor_->get_unit_of_measurement_ref() == "%")) {
    return this->humidity_sensor_->get_state(); // NAN state is fine
  } else {
    return NAN;
  }
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

  // HVAC unit takes a few seconds to begin reporting settings changes back to
  // the controller, so when modifying don't publish current state until it
  // takes effect or is given enough time to take effect or we get a jumpy UI
  // in Home Assistant as stale state is published over the commanded state
  // followed by the active state.
  this->command_active = true;
  this->set_timeout(command_timeout_name,
                    DaikinS21Climate::state_publication_timeout_ms + this->get_parent()->get_update_interval(), // extend the timeour if S21 is polling to wait for slower updates
                    [this](){ this->command_timeout_handler(); });

  ESP_LOGI(TAG, "Controlling S21 climate:");
  ESP_LOGI(TAG, "  Mode: %s", LOG_STR_ARG(climate::climate_mode_to_string(this->commanded.mode)));
  ESP_LOGI(TAG, "  Setpoint: %.1f (s21: %.1f)", this->target_temperature, this->commanded.setpoint.f_degc());
  ESP_LOGI(TAG, "  Fan: %" PRI_SV, PRI_SV_ARGS(daikin_fan_mode_to_string_view(this->commanded.fan)));
  ESP_LOGI(TAG, "  Swing: %s", LOG_STR_ARG(climate::climate_swing_mode_to_string(this->commanded.swing)));
  ESP_LOGI(TAG, "  Preset: %s", LOG_STR_ARG(climate::climate_preset_to_string(this->commanded.preset)));

  this->save_setpoint(this->target_temperature);  // todo much slower interval?
}

} // namespace esphome::daikin_s21
