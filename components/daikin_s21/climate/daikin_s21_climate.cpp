#include <cmath>
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "daikin_s21_climate.h"
#include "../s21.h"

using namespace esphome;

namespace esphome::daikin_s21 {

static const char * const TAG = "daikin_s21.climate";

/**
 * Save target for the mode to persistent storage.
*
 * Only save if value is different from what's already saved. Some platforms don't support this internally.
 */
void DaikinSetpointMode::save_target(const DaikinC10 value) {
  if (value != this->load_target()) {
    const int16_t save_val = static_cast<int16_t>(value);
    this->target_pref.save(&save_val);
  }
}

/**
 * Load target for the mode from persistent storage.
 */
DaikinC10 DaikinSetpointMode::load_target() {
  int16_t load_val{};
  if (this->target_pref.load(&load_val)) {
    return load_val;
  }
  return TEMPERATURE_INVALID;
}

void DaikinS21Climate::setup() {
  uint32_t h = this->get_object_id_hash();
  this->heat_cool_params.target_pref = global_preferences->make_preference<int16_t>(h + 1);
  this->cool_params.target_pref = global_preferences->make_preference<int16_t>(h + 2);
  this->heat_params.target_pref = global_preferences->make_preference<int16_t>(h + 3);
  // populate default traits
  this->traits_.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE | climate::CLIMATE_SUPPORTS_ACTION);
#ifdef USE_CLIMATE_VISUAL_OVERRIDES
  if (std::isfinite(this->visual_min_temperature_override_)) {
    this->traits_.set_visual_min_temperature(this->visual_min_temperature_override_);
  } else
#endif
  {
    this->traits_.set_visual_min_temperature(std::min({this->heat_cool_params.min, this->cool_params.min, this->heat_params.min}).f_degc());
  }
#ifdef USE_CLIMATE_VISUAL_OVERRIDES
  if (std::isfinite(this->visual_max_temperature_override_)) {
    this->traits_.set_visual_max_temperature(this->visual_max_temperature_override_);
  } else
#endif
  {
    this->traits_.set_visual_max_temperature(std::max({this->heat_cool_params.max, this->cool_params.max, this->heat_params.max}).f_degc());
  }
#ifdef USE_CLIMATE_VISUAL_OVERRIDES
  if (std::isfinite(this->visual_target_temperature_step_override_)) {
    this->traits_.set_visual_target_temperature_step(this->visual_target_temperature_step_override_);
  } else
#endif
  {
    this->traits_.set_visual_target_temperature_step(SETPOINT_STEP.f_degc());
  }
#ifdef USE_CLIMATE_VISUAL_OVERRIDES
  if (std::isfinite(this->visual_current_temperature_step_override_)) {
    this->traits_.set_visual_current_temperature_step(this->visual_current_temperature_step_override_);
  } else
#endif
  {
    this->traits_.set_visual_current_temperature_step(TEMPERATURE_STEP.f_degc());
  }
  this->traits_.set_supported_fan_modes({climate::CLIMATE_FAN_AUTO, climate::CLIMATE_FAN_QUIET});
  this->traits_.set_supported_custom_fan_modes({
      daikin_fan_mode_to_cstr(DaikinFan1),
      daikin_fan_mode_to_cstr(DaikinFan2),
      daikin_fan_mode_to_cstr(DaikinFan3),
      daikin_fan_mode_to_cstr(DaikinFan4),
      daikin_fan_mode_to_cstr(DaikinFan5),
  });
  // ensure optionals are populated with defaults
  this->set_fan_mode_(climate::CLIMATE_FAN_AUTO);
  // initialize setpoint, will be loaded from preferences or unit shortly
  this->target_temperature = NAN;
  // enable event driven updates
  this->get_parent()->update_callbacks.add([this](){ this->enable_loop_soon_any_context(); }); // enable update events from DaikinS21
  this->disable_loop(); // wait for updates
}

/**
 * ESPHome Component loop
 *
 * Deferred work when an update occurs.
 * Recalculates the internal setpoint.
 * Publishes any state changes to Home Assistant.
 */
void DaikinS21Climate::loop() {
  const auto reported_climate = this->get_parent()->get_climate();
  const auto reported_swing = this->get_parent()->get_swing_mode();
  bool do_publish = false;
  bool update_unit_setpoint = false;

  // If the reported state differs from the component state, update component and publish an update
  const float new_temperature = this->get_current_temperature().f_degc();
  const float new_humidity = this->get_current_humidity();
  if ((this->mode != reported_climate.mode) ||
      (this->action != this->get_parent()->get_climate_action()) ||
      (std::isfinite(this->current_temperature) != std::isfinite(new_temperature)) || // differ in finite-ness
      (std::isfinite(this->current_temperature) && (this->current_temperature != new_temperature)) || // differ in finite value
      (std::isfinite(this->current_humidity) != std::isfinite(new_humidity)) ||
      (std::isfinite(this->current_humidity) && (this->current_humidity != new_humidity)) ||
      (this->swing_mode != reported_swing)) {
    this->mode = reported_climate.mode;
    this->action = this->get_parent()->get_climate_action();
    this->current_temperature = new_temperature;
    this->current_humidity = new_humidity;
    this->swing_mode = reported_swing;
    do_publish = true;
  }
  if (this->set_daikin_fan_mode(reported_climate.fan)) {
    do_publish = true;
  }

  // Update target temperature (user's desire) and unit setpoint (after offset)
  if (auto * const mode_params = this->get_setpoint_mode_params(this->mode)) {
    // Initialize setpoint so chenge detection can work
    if (this->unit_setpoint == TEMPERATURE_INVALID) {
      this->unit_setpoint = reported_climate.setpoint;
    }

    // Determine a new target temperature?
    if ((std::isfinite(this->target_temperature) == false) || // controller init or external mode change to a setpoint mode
        (this->unit_setpoint != reported_climate.setpoint)) { // external change to setpoint
      // Assume the reported setpoint (external IR remote change) should be the target temperature
      auto new_target = reported_climate.setpoint;
      // When first initializing, we don't know if the reported setpoint is from the IR remote or an offset value from a
      // previous ESPHome run. Use the saved target to resolve this once and in the future we can trust that we have set
      // target_temperature and unit_setpoint when commanding the unit and so any changes must be from the IR remote.
      if (this->target_resolved == false) {
        this->target_resolved = true;
        const auto saved_target = mode_params->load_target();
        if (saved_target != TEMPERATURE_INVALID) {
          new_target = saved_target;
        }
      }
      ESP_LOGI(TAG, "Target temperature changed: %.1f -> %.1f",
          this->target_temperature, new_target.f_degc());
      this->target_temperature = new_target.f_degc();
      this->unit_setpoint = reported_climate.setpoint;  // will be recalculated shortly, but ensure that log statement is sensical
      do_publish = true;
      update_unit_setpoint = true;
    }

    // Recalculate the unit setpoint if the target temperature changed or it's time to recheck
    if (update_unit_setpoint || this->is_free_run() || this->check_setpoint) {
      this->check_setpoint = false;
      // Reuse this flag to mean the setpoint has been updated and should be sent
      update_unit_setpoint = this->calc_unit_setpoint();
    }
  } else {
    // Not a setpoint mode
    // No previous target to recover
    this->target_resolved = true;
    // Clear setpoints and publish
    if (std::isfinite(this->target_temperature)) {
      this->target_temperature = NAN;
      this->unit_setpoint = TEMPERATURE_INVALID;
      do_publish = true;
    }
  }

  // Publish when state changed
  if (do_publish) {
    this->publish_state();
  }
  // Command unit when setpoint changed
  if (update_unit_setpoint) {
    this->set_s21_climate();
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

void DaikinS21Climate::dump_config() {
  LOG_CLIMATE("", "Daikin S21 Climate", this);
  LOG_SENSOR("  ", "Temperature Reference", this->temperature_sensor_);
  if ((this->temperature_sensor_ != nullptr) && (this->temperature_sensor_unit_is_valid() == false)) {
    ESP_LOGCONFIG(TAG, "  TEMPERATURE SENSOR: INVALID UNIT '%s' (must be °C or °F)",
        this->temperature_sensor_->get_unit_of_measurement_ref().c_str());
  }
  LOG_SENSOR("  ", "Humidity Reference", this->humidity_sensor_);
  if ((this->humidity_sensor_ != nullptr) && (this->humidity_sensor_->get_unit_of_measurement_ref() != "%")) {
    ESP_LOGCONFIG(TAG, "  HUMIDITY SENSOR: INVALID UNIT '%s' (must be %%)",
        this->humidity_sensor_->get_unit_of_measurement_ref().c_str());
  }
  ESP_LOGCONFIG(TAG, "  Setpoint interval: %" PRIu32 "s", this->get_update_interval() / 1000);
  for (const climate::ClimateMode mode : {climate::CLIMATE_MODE_HEAT_COOL, climate::CLIMATE_MODE_COOL, climate::CLIMATE_MODE_HEAT}) {
    if (const auto * const params = get_setpoint_mode_params(mode)) {
      ESP_LOGCONFIG(TAG, "  %s parameters\n"
                         "    Unit setpoint range: %.1f-%.1f\n"
                         "    User offset: %+.1f",
          LOG_STR_ARG(climate::climate_mode_to_string(mode)), params->min.f_degc(), params->max.f_degc(), params->offset.f_degc());
    }
  }
  this->dump_traits_(TAG);
}

/**
 * Override supported modes
 *
 * @note Modifies traits, call during setup only
 */
void DaikinS21Climate::set_supported_modes(const climate::ClimateModeMask modes) {
  this->traits_.set_supported_modes(modes);
}

/**
 * Override supported swing modes
 *
 * @note Modifies traits, call during setup only
 */
void DaikinS21Climate::set_supported_swing_modes(const climate::ClimateSwingModeMask swing_modes) {
  this->traits_.set_supported_swing_modes(swing_modes);
  this->get_parent()->request_readout(DaikinS21::ReadoutSwingHumidty);
}

/**
 * Set the humidity sensor used to report the climate humidity.
 *
 * @note Modifies traits, call during setup only
 */
void DaikinS21Climate::set_humidity_reference_sensor(sensor::Sensor * const sensor) {
  this->traits_.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_HUMIDITY);
  this->humidity_sensor_ = sensor;
}

/**
 * Set parameters for a given setpoint mode.
 */
void DaikinS21Climate::set_setpoint_mode_config(const climate::ClimateMode mode, const DaikinC10 offset, const DaikinC10 min, const DaikinC10 max) {
  if (auto * const mode_params = this->get_setpoint_mode_params(mode)) {
    mode_params->offset = offset;
    mode_params->min = min;
    mode_params->max = max;
  }
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
 * Determine the unit setpoint value based on the current temperature and target temperature.
 *
 * Applies offsets from the external reference sensor and user correction if present.
 *
 * @return true if the setpoint changed, false otherwise
 */
bool DaikinS21Climate::calc_unit_setpoint() {
  // First ensure we're in a setpoint mode
  auto * const mode_params = this->get_setpoint_mode_params(this->mode);
  if (mode_params == nullptr) {
    this->unit_setpoint = TEMPERATURE_INVALID;
    return false;
  }

  // Find the difference between the unit and reference sensor (0 if the same sensor)
  const auto current_temperature = this->get_current_temperature();
  const auto unit_temperature = this->get_parent()->get_temp_inside();
  const auto sensor_offset = unit_temperature - current_temperature;

  // Find the ideal unit setpoint by applying the sensor and user correction offsets
  auto new_unit_setpoint = static_cast<DaikinC10>(this->target_temperature) + sensor_offset + mode_params->offset;

  // Round to Daikin's internal setpoint resolution
  // When the ideal setpoint is between steps force it in the direction of change by controlling rounding. Over time it should oscillate over the ideal setpoint.
  if (new_unit_setpoint < unit_temperature) {
    // commanding the unit lower, round down
  } else if (new_unit_setpoint > unit_temperature) {
    // commanding the unit higher, round up by adding almost a full step
    new_unit_setpoint = new_unit_setpoint + (SETPOINT_STEP - 1);
  } else {
    // no difference, no rounding necessary
  }
  new_unit_setpoint = (new_unit_setpoint / SETPOINT_STEP) * SETPOINT_STEP;  // complete round by truncating fractional component of step

  // Ensure it's valid for the unit's current mode.
  // Daikin will clamp internally with a slightly out of range value, but it's faster for the UI to do it here without waiting for comms
  // Also, when large offsets are used, the value can be so far out of range it will be NAK'd
  new_unit_setpoint = std::clamp(new_unit_setpoint, mode_params->min, mode_params->max);

  // Log results if changing
  const bool unit_setpoint_changed = (this->unit_setpoint != new_unit_setpoint);
  if (unit_setpoint_changed) {
    ESP_LOGI(TAG, "Unit setpoint recalculated: %.1f -> %.1f%+.1f%+.1f = %.1f",
        this->unit_setpoint.f_degc(), this->target_temperature, sensor_offset.f_degc(), mode_params->offset.f_degc(), new_unit_setpoint.f_degc());
    this->unit_setpoint = new_unit_setpoint;
  }

  return unit_setpoint_changed;
}

/**
 * ESPHome climate control call handler.
 *
 * Populates internal state with contained arguments then applies to the unit.
 */
void DaikinS21Climate::control(const climate::ClimateCall &call) {
  // DaikinClimateSettings changes
  bool climate_changed = false;
  if (call.get_mode().has_value() && (this->mode != call.get_mode().value())) {
    this->mode = call.get_mode().value();
    climate_changed = true;
    // If call sets the mode but does not include target, then try to use saved target.
    if (call.get_target_temperature().has_value() == false) {
      if (auto * const mode_params = this->get_setpoint_mode_params(this->mode)) {
        const auto sp = mode_params->load_target();
        if (sp != TEMPERATURE_INVALID) {
          this->target_temperature = sp.f_degc();
        }
      } else {
        this->target_temperature = NAN; // Clear setpoint if not in a setpoint mode
      }
    }
  }
  if (call.get_target_temperature().has_value() && (this->target_temperature != call.get_target_temperature().value())) {
    this->target_temperature = call.get_target_temperature().value();
    climate_changed = true;
  }
  if (climate_changed) {
    (void)this->calc_unit_setpoint();  // mode and target temperature required
  }
  if (call.get_fan_mode().has_value()) {
    if (this->set_fan_mode_(call.get_fan_mode().value())) {
      climate_changed = true;
    }
  } else if (call.has_custom_fan_mode()) {
    if (this->set_custom_fan_mode_(call.get_custom_fan_mode())) {
      climate_changed = true;
    }
  }
  if (climate_changed) {
    this->set_s21_climate();  // mode, unit setpoint and fan required
  }

  // climate::ClimateSwingMode changes
  if (call.get_swing_mode().has_value() && (this->swing_mode != call.get_swing_mode().value())) {
    this->swing_mode = call.get_swing_mode().value();
    this->get_parent()->set_swing_mode(this->swing_mode);
  }

  // push back to UI
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

DaikinFanMode DaikinS21Climate::get_daikin_fan_mode() const {
  if (this->fan_mode.has_value()) {
    if (this->fan_mode.value() == climate::CLIMATE_FAN_QUIET) {
      return DaikinFanSilent;
    } else {
      return DaikinFanAuto;
    }
  } else {
    return stringref_to_daikin_fan_mode(this->get_custom_fan_mode());
  }
}

bool DaikinS21Climate::set_daikin_fan_mode(const DaikinFanMode fan) {
  if (fan == DaikinFanAuto) {
    return this->set_fan_mode_(climate::CLIMATE_FAN_AUTO);
  } else if (fan == DaikinFanSilent) {
    return this->set_fan_mode_(climate::CLIMATE_FAN_QUIET);
  } else {
    return this->set_custom_fan_mode_(daikin_fan_mode_to_cstr(fan));
  }
}

/**
 * Apply ESPHome Climate state to the unit.
 *
 * Converts to internal settings format and forwards to DaikinS21 component to apply.
 */
void DaikinS21Climate::set_s21_climate() {
  // Command new settings
  this->get_parent()->set_climate_settings({this->mode, this->get_daikin_fan_mode(), this->unit_setpoint});
  if (auto * const mode_params = this->get_setpoint_mode_params(this->mode)) {
    mode_params->save_target(this->target_temperature);
  }
}

/**
 * Get the parameters associated with the setpoint mode, nullptr if not a setpoint mode.
 */
DaikinSetpointMode* DaikinS21Climate::get_setpoint_mode_params(climate::ClimateMode mode) {
  switch (mode) {
    case climate::CLIMATE_MODE_HEAT_COOL:
      return &this->heat_cool_params;
      break;
    case climate::CLIMATE_MODE_COOL:
    return &this->cool_params;
      break;
    case climate::CLIMATE_MODE_HEAT:
    return &this->heat_params;
      break;
    default:
      return nullptr;
  }
}

} // namespace esphome::daikin_s21
