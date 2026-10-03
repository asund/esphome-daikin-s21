#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"
#include "../daikin_s21_types.h"

namespace esphome::daikin_s21 {

/**
 * Finite setpoint mode parameters and storage helper.
 */
class DaikinSetpointMode {
 public:
  ESPPreferenceObject target_pref{};
  DaikinC10 offset{};
  DaikinC10 min{};
  DaikinC10 max{};

  void save_target(DaikinC10 value);
  DaikinC10 load_target();
  static_assert(std::is_trivially_copyable_v<DaikinC10>, "persisted verbatim to flash");
};

/**
 * Climate component change tracking structure.
 */
struct DaikinS21ClimateChanges {
  constexpr DaikinS21ClimateChanges& operator|=(const DaikinS21ClimateChanges &other) {
    this->internal = this->internal || other.internal;
    this->external = this->external || other.external;
    return *this;
  }

  bool internal{};  /**< Internal change, Daikin unit should be commanded to new climate values. */
  bool external{};  /**< External change, new climate values should be published to Home Assistant. */
};

class DaikinS21Climate : public climate::Climate,
                         public PollingComponent,
                         public Parented<DaikinS21> {
 public:
  void setup() final;
  void loop() final;
  void update() final { this->check_sensors = true; };
  void dump_config() final;
  void control(const climate::ClimateCall &call) final;

  void set_offset_interval(uint32_t offset_interval);
  void set_setpoint_dither(const bool dither) { this->setpoint_dither = dither; }
  void set_supported_modes(climate::ClimateModeMask modes);
  void set_supported_swing_modes(climate::ClimateSwingModeMask swing_modes);
  void set_temperature_reference_sensor(sensor::Sensor * const sensor) { this->temperature_sensor_ = sensor; }
  void set_enable_presets(bool enable);
  void set_humidity_reference_sensor(sensor::Sensor * sensor);
  void set_setpoint_mode_config(climate::ClimateMode mode, DaikinC10 offset, DaikinC10 min, DaikinC10 max);

 protected:
  climate::ClimateTraits traits_{};
  climate::ClimateTraits traits() final { return traits_; };

  bool is_free_run() const { return this->get_update_interval() == SCHEDULER_DONT_RUN; }
  bool temperature_sensor_unit_is_valid();
  DaikinC10 get_current_temperature();
  bool calc_unit_setpoint(const DaikinSetpointMode &mode_params, DaikinC10 current_temperature);
  constexpr DaikinS21ClimateChanges synchronize_special_setpoint(const DaikinC10 setpoint) {
    const DaikinS21ClimateChanges changes{(this->unit_setpoint != setpoint), std::isfinite(this->target_temperature)};
    this->unit_setpoint = setpoint;
    this->target_temperature = NAN;
    return changes;
  }
  float get_current_humidity() const;
  DaikinFanMode get_daikin_fan_mode() const;
  bool set_daikin_fan_mode(DaikinFanMode fan);
  DaikinPreset get_daikin_preset() const;
  bool set_daikin_preset(DaikinPreset preset);
  void handle_climate_changes(DaikinS21ClimateChanges changes);

  sensor::Sensor *temperature_sensor_{};
  sensor::Sensor *humidity_sensor_{};
  DaikinC10 unit_setpoint{};
  bool setpoint_dither{true};
  bool check_sensors{true};
  bool check_offset{true};
  bool freerun_offset{};

  struct SetpointModeParams {
    DaikinSetpointMode cool{};
    DaikinSetpointMode heat{};
    DaikinSetpointMode heat_cool{};

    DaikinSetpointMode* get(climate::ClimateMode mode);
  } setpoint_params;
};

} // namespace esphome::daikin_s21
