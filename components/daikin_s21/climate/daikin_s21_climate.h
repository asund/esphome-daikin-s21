#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"
#include "../daikin_s21_fan_modes.h"
#include "../daikin_s21_types.h"

namespace esphome::daikin_s21 {

class DaikinS21Climate : public climate::Climate,
                         public PollingComponent,
                         public Parented<DaikinS21> {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  void control(const climate::ClimateCall &call) override;

  void set_supported_modes(std::set<climate::ClimateMode> modes);
  void set_supported_presets(std::set<climate::ClimatePreset> presets);
  void set_temperature_reference_sensor(sensor::Sensor *sensor) { this->temperature_sensor_ = sensor; }
  void set_humidity_reference_sensor(sensor::Sensor *sensor);
  void set_max_cool_temperature(DaikinC10 temperature) { this->max_cool_temperature = temperature; };
  void set_min_cool_temperature(DaikinC10 temperature) { this->min_cool_temperature = temperature; };
  void set_max_heat_temperature(DaikinC10 temperature) { this->max_heat_temperature = temperature; };
  void set_min_heat_temperature(DaikinC10 temperature) { this->min_heat_temperature = temperature; };

 protected:
  static constexpr const char * command_timeout_name = "cmd";
  static constexpr uint32_t state_publication_timeout_ms{8 * 1000}; // experimentally determined with fudge factor

  climate::ClimateTraits traits_{};
  climate::ClimateTraits traits() override { return traits_; };

  sensor::Sensor *temperature_sensor_{};
  sensor::Sensor *humidity_sensor_{};
  DaikinC10 max_cool_temperature{};
  DaikinC10 min_cool_temperature{};
  DaikinC10 max_heat_temperature{};
  DaikinC10 min_heat_temperature{};

  bool command_active{};  // ESPHome could use a is_timeout_active()...
  bool check_setpoint{};
  DaikinClimateSettings commanded{};

  ESPPreferenceObject auto_setpoint_pref;
  ESPPreferenceObject cool_setpoint_pref;
  ESPPreferenceObject heat_setpoint_pref;

  void set_custom_fan_mode(DaikinFanMode mode);
  bool temperature_sensor_unit_is_valid();
  bool use_temperature_sensor();
  DaikinC10 temperature_sensor_degc();
  DaikinC10 get_current_temperature();
  DaikinC10 calc_s21_setpoint();
  void save_setpoint(DaikinC10 value);
  DaikinC10 load_setpoint();
  float get_current_humidity() const;
  void set_s21_climate();
  void command_timeout_handler();
};

} // namespace esphome::daikin_s21
