#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"
#include "../daikin_s21_types.h"

namespace esphome::daikin_s21 {

class DaikinSetpointMode {
 public:
  ESPPreferenceObject setpoint_pref{};
  DaikinC10 offset{};
  DaikinC10 min{};
  DaikinC10 max{};

  void save_setpoint(DaikinC10 value);
  DaikinC10 load_setpoint();
};

class DaikinS21Climate : public climate::Climate,
                         public PollingComponent,
                         public Parented<DaikinS21> {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  void control(const climate::ClimateCall &call) override;

  void set_supported_modes(climate::ClimateModeMask modes);
  void set_supported_swing_modes(climate::ClimateSwingModeMask swing_modes);
  void set_temperature_reference_sensor(sensor::Sensor *sensor) { this->temperature_sensor_ = sensor; }
  void set_humidity_reference_sensor(sensor::Sensor *sensor);
  void set_setpoint_mode_config(climate::ClimateMode mode, DaikinC10 offset, DaikinC10 min, DaikinC10 max);

 protected:
  climate::ClimateTraits traits_{};
  climate::ClimateTraits traits() override { return traits_; };

  bool is_free_run() const { return this->get_update_interval() == 0; }
  bool temperature_sensor_unit_is_valid();
  bool use_temperature_sensor();
  DaikinC10 temperature_sensor_degc();
  DaikinC10 get_current_temperature();
  bool calc_unit_setpoint();
  float get_current_humidity() const;
  DaikinFanMode get_daikin_fan_mode() const;
  bool set_daikin_fan_mode(DaikinFanMode fan);
  void set_s21_climate();

  sensor::Sensor *temperature_sensor_{};
  sensor::Sensor *humidity_sensor_{};
  DaikinC10 unit_setpoint{TEMPERATURE_INVALID};
  bool check_setpoint{};

  DaikinSetpointMode* get_setpoint_mode_params(climate::ClimateMode mode);
  DaikinSetpointMode heat_cool_params{};
  DaikinSetpointMode cool_params{};
  DaikinSetpointMode heat_params{};
};

} // namespace esphome::daikin_s21
