#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "../daikin_s21_types.h"
#include "../s21.h"

namespace esphome::daikin_s21 {

class DaikinS21Sensor : public PollingComponent,
                        public Parented<DaikinS21> {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;

  void publish_sensors();

  void set_energy_sensor(sensor::Sensor * const sensor) {
    this->energy_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutEnergyConsumptionTotal);
  }
  void set_energy_cooling_sensor(sensor::Sensor * const sensor) {
    this->energy_cooling_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutEnergyConsumptionClimateModes);
  }
  void set_energy_heating_sensor(sensor::Sensor * const sensor) {
    this->energy_heating_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutEnergyConsumptionClimateModes);
  }
  void set_temp_setpoint_sensor(sensor::Sensor * const sensor) {
    this->temp_setpoint_sensor_ = sensor;
  }
  void set_temp_inside_sensor(sensor::Sensor * const sensor) {
    this->temp_inside_sensor_ = sensor;
  }
  void set_temp_target_sensor(sensor::Sensor * const sensor) {
    this->temp_target_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutTemperatureTarget);
  }
  void set_temp_outside_sensor(sensor::Sensor * const sensor) {
    this->temp_outside_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutTemperatureOutside);
  }
  void set_temp_coil_sensor(sensor::Sensor * const sensor) {
    this->temp_coil_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutTemperatureCoil);
  }
  void set_fan_speed_sensor(sensor::Sensor * const sensor) {
    this->fan_speed_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutFanSpeed);
  }
  void set_swing_vertical_angle_sensor(sensor::Sensor * const sensor) {
    this->swing_vertical_angle_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutSwingAngle);
  }
  void set_compressor_frequency_sensor(sensor::Sensor * const sensor) {
    this->compressor_frequency_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutCompressorFrequency);
  }
  void set_humidity_sensor(sensor::Sensor * const sensor) {
    this->humidity_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutHumidity);
  }
  void set_demand_sensor(sensor::Sensor * const sensor) {
    this->demand_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutDemand);
  }
  void set_ir_counter_sensor(sensor::Sensor * const sensor) {
    this->ir_counter_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutIRCounter);
  }
  void set_outdoor_capacity_sensor(sensor::Sensor * const sensor) {
    this->outdoor_capacity_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutOutdoorCapacity);
  }

 protected:
  bool is_free_run() const { return this->get_update_interval() == 0; }

  sensor::Sensor *energy_sensor_{};
  sensor::Sensor *energy_cooling_sensor_{};
  sensor::Sensor *energy_heating_sensor_{};
  sensor::Sensor *temp_setpoint_sensor_{};
  sensor::Sensor *temp_inside_sensor_{};
  sensor::Sensor *temp_target_sensor_{};
  sensor::Sensor *temp_outside_sensor_{};
  sensor::Sensor *temp_coil_sensor_{};
  sensor::Sensor *fan_speed_sensor_{};
  sensor::Sensor *swing_vertical_angle_sensor_{};
  sensor::Sensor *compressor_frequency_sensor_{};
  sensor::Sensor *humidity_sensor_{};
  sensor::Sensor *demand_sensor_{};
  sensor::Sensor *ir_counter_sensor_{};
  sensor::Sensor *outdoor_capacity_sensor_{};
};

} // namespace esphome::daikin_s21
