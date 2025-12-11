#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "../daikin_s21_types.h"
#include "../s21.h"

namespace esphome::daikin_s21 {

class DaikinS21BinarySensor : public Component,
                              public Parented<DaikinS21> {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_powerful_sensor(binary_sensor::BinarySensor *sensor) {
    this->powerful_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutUnitStateBits); // for units without special modes, use a switch if supported
  }
  void set_defrost_sensor(binary_sensor::BinarySensor *sensor) {
    this->defrost_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutUnitStateBits);
  }
  void set_active_sensor(binary_sensor::BinarySensor *sensor) {
    this->active_sensor_ = sensor;
  }
  void set_online_sensor(binary_sensor::BinarySensor *sensor) {
    this->online_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutUnitStateBits);
  }
  void set_valve_sensor(binary_sensor::BinarySensor *sensor) {
    this->valve_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutSystemStateBits);
  }
  void set_short_cycle_sensor(binary_sensor::BinarySensor *sensor) {
    this->short_cycle_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutSystemStateBits);
  }
  void set_system_defrost_sensor(binary_sensor::BinarySensor *sensor) {
    this->system_defrost_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutSystemStateBits);
  }
  void set_multizone_conflict_sensor(binary_sensor::BinarySensor *sensor) {
    this->multizone_conflict_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutSystemStateBits);
  }
  void set_serial_error_sensor(binary_sensor::BinarySensor *sensor) {
    this->serial_error_sensor_ = sensor;
  }

 protected:
  binary_sensor::BinarySensor *powerful_sensor_{};
  binary_sensor::BinarySensor *defrost_sensor_{};
  binary_sensor::BinarySensor *active_sensor_{};
  binary_sensor::BinarySensor *online_sensor_{};
  binary_sensor::BinarySensor *valve_sensor_{};
  binary_sensor::BinarySensor *short_cycle_sensor_{};
  binary_sensor::BinarySensor *system_defrost_sensor_{};
  binary_sensor::BinarySensor *multizone_conflict_sensor_{};
  binary_sensor::BinarySensor *serial_error_sensor_{};
};

} // namespace esphome::daikin_s21
