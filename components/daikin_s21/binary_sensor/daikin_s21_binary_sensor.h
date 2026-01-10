#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "../daikin_s21_types.h"
#include "../s21.h"

namespace esphome::daikin_s21 {

class DaikinS21BinarySensorMode : public binary_sensor::BinarySensor {
 public:
  DaikinS21BinarySensorMode(const DaikinMode mode) : mode(mode) {}
  DaikinMode mode;
};

class DaikinS21BinarySensor : public Component,
                              public Parented<DaikinS21> {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_mode_sensor(DaikinS21BinarySensorMode * const mode_sensor) {
    this->mode_sensors_[mode_sensor->mode] = mode_sensor;
    if (mode_sensor->mode == ModePowerful) {
      this->get_parent()->request_readout(DaikinS21::ReadoutPowerful);
    } else if (mode_sensor->mode == ModeEcono) {
      this->get_parent()->request_readout(DaikinS21::ReadoutDemandAndEcono);
    } else {
      this->get_parent()->request_readout(DaikinS21::ReadoutSpecialModes);
    }
  }
  void set_defrost_sensor(binary_sensor::BinarySensor * const sensor) {
    this->defrost_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutUnitStateBits);
  }
  void set_active_sensor(binary_sensor::BinarySensor * const sensor) {
    this->active_sensor_ = sensor;
  }
  void set_online_sensor(binary_sensor::BinarySensor * const sensor) {
    this->online_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutUnitStateBits);
  }
  void set_valve_sensor(binary_sensor::BinarySensor * const sensor) {
    this->valve_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutSystemStateBits);
  }
  void set_short_cycle_sensor(binary_sensor::BinarySensor * const sensor) {
    this->short_cycle_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutSystemStateBits);
  }
  void set_system_defrost_sensor(binary_sensor::BinarySensor * const sensor) {
    this->system_defrost_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutSystemStateBits);
  }
  void set_multizone_conflict_sensor(binary_sensor::BinarySensor * const sensor) {
    this->multizone_conflict_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutSystemStateBits);
  }
  void set_serial_error_sensor(binary_sensor::BinarySensor * const sensor) {
    this->serial_error_sensor_ = sensor;
    this->get_parent()->request_readout(DaikinS21::ReadoutErrorStatus);
  }

 protected:
  DaikinS21BinarySensorMode *mode_sensors_[DaikinModeCount]{};
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
