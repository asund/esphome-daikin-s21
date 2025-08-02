#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "../s21.h"

namespace esphome {
namespace daikin_s21 {

class DaikinS21BinarySensor : public Component,
                              public Parented<DaikinS21> {
 public:
  void setup() override;
  void dump_config() override;
  void update_handler(uint8_t unit_state, uint8_t system_state);

  void set_powerful_sensor(binary_sensor::BinarySensor *sensor) {
    this->powerful_sensor_ = sensor;
  }
  void set_defrost_sensor(binary_sensor::BinarySensor *sensor) {
    this->defrost_sensor_ = sensor;
  }
  void set_active_sensor(binary_sensor::BinarySensor *sensor) {
    this->active_sensor_ = sensor;
  }
  void set_online_sensor(binary_sensor::BinarySensor *sensor) {
    this->online_sensor_ = sensor;
  }
  void set_valve_sensor(binary_sensor::BinarySensor *sensor) {
    this->valve_sensor_ = sensor;
  }
  void set_short_cycle_sensor(binary_sensor::BinarySensor *sensor) {
    this->short_cycle_sensor_ = sensor;
  }
  void set_system_defrost_sensor(binary_sensor::BinarySensor *sensor) {
    this->system_defrost_sensor_ = sensor;
  }
    void set_multizone_conflict_sensor(binary_sensor::BinarySensor *sensor) {
    this->multizone_conflict_sensor_ = sensor;
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
};

}  // namespace daikin_s21
}  // namespace esphome
