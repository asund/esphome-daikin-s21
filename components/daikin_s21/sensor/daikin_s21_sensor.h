#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "../daikin_s21_types.h"

namespace esphome::daikin_s21 {

class DaikinS21Sensor : public PollingComponent,
                        public Parented<DaikinS21> {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;

  void update_handler();
  void publish_sensors();

  void set_temp_inside_sensor(sensor::Sensor *sensor) {
    this->temp_inside_sensor_ = sensor;
  }
  void set_temp_target_sensor(sensor::Sensor *sensor) {
    this->temp_target_sensor_ = sensor;
  }
  void set_temp_outside_sensor(sensor::Sensor *sensor) {
    this->temp_outside_sensor_ = sensor;
  }
  void set_temp_coil_sensor(sensor::Sensor *sensor) {
    this->temp_coil_sensor_ = sensor;
  }
  void set_fan_speed_sensor(sensor::Sensor *sensor) {
    this->fan_speed_sensor_ = sensor;
  }
  void set_swing_vertical_angle_sensor(sensor::Sensor *sensor) {
    this->swing_vertical_angle_sensor_ = sensor;
  }
  void set_compressor_frequency_sensor(sensor::Sensor *sensor) {
    this->compressor_frequency_sensor_ = sensor;
  }
  void set_humidity_sensor(sensor::Sensor *sensor) {
    this->humidity_sensor_ = sensor;
  }
  void set_demand_sensor(sensor::Sensor *sensor) {
    this->demand_sensor_ = sensor;
  }
  void set_ir_counter_sensor(sensor::Sensor *sensor) {
    this->ir_counter_sensor_ = sensor;
  }
  void set_power_consumption_sensor(sensor::Sensor *sensor) {
    this->power_consumption_sensor_ = sensor;
  }

 protected:
  bool is_free_run() const { return this->get_update_interval() == 0; }

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
  sensor::Sensor *power_consumption_sensor_{};
};

} // namespace esphome::daikin_s21
