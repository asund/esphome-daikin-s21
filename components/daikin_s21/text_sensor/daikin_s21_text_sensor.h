#pragma once

#include <vector>
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "../s21.h"

using namespace esphome;

namespace esphome::daikin_s21 {

class DaikinS21TextSensor : public Component,
                            public Parented<DaikinS21> {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_software_version_sensor(text_sensor::TextSensor *sensor) {
    this->software_version_sensor_ = sensor;
  }

  void set_model_sensor(text_sensor::TextSensor *sensor) {
    this->model_sensor_ = sensor;
  }

  void set_debug_query_sensors(std::vector<text_sensor::TextSensor *> &&sensors);

 protected:
  bool statics_done{};
  text_sensor::TextSensor *software_version_sensor_{};
  text_sensor::TextSensor *model_sensor_{};
  std::vector<text_sensor::TextSensor *> sensors{};
};

} // namespace esphome::daikin_s21
