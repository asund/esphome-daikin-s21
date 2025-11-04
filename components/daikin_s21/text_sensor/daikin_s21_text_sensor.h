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

  void set_debug_query_sensors(std::vector<text_sensor::TextSensor *> &&sensors);

 protected:
  std::vector<text_sensor::TextSensor *> sensors{};
};

} // namespace esphome::daikin_s21
