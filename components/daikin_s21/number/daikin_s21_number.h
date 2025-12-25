#pragma once

#include "esphome/components/number/number.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "../daikin_s21_types.h"

namespace esphome::daikin_s21 {

class DaikinS21NumberDemand : public number::Number,
                              public Parented<DaikinS21> {
 protected:
  void control(float value) override;
};

class DaikinS21Number : public Component,
                        public Parented<DaikinS21> {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_demand(DaikinS21NumberDemand *number) {
    this->demand_number_ = number;
  }

 protected:
  DaikinS21NumberDemand *demand_number_{};
};

} // namespace esphome::daikin_s21
