#pragma once

#include "esphome/components/number/number.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "../daikin_s21_types.h"
#include "../s21.h"

namespace esphome::daikin_s21 {

class DaikinS21NumberDemand final : public number::Number,
                                    public Parented<DaikinS21> {
 protected:
  void control(float value);
};

class DaikinS21Number final : public Component,
                              public Parented<DaikinS21> {
 public:
  void setup();
  void loop();
  void dump_config();

  void set_demand(DaikinS21NumberDemand * const number) {
    this->demand_number_ = number;
    this->get_parent()->request_readout(ReadoutDemandAndEcono);
  }

 protected:
  DaikinS21NumberDemand *demand_number_{};
};

} // namespace esphome::daikin_s21
