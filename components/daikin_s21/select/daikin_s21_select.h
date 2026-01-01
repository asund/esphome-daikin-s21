#pragma once

#include "esphome/components/select/select.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "../s21.h"

namespace esphome::daikin_s21 {

class DaikinS21SelectVerticalSwing : public select::Select,
                                     public Parented<DaikinS21> {
 protected:
  void control(size_t index) override;
};

class DaikinS21Select : public Component,
                        public Parented<DaikinS21> {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_vertical_swing_select(DaikinS21SelectVerticalSwing *vertical_swing_select) {
    this->vertical_swing_select_ = vertical_swing_select;
    this->get_parent()->request_readout(DaikinS21::ReadoutVerticalSwingMode);
  }

 protected:
  DaikinS21SelectVerticalSwing *vertical_swing_select_{};
};

} // namespace esphome::daikin_s21
