#pragma once

#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "../daikin_s21_types.h"
#include "../s21.h"

namespace esphome::daikin_s21 {

class DaikinS21SwitchMode final : public switch_::Switch,
                                  public Parented<DaikinS21> {
 protected:
  void write_state(bool state);
 public:
  DaikinS21SwitchMode(const DaikinMode mode) : mode(mode) {}
  DaikinMode mode{};
};

class DaikinS21Switch final : public Component,
                              public Parented<DaikinS21> {
 public:
  void setup();
  void loop();
  void dump_config();

  void set_mode_switch(DaikinS21SwitchMode * const mode_switch) {
    this->mode_switches_[mode_switch->mode] = mode_switch;
    if (mode_switch->mode == ModePowerful) {
      this->get_parent()->request_readout(ReadoutPowerful);
    } else if (mode_switch->mode == ModeEcono) {
      this->get_parent()->request_readout(ReadoutDemandAndEcono);
    } else {
      this->get_parent()->request_readout(ReadoutSpecialModes);
    }
  }

 protected:
  DaikinS21SwitchMode *mode_switches_[DaikinModeCount]{};
};

} // namespace esphome::daikin_s21
