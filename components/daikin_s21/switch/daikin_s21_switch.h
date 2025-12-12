#pragma once

#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "../daikin_s21_types.h"
#include "../s21.h"

namespace esphome::daikin_s21 {

class DaikinS21SwitchMode : public switch_::Switch,
                            public Parented<DaikinS21> {
 protected:
  void write_state(bool state) override;
 public:
  DaikinMode mode;
};

class DaikinS21Switch : public Component,
                        public Parented<DaikinS21> {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_powerful_switch(DaikinS21SwitchMode *mode_switch) {
    mode_switch->mode = ModePowerful;
    this->mode_switches_[mode_switch->mode] = mode_switch;
    this->get_parent()->request_readout(DaikinS21::ReadoutSpecialModes);
  }

  void set_comfort_switch(DaikinS21SwitchMode *mode_switch) {
    mode_switch->mode = ModeComfort;
    this->mode_switches_[mode_switch->mode] = mode_switch;
    this->get_parent()->request_readout(DaikinS21::ReadoutSpecialModes);
  }

  void set_quiet_switch(DaikinS21SwitchMode *mode_switch) {
    mode_switch->mode = ModeQuiet;
    this->mode_switches_[mode_switch->mode] = mode_switch;
    this->get_parent()->request_readout(DaikinS21::ReadoutSpecialModes);
  }

  void set_streamer_switch(DaikinS21SwitchMode *mode_switch) {
    mode_switch->mode = ModeStreamer;
    this->mode_switches_[mode_switch->mode] = mode_switch;
    this->get_parent()->request_readout(DaikinS21::ReadoutSpecialModes);
  }

  void set_sensor_switch(DaikinS21SwitchMode *mode_switch) {
    mode_switch->mode = ModeSensor;
    this->mode_switches_[mode_switch->mode] = mode_switch;
    this->get_parent()->request_readout(DaikinS21::ReadoutSpecialModes);
  }

  void set_econo_switch(DaikinS21SwitchMode *mode_switch) {
    mode_switch->mode = ModeEcono;
    this->mode_switches_[mode_switch->mode] = mode_switch;
    this->get_parent()->request_readout(DaikinS21::ReadoutDemandAndEcono);
  }

 protected:
  DaikinS21SwitchMode * mode_switches_[DaikinModeCount]{};
};

} // namespace esphome::daikin_s21
