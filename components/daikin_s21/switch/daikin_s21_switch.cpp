#include "daikin_s21_switch.h"

namespace esphome::daikin_s21 {

static const char * const TAG = "daikin_s21.switch";

void DaikinS21Switch::setup() {
  this->get_parent()->update_callbacks.add([this](){ this->enable_loop_soon_any_context(); }); // enable update events from DaikinS21
  this->disable_loop(); // wait for updates
}

void DaikinS21Switch::loop() {
  for (auto mode_switch : this->mode_switches_) {
    if (mode_switch != nullptr) {
      mode_switch->publish_state(this->get_parent()->get_mode(mode_switch->mode));
    }
  }

  this->disable_loop(); // wait for further updates
}

void DaikinS21Switch::dump_config() {
  for (auto mode_switch : this->mode_switches_) {
    LOG_SWITCH("", "Mode Switch", mode_switch);
  }
}

void DaikinS21SwitchMode::write_state(bool state) {
  this->get_parent()->set_mode(this->mode, state);
  this->publish_state(state);
}

} // namespace esphome::daikin_s21
