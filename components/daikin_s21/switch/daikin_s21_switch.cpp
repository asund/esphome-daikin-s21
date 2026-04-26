#include "daikin_s21_switch.h"

namespace esphome::daikin_s21 {

static const char * const TAG = "daikin_s21.switch";

void DaikinS21Switch::setup() {
  // register for update events from DaikinS21
  this->get_parent()->update_callbacks.add([this](){ this->enable_loop_soon_any_context(); });
  this->disable_loop(); // wait for updates
}

/**
 * ESPHome Component loop
 *
 * Deferred work when an update occurs. Use Component::defer if more work items are added.
 *
 * Publishes any state changes to Home Assistant.
 */
void DaikinS21Switch::loop() {
  this->disable_loop(); // use loop as a oneshot timer

  for (auto mode_switch : this->mode_switches_) {
    if (mode_switch != nullptr) {
      mode_switch->publish_state(this->get_parent()->get_mode(mode_switch->mode));
    }
  }
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
