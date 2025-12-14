#include "daikin_s21_switch.h"

namespace esphome::daikin_s21 {

static const char *const TAG = "daikin_s21.switch";

void DaikinS21Switch::setup() {
  this->get_parent()->update_callbacks.add([this](){ this->enable_loop_soon_any_context(); }); // enable update events from DaikinS21
  this->disable_loop(); // wait for updates
}

void DaikinS21Switch::loop() {
  const auto now = millis();
  for (auto mode_switch : this->mode_switches_) {
    if ((mode_switch != nullptr) && (static_cast<std::make_signed_t<decltype(now)>>(now - mode_switch->last_publish_ms) >= 0)) {
      mode_switch->last_publish_ms = now;
      mode_switch->publish_state(this->get_parent()->get_mode(mode_switch->mode));
    }
  }

  this->disable_loop(); // wait for further updates
}

void DaikinS21Switch::dump_config() {
  ESP_LOGCONFIG(TAG, "Daikin S21 Switch:");
  for (auto mode_switch : this->mode_switches_) {
    if (mode_switch != nullptr) {
      LOG_SWITCH("  ", mode_switch->get_name().c_str(), mode_switch);
    }
  }
}

void DaikinS21SwitchMode::write_state(bool state) {
  this->last_publish_ms = millis() + (this->get_parent()->get_cycle_interval_ms() * 2) + 1000;
  this->get_parent()->set_mode(this->mode, state);
  this->publish_state(state);
}

} // namespace esphome::daikin_s21
