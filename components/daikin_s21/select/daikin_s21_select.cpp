#include "daikin_s21_select.h"

namespace esphome::daikin_s21 {

static const char *const TAG = "daikin_s21.select";

void DaikinS21Select::setup() {
  this->get_parent()->update_callbacks.add([this](){ this->enable_loop_soon_any_context(); }); // enable update events from DaikinS21
  this->disable_loop(); // wait for updates
}

void DaikinS21Select::loop() {
  if (this->swing_select_ != nullptr) {
    const auto current = static_cast<size_t>(this->get_parent()->get_vertical_swing_mode());
    const auto index = this->swing_select_->active_index();
    if ((index.has_value() == false) || (index.value() != current)) {
      this->swing_select_->publish_state(current);
    }
  }

  this->disable_loop(); // wait for further updates
}

void DaikinS21Select::dump_config() {
  ESP_LOGCONFIG(TAG, "Daikin S21 Select:");
  if (swing_select_ != nullptr) {
    LOG_SELECT("  ", "Vertical Swing", this->swing_select_);
  }
}

void DaikinS21SelectSwing::control(const size_t index) {
  this->get_parent()->set_vertical_swing_mode(static_cast<DaikinVerticalSwingMode>(index));
  this->publish_state(index);
}

} // namespace esphome::daikin_s21
