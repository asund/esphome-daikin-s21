#include "daikin_s21_select.h"
#include "../daikin_s21_types.h"

namespace esphome::daikin_s21 {

static const char *const TAG = "daikin_s21.select";

void DaikinS21SelectHumidity::control(const size_t index) {
  if (index < DaikinHumidityModeCount) {
    this->get_parent()->set_humidity_mode(static_cast<DaikinHumidityMode>(index));
    this->publish_state(index);
  }
}

void DaikinS21SelectVerticalSwing::control(const size_t index) {
  this->get_parent()->set_vertical_swing_mode(static_cast<DaikinVerticalSwingMode>(index));
  this->publish_state(index);
}

void DaikinS21Select::setup() {
  this->get_parent()->update_callbacks.add([this](){ this->enable_loop_soon_any_context(); }); // enable update events from DaikinS21
  this->disable_loop(); // wait for updates
}

void DaikinS21Select::loop() {
  if (this->humidity_select_ != nullptr) {
    const auto current = this->get_parent()->get_humidity_mode();
    const auto index = this->humidity_select_->active_index();
    if ((index.has_value() == false) || (index.value() != current)) {
      this->humidity_select_->publish_state(current);
    }
  }
  if (this->vertical_swing_select_ != nullptr) {
    const auto current = this->get_parent()->get_vertical_swing_mode();
    const auto index = this->vertical_swing_select_->active_index();
    if ((index.has_value() == false) || (index.value() != current)) {
      this->vertical_swing_select_->publish_state(current);
    }
  }

  this->disable_loop(); // wait for further updates
}

void DaikinS21Select::dump_config() {
  ESP_LOGCONFIG(TAG, "Daikin S21 Select:");
  if (humidity_select_ != nullptr) {
    LOG_SELECT("  ", "Humidity", this->humidity_select_);
  }
  if (vertical_swing_select_ != nullptr) {
    LOG_SELECT("  ", "Vertical Swing", this->vertical_swing_select_);
  }
}

} // namespace esphome::daikin_s21
