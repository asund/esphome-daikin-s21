#include "daikin_s21_select.h"
#include "../daikin_s21_types.h"

namespace esphome::daikin_s21 {

static const char *const TAG = "daikin_s21.select";

void DaikinS21SelectHumidity::control(const size_t index) {
  this->get_parent()->set_humidity_mode(static_cast<DaikinHumidityMode>(index));
  this->publish_state(index);
}

void DaikinS21SelectVerticalSwing::control(const size_t index) {
  this->get_parent()->set_vertical_swing_mode(static_cast<DaikinVerticalSwingMode>(index));
  this->publish_state(index);
}

void DaikinS21Select::setup() {
  if (((humidity_select_ != nullptr) && (humidity_select_->size() != DaikinHumidityModeCount)) ||
      ((vertical_swing_select_ != nullptr) && (vertical_swing_select_->size() != DaikinVerticalSwingModeCount))) {
    ESP_LOGE(TAG, "Select size and underlying enum mismatch!");
    this->mark_failed();
  }

  this->get_parent()->update_callbacks.add([this](){ this->enable_loop_soon_any_context(); }); // enable update events from DaikinS21
  this->disable_loop(); // wait for updates
}

void DaikinS21Select::loop() {
  const std::pair<select::Select *, size_t> selects_and_values[] = {
    {this->humidity_select_, this->get_parent()->get_humidity_mode()},
    {this->vertical_swing_select_, this->get_parent()->get_vertical_swing_mode()},
  };
  for (const auto& [sel, curr_idx] : selects_and_values) {
    if (sel != nullptr) {
      const auto last_idx = sel->active_index();
      if ((last_idx.has_value() == false) || (last_idx.value() != curr_idx)) {
        sel->publish_state(curr_idx);
      }
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
