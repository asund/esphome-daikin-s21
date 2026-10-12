#include "daikin_s21_select.h"
#include "../daikin_s21_types.h"

namespace esphome::daikin_s21 {

static const char * const TAG = "daikin_s21.select";

void DaikinS21SelectLEDBrightness::control(const size_t index) {
  this->get_parent()->set_brightness_mode(static_cast<DaikinLEDBrightnessMode>(index));
  this->publish_state(index);
}

void DaikinS21SelectHumidity::control(const size_t index) {
  this->get_parent()->set_humidity_mode(static_cast<DaikinHumidityMode>(index));
  this->publish_state(index);
}

void DaikinS21SelectVerticalSwing::control(const size_t index) {
  this->get_parent()->set_vertical_swing_mode(static_cast<DaikinVerticalSwingMode>(index));
  this->publish_state(index);
}

void DaikinS21Select::setup() {
  if (((brightness_select_ != nullptr) && (brightness_select_->size() != DaikinLEDBrightnessModeCount)) ||
      ((humidity_select_ != nullptr) && (humidity_select_->size() != DaikinHumidityModeCount)) ||
      ((vertical_swing_select_ != nullptr) && (vertical_swing_select_->size() != DaikinVerticalSwingModeCount))) {
    ESP_LOGE(TAG, "Select size and underlying enum mismatch!");
    this->mark_failed();
  }

  // register for update events from DaikinS21
  this->get_parent()->update_callbacks.add([this](){ this->enable_loop_soon_any_context(); });
  this->disable_loop(); // wait for updates
}

// only publish on changes or Select::publish state will spam the logs
void update_select(select::Select &sel, const size_t curr_idx) {
  const auto last_idx = sel.active_index();
  if ((last_idx.has_value() == false) || (last_idx.value() != curr_idx)) {
    sel.publish_state(curr_idx);
  }
}

/**
 * ESPHome Component loop
 *
 * Deferred work when an update occurs. Use Component::defer if more work items are added.
 *
 * Publishes any state changes to Home Assistant.
 */
void DaikinS21Select::loop() {
  this->disable_loop(); // use loop as a oneshot timer

  if (this->brightness_select_ != nullptr) {
    update_select(*(this->brightness_select_), this->get_parent()->get_brightness_mode());
  }
  if (this->humidity_select_ != nullptr) {
    update_select(*(this->humidity_select_), this->get_parent()->get_humidity_mode());
  }
  if (this->vertical_swing_select_ != nullptr) {
    update_select(*(this->vertical_swing_select_), this->get_parent()->get_vertical_swing_mode());
  }
}

void DaikinS21Select::dump_config() {
  LOG_SELECT("", "Brightness", this->brightness_select_);
  LOG_SELECT("", "Humidity", this->humidity_select_);
  LOG_SELECT("", "Vertical Swing", this->vertical_swing_select_);
}

} // namespace esphome::daikin_s21
