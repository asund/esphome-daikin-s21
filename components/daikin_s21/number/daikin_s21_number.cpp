#include "daikin_s21_number.h"
#include "../s21.h"

namespace esphome::daikin_s21 {

static const char *const TAG = "daikin_s21.number";

void DaikinS21NumberDemand::control(const float value) {
  this->get_parent()->set_demand_control(value);
  this->publish_state(value);
}

void DaikinS21Number::setup() {
  this->get_parent()->update_callbacks.add([this](){ this->enable_loop_soon_any_context(); }); // enable update events from DaikinS21
  this->disable_loop(); // wait for updates
}

void DaikinS21Number::loop() {
  if (this->demand_number_ != nullptr) {
    const float new_val = this->get_parent()->get_demand_control();
    if (this->demand_number_->state != new_val) {
      this->demand_number_->publish_state(new_val);
    }
  }

  this->disable_loop(); // wait for further updates
}

void DaikinS21Number::dump_config() {
  ESP_LOGCONFIG(TAG, "Daikin S21 Number:");
  LOG_NUMBER("  ", "Demand", this->demand_number_);
}

} // namespace esphome::daikin_s21
