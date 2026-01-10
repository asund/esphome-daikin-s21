#include <ranges>
#include "esphome/components/text_sensor/text_sensor.h"
#include "../utils.h"
#include "daikin_s21_text_sensor.h"

namespace esphome::daikin_s21 {

static const char *const TAG = "daikin_s21.text_sensor";

void DaikinS21TextSensor::setup() {
  for (const auto &sensor : this->sensors) {
    this->get_parent()->add_debug_query(sensor->get_name().c_str());  // register queries
  }
  this->get_parent()->update_callbacks.add([this](){ this->enable_loop_soon_any_context(); }); // enable update events from DaikinS21
  this->disable_loop(); // wait for updates
}

/**
 * ESPHome Component loop
 *
 * Deferred work when an update occurs.
 *
 * Publish the sensors and wait for further updates.
 */
void DaikinS21TextSensor::loop() {
  if (this->statics_done == false) {
    if (this->software_version_sensor_ != nullptr) {
      this->software_version_sensor_->publish_state(this->get_parent()->get_software_version());
    }
    if (this->software_revision_sensor_ != nullptr) {
      this->software_revision_sensor_->publish_state(this->get_parent()->get_software_revision());
    }
    if (this->model_sensor_ != nullptr) {
      this->model_sensor_->publish_state(this->get_parent()->get_model_name());
    }
    this->statics_done = true;
  }
  // update all debug sensors
  for (auto * const sensor : this->sensors) {
    std::string current_state = str_repr(this->get_parent()->get_query_result(sensor->get_name().c_str()));
    if (sensor->state != current_state) {
      sensor->publish_state(current_state);
    }
  }
  this->disable_loop(); // wait for further updates
}

void DaikinS21TextSensor::set_debug_query_sensors(std::vector<text_sensor::TextSensor *> &&sensors) {
  this->sensors = sensors;
}

void DaikinS21TextSensor::dump_config() {
  LOG_TEXT_SENSOR("", "Software Version", this->software_version_sensor_);
  LOG_TEXT_SENSOR("", "Model", this->model_sensor_);
  for (const auto sensor : this->sensors) {
    LOG_TEXT_SENSOR("", "Debug Query", sensor);
  }
}

} // namespace esphome::daikin_s21
