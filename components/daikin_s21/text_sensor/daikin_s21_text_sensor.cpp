#include <ranges>
#include "esphome/components/text_sensor/text_sensor.h"
#include "../utils.h"
#include "daikin_s21_text_sensor.h"

namespace esphome::daikin_s21 {

static const char *const TAG = "daikin_s21.text_sensor";

void DaikinS21TextSensor::setup() {
  this->get_parent()->update_callbacks.add([this](){ this->enable_loop_soon_any_context(); }); // enable update events from DaikinS21
  const auto query_strings = std::views::transform(this->sensors, [](const auto sensor){ return sensor->get_name().c_str(); });
  this->get_parent()->debug_queries = {query_strings.begin(), query_strings.end()}; // register queries
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
  // update all sensors
  for (auto * const sensor : this->sensors) {
    auto result = this->get_parent()->get_query_result(sensor->get_name().c_str());
    std::string current_state = str_repr(result.value);
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
  ESP_LOGCONFIG(TAG, "Daikin S21 Text Sensor:");
  for (const auto sensor : this->sensors) {
    LOG_TEXT_SENSOR("  ", sensor->get_name().c_str(), sensor);
  }
}

} // namespace esphome::daikin_s21
