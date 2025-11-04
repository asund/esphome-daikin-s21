#include "daikin_s21_sensor.h"
#include "../s21.h"

namespace esphome::daikin_s21 {

static const char *const TAG = "daikin_s21.sensor";

void DaikinS21Sensor::setup() {
  if (this->is_free_run()) {
    this->get_parent()->update_callbacks.add([this](){ this->enable_loop_soon_any_context(); });  // enable update events from DaikinS21
  }
  this->disable_loop(); // wait for updates
}

/**
 * ESPHome Component loop
 *
 * Deferred work when an update occurs.
 *
 * Publish the sensors and wait for further updates.
 */
void DaikinS21Sensor::loop() {
  this->publish_sensors();
  this->disable_loop();
}

/**
 * ESPHome PollingComponent loop
 *
 * Publishes sensors on user interval when S21 is ready.
 */
void DaikinS21Sensor::update() {
  if (this->get_parent()->is_ready()) {
    this->publish_sensors();
  }
}

/**
 * Unconditionally publish the sensors
 */
void DaikinS21Sensor::publish_sensors() {
  if (this->temp_inside_sensor_ != nullptr) {
    this->temp_inside_sensor_->publish_state(this->get_parent()->get_temp_inside().f_degc());
  }
  if (this->temp_target_sensor_ != nullptr) {
    this->temp_target_sensor_->publish_state(this->get_parent()->get_temp_target().f_degc());
  }
  if (this->temp_outside_sensor_ != nullptr) {
    this->temp_outside_sensor_->publish_state(this->get_parent()->get_temp_outside().f_degc());
  }
  if (this->temp_coil_sensor_ != nullptr) {
    this->temp_coil_sensor_->publish_state(this->get_parent()->get_temp_coil().f_degc());
  }
  if (this->fan_speed_sensor_ != nullptr) {
    this->fan_speed_sensor_->publish_state(this->get_parent()->get_fan_rpm());
  }
  if (this->swing_vertical_angle_sensor_ != nullptr) {
    this->swing_vertical_angle_sensor_->publish_state(this->get_parent()->get_swing_vertical_angle());
  }
  if (this->compressor_frequency_sensor_ != nullptr) {
    this->compressor_frequency_sensor_->publish_state(this->get_parent()->get_compressor_frequency());
  }
  if (this->humidity_sensor_ != nullptr) {
    this->humidity_sensor_->publish_state(this->get_parent()->get_humidity());
  }
  if (this->demand_sensor_ != nullptr) {
    this->demand_sensor_->publish_state(this->get_parent()->get_demand());
  }
  if (this->ir_counter_sensor_ != nullptr) {
    this->ir_counter_sensor_->publish_state(this->get_parent()->get_ir_counter());
  }
  if (this->power_consumption_sensor_ != nullptr) {
    this->power_consumption_sensor_->publish_state(this->get_parent()->get_power_consumption() / 100.0F);
  }
}

void DaikinS21Sensor::dump_config() {
  ESP_LOGCONFIG(TAG, "Daikin S21 Sensor:");
  LOG_SENSOR("  ", "Temperature Inside", this->temp_inside_sensor_);
  LOG_SENSOR("  ", "Temperature Target", this->temp_target_sensor_);
  LOG_SENSOR("  ", "Temperature Outside", this->temp_outside_sensor_);
  LOG_SENSOR("  ", "Temperature Coil", this->temp_coil_sensor_);
  LOG_SENSOR("  ", "Fan Speed", this->fan_speed_sensor_);
  LOG_SENSOR("  ", "Swing Vertical Angle", this->swing_vertical_angle_sensor_);
  LOG_SENSOR("  ", "Compressor Frequency", this->compressor_frequency_sensor_);
  LOG_SENSOR("  ", "Humidity", this->humidity_sensor_);
  LOG_SENSOR("  ", "Demand", this->demand_sensor_);
  LOG_SENSOR("  ", "IR Counter", this->ir_counter_sensor_);
  LOG_SENSOR("  ", "Power Consumption", this->power_consumption_sensor_);
}

} // namespace esphome::daikin_s21
