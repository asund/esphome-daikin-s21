#include "daikin_s21_sensor.h"

namespace esphome {
namespace daikin_s21 {

static const char *const TAG = "daikin_s21.sensor";

void DaikinS21Sensor::update() {
  if (!this->s21->is_ready())
    return;
  if (this->temp_inside_sensor_ != nullptr) {
    this->temp_inside_sensor_->publish_state(this->s21->get_temp_inside());
  }
  if (this->temp_outside_sensor_ != nullptr) {
    this->temp_outside_sensor_->publish_state(this->s21->get_temp_outside());
  }
  if (this->temp_coil_sensor_ != nullptr) {
    this->temp_coil_sensor_->publish_state(this->s21->get_temp_coil());
  }
  if (this->fan_speed_sensor_ != nullptr) {
    this->fan_speed_sensor_->publish_state(this->s21->get_fan_rpm());
  }
  if (this->swing_vertical_angle_sensor_ != nullptr) {
    this->swing_vertical_angle_sensor_->publish_state(this->s21->get_swing_vertical_angle());
  }
  if (this->compressor_frequency_sensor_ != nullptr) {
    this->compressor_frequency_sensor_->publish_state(this->s21->get_compressor_frequency());
  }
}

void DaikinS21Sensor::dump_config() {
  ESP_LOGCONFIG(TAG, "Daikin S21 Sensor:");
  LOG_SENSOR("  ", "Temperature Inside", this->temp_inside_sensor_);
  LOG_SENSOR("  ", "Temperature Outside", this->temp_outside_sensor_);
  LOG_SENSOR("  ", "Temperature Coil", this->temp_coil_sensor_);
  LOG_SENSOR("  ", "Fan Speed", this->fan_speed_sensor_);
  LOG_SENSOR("  ", "Swing Vertical Angle", this->swing_vertical_angle_sensor_);
  LOG_SENSOR("  ", "Compressor Frequency", this->compressor_frequency_sensor_);
}

}  // namespace daikin_s21
}  // namespace esphome
