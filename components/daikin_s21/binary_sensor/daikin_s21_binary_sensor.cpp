#include "daikin_s21_binary_sensor.h"

namespace esphome::daikin_s21 {

static const char *const TAG = "daikin_s21.binary_sensor";

void DaikinS21BinarySensor::setup() {
  this->get_parent()->binary_sensor_callback = std::bind(&DaikinS21BinarySensor::update_handler, this); // enable update events from DaikinS21
  this->disable_loop(); // wait for updates
}

void DaikinS21BinarySensor::loop() {
  const DaikinUnitState unit = this->get_parent()->get_unit_state();
  const DaikinSystemState system = this->get_parent()->get_system_state();

  if (this->powerful_sensor_ != nullptr) {
    this->powerful_sensor_->publish_state(unit.powerful());
  }
  if (this->defrost_sensor_ != nullptr) {
    this->defrost_sensor_->publish_state(unit.defrost());
  }
  if (this->active_sensor_ != nullptr) {
    this->active_sensor_->publish_state(unit.active());  // unit
  }
  if (this->online_sensor_ != nullptr) {
    this->online_sensor_->publish_state(unit.online());
  }
  if (this->valve_sensor_ != nullptr) {
    this->valve_sensor_->publish_state(system.active());  // refrigerant valve
  }
  if (this->short_cycle_sensor_ != nullptr) {
    this->short_cycle_sensor_->publish_state(!system.locked());  // invert for Home Assistant locked/unlocked logic
  }
  if (this->system_defrost_sensor_ != nullptr) {
    this->system_defrost_sensor_->publish_state(system.defrost());
  }
  if (this->multizone_conflict_sensor_ != nullptr) {
    this->multizone_conflict_sensor_->publish_state(!system.multizone_conflict()); // invert for Home Assistant locked/unlocked logic
  }

  this->disable_loop(); // wait for further updates
}

void DaikinS21BinarySensor::update_handler() {
  this->enable_loop_soon_any_context();  // defer publish to next loop()
}

void DaikinS21BinarySensor::dump_config() {
  ESP_LOGCONFIG(TAG, "Daikin S21 Binary Sensor:");
  LOG_BINARY_SENSOR("  ", "Powerful", this->powerful_sensor_);
  LOG_BINARY_SENSOR("  ", "Defrost", this->defrost_sensor_);
  LOG_BINARY_SENSOR("  ", "Active", this->active_sensor_);
  LOG_BINARY_SENSOR("  ", "Online", this->online_sensor_);
  LOG_BINARY_SENSOR("  ", "Valve", this->valve_sensor_);
  LOG_BINARY_SENSOR("  ", "Short Cycle", this->short_cycle_sensor_);
  LOG_BINARY_SENSOR("  ", "System Defrost", this->system_defrost_sensor_);
  LOG_BINARY_SENSOR("  ", "Multizone Conflict", this->multizone_conflict_sensor_);
}

} // namespace esphome::daikin_s21
