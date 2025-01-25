#pragma once

#include <bitset>
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

namespace esphome {
namespace daikin_s21 {

enum class DaikinClimateMode : uint8_t {
  Disabled = '1',
  Auto = '0',
  Dry = '2',
  Cool = '3',
  Heat = '4',
  Fan = '6',
};

enum class DaikinFanMode : uint8_t {
  Auto = 'A',
  Silent = 'B',
  Speed1 = '3',
  Speed2 = '4',
  Speed3 = '5',
  Speed4 = '6',
  Speed5 = '7',
};

std::string daikin_climate_mode_to_string(DaikinClimateMode mode);
std::string daikin_fan_mode_to_string(DaikinFanMode mode);

inline float c10_c(int16_t c10) { return c10 / 10.0; }
inline float c10_f(int16_t c10) { return c10_c(c10) * 1.8 + 32.0; }

struct DaikinSettings {
  bool power_on = false;
  DaikinClimateMode mode = DaikinClimateMode::Disabled;
  DaikinFanMode fan = DaikinFanMode::Auto;
  int16_t setpoint = 23;

  bool swing_v = false;
  bool swing_h = false;
  bool powerful = false;
  bool econo = false;
};

class DaikinS21 : public PollingComponent {
 public:
  enum class CommState : uint8_t {
    Idle,
    QueryAck,
    QueryStx,
    QueryEtx,
    CommandAck,
    Error
  };

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  void set_uarts(uart::UARTComponent *tx, uart::UARTComponent *rx);
  void set_debug_protocol(bool set) { this->debug_protocol = set; }

  bool is_ready() { return this->ready.all(); }

  bool is_power_on() { return this->active.power_on; }
  DaikinClimateMode get_climate_mode() { return this->active.mode; }
  DaikinFanMode get_fan_mode() { return this->active.fan; }
  float get_setpoint() { return this->active.setpoint / 10.0; }
  bool get_swing_h() { return this->active.swing_h; }
  bool get_swing_v() { return this->active.swing_v; }
  bool get_powerful() { return this->active.powerful; }
  bool get_econo() { return this->active.econo; }

  // external command actions
  void set_daikin_climate_settings(bool power_on, DaikinClimateMode mode,
                                   float setpoint, DaikinFanMode fan_mode);
  void set_swing_settings(bool swing_v, bool swing_h);
  void set_powerful_settings(bool value);
  void set_econo_settings(bool value);

  float get_temp_inside() { return this->temp_inside / 10.0; }
  float get_temp_outside() { return this->temp_outside / 10.0; }
  float get_temp_coil() { return this->temp_coil / 10.0; }
  uint16_t get_fan_rpm() { return this->fan_rpm; }
  uint8_t get_swing_vertical_angle() { return this->swing_vertical_angle; }
  uint16_t get_compressor_frequency() { return this->demand; }
  bool is_idle() { return this->demand == 0; }
  void set_has_presets(bool value) { this->has_presets = value; }

 protected:
  static constexpr uint32_t S21_RESPONSE_TURNAROUND = 75;
  static constexpr uint32_t S21_RESPONSE_TIMEOUT = 250;
  static constexpr uint32_t S21_ERROR_TIMEOUT = 3000;
  static constexpr uint32_t S21_MAX_COMMAND_SIZE = 4;
  static constexpr uint32_t S21_MAX_PAYLOAD_SIZE = 4;

  void dump_state();
  void check_uart_settings();
  void write_frame(const uint8_t *payload = nullptr, size_t payload_len = 0);
  void tx_next_command();
  void parse_command_response();
  void handle_rx_byte(uint8_t new_byte);

  uart::UARTComponent *tx_uart{nullptr};
  uart::UARTComponent *rx_uart{nullptr};
  std::bitset<3> ready = {};
  bool debug_protocol = false;
  bool refresh_state = false;
  
  CommState comm_state = CommState::Idle;
  std::vector<const char *> queries = {};
  std::vector<const char *>::iterator current_query;
  const char *tx_command = "";
  std::vector<uint8_t> rx_buffer = {};
  uint32_t rx_timeout = 0;

  DaikinSettings active = {};
  DaikinSettings pending = {};
  bool activate_climate = false;
  bool activate_swing = false;
  bool activate_powerful = false;
  bool activate_econo = false;

  int16_t temp_inside = 0;
  int16_t temp_outside = 0;
  int16_t temp_coil = 0;
  uint16_t fan_rpm = 0;
  int16_t swing_vertical_angle = 0;
  uint16_t demand = 0;

  //proto support
  bool has_presets = true;
  bool support_rg = false;
};

class DaikinS21Client {
 public:
  void set_s21(DaikinS21 *s21) { this->s21 = s21; }

 protected:
  DaikinS21 *s21;
};

}  // namespace daikin_s21
}  // namespace esphome
