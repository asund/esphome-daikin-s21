#include <numeric>
#include <type_traits>
#include "esphome/core/application.h"
#include "daikin_s21_serial.h"
#include "s21.h"
#include "utils.h"

namespace esphome::daikin_s21 {

static const char *const TAG = "daikin_s21.serial";

static constexpr uint8_t STX{2};
static constexpr uint8_t ETX{3};
static constexpr uint8_t ENQ{5};
static constexpr uint8_t ACK{6};
static constexpr uint8_t NAK{21};

void DaikinSerial::setup() {
  // start idle, wait for updates
  this->disable_loop();
}

/**
 * Component polling loop
 *
 * If the loop is active, we're trying the receive data from the unit.
 */
void DaikinSerial::loop() {
  uint8_t rx_bytes = 0;
  bool rx_in_progress = true;

  while (rx_in_progress && this->uart.available()) {
    uint8_t byte;
    this->uart.read_byte(&byte);
    rx_bytes++;

    // handle the byte
    switch (this->comm_state) {
      case CommState::QueryAck:
      case CommState::CommandAck:
        switch (byte) {
          case ACK:
            if (this->comm_state == CommState::QueryAck) {
              this->comm_state = CommState::QueryStx; // query results text to follow
            } else {
              this->set_busy_timeout(DaikinSerial::next_tx_delay_period_ms);
              this->get_parent()->handle_serial_result(Result::Ack);
              rx_in_progress = false;
            }
            break;

          case NAK:
            this->set_busy_timeout(DaikinSerial::next_tx_delay_period_ms);
            this->get_parent()->handle_serial_result(Result::Nak);
            rx_in_progress = false;
            break;

          default:
            ESP_LOGW(TAG, "Rx ACK: Unexpected 0x%02" PRIX8, byte);
            this->set_busy_timeout(DaikinSerial::error_delay_period_ms);
            this->get_parent()->handle_serial_result(Result::Error);
            rx_in_progress = false;
            break;
        }
        break;

      case CommState::QueryStx:
        switch (byte) {
          case STX:
            this->comm_state = CommState::QueryEtx; // query results payload to follow
            break;

          case ACK:
            ESP_LOGD(TAG, "Rx STX: Repeated ACK, ignoring"); // on rare occasions my unit will do this
            break;

          default:
            ESP_LOGW(TAG, "Rx STX: Unexpected 0x%02" PRIX8, byte);
            this->set_busy_timeout(DaikinSerial::error_delay_period_ms);
            this->get_parent()->handle_serial_result(Result::Error);
            rx_in_progress = false;
        }
        break;

      case CommState::QueryEtx:
        switch (byte) {
          case ETX: // frame received, validate checksum
            {
              const uint8_t checksum = this->response.back();
              this->response.pop_back();
              const uint8_t calc_checksum = std::reduce(this->response.begin(), this->response.end(), 0U);
              if ((calc_checksum == checksum) || ((calc_checksum == STX) && (checksum == ENQ))) {  // protocol avoids STX in message body
                this->set_ack_timeout(rx_bytes);
                this->get_parent()->handle_serial_result(Result::Ack, this->response);
                rx_in_progress = false;
              } else {
                ESP_LOGW(TAG, "Rx ETX: Checksum mismatch: 0x%02" PRIX8 " != 0x%02" PRIX8 " (calc from %s)",
                  checksum, calc_checksum, hex_repr(this->response).c_str());
                this->set_busy_timeout(DaikinSerial::error_delay_period_ms);
                this->get_parent()->handle_serial_result(Result::Error);
                rx_in_progress = false;
              }
            }
            break;

          default:  // not the end, add to buffer
            this->response.push_back(byte);
            if (this->response.size() > MAX_RESPONSE_SIZE) {
              ESP_LOGW(TAG, "Rx ETX: Overflow %s %s + 0x%02" PRIX8,
                str_repr(this->response).c_str(), hex_repr(this->response).c_str(), byte);
              this->set_busy_timeout(DaikinSerial::error_delay_period_ms);
              this->get_parent()->handle_serial_result(Result::Error);
              rx_in_progress = false;
            }
            break;
        }
        break;

      default:
        rx_in_progress = false;
        break;
    }
  }

  // if we received some bytes but still need more, reset the character timeout
  if (rx_in_progress && (rx_bytes != 0)) {
    this->set_rx_timeout();
  }
}

void DaikinSerial::send_frame(const std::string_view cmd, const std::span<const uint8_t> payload /*= {}*/) {
  if (cmd.size() > MAX_COMMAND_SIZE) {
    ESP_LOGE(TAG, "Tx: Command '%" PRI_SV "' too large", PRI_SV_ARGS(cmd));
    this->get_parent()->handle_serial_result(Result::Error);
    // prevent spam by marking the component as busy
    // called from DaikinS21 context when idle, trigger it again after a cooldown without touching DaikinSerial's loop state
    this->set_timeout(DaikinSerial::timer_name, DaikinSerial::error_delay_period_ms, [this](){ this->get_parent()->handle_serial_idle(); });
    return;
  }

  if (this->debug) {
    if (payload.empty()) {
      ESP_LOGD(TAG, "Tx: %" PRI_SV, PRI_SV_ARGS(cmd));
    } else {
      ESP_LOGD(TAG, "Tx: %" PRI_SV " %s %s",
               PRI_SV_ARGS(cmd),
               str_repr(payload).c_str(),
               hex_repr(payload).c_str());
    }
  }

  // clear software and hardware receive buffers
  this->response.clear();
  while (this->uart.available()) {
    uint8_t byte;
    this->uart.read_byte(&byte);
  }

  // transmit
  this->uart.write_byte(STX);
  this->uart.write_array(reinterpret_cast<const uint8_t *>(cmd.data()), cmd.size());
  uint8_t checksum = std::reduce(cmd.begin(), cmd.end(), 0U);
  if (payload.empty() == false) {
    this->uart.write_array(payload.data(), payload.size());
    checksum = std::reduce(payload.begin(), payload.end(), checksum);
  }
  if (checksum == STX) {
    checksum = ENQ;  // mid-message STX characters are escaped
  }
  this->uart.write_byte(checksum);
  this->uart.write_byte(ETX);

  // wait for result
  this->comm_state = payload.empty() ? CommState::QueryAck : CommState::CommandAck;
  this->set_rx_timeout();
  this->enable_loop_soon_any_context();
}

void DaikinSerial::set_ack_timeout(const int bytes_received) {
  // Reduce the delay period by the number of character times waiting for the scheduler
  // to call loop() since the final ETX was received. This is very minor.
  auto delay_period_ms = DaikinSerial::ack_delay_period_ms;
  constexpr uint32_t char_time = 1000 / (2400 / (1+8+2+1));
  const int max_bytes_per_loop = App.get_loop_interval() / char_time;
  delay_period_ms -= std::max(max_bytes_per_loop - bytes_received, 0) * char_time;

  this->disable_loop();
  this->set_timeout(DaikinSerial::timer_name, delay_period_ms, [this](){ this->ack_timeout_handler(); });
}

void DaikinSerial::ack_timeout_handler() {
  this->uart.write_byte(ACK);
  this->set_timeout(DaikinSerial::timer_name, DaikinSerial::next_tx_delay_period_ms, [this](){ this->busy_timeout_handler(); });
}

void DaikinSerial::set_busy_timeout(const uint32_t delay_ms) {
  this->disable_loop();
  this->set_timeout(DaikinSerial::timer_name, delay_ms, [this](){ this->busy_timeout_handler(); });
}

void DaikinSerial::busy_timeout_handler() {
  this->get_parent()->handle_serial_idle();
}

void DaikinSerial::set_rx_timeout() {
  this->set_timeout(DaikinSerial::timer_name, DaikinSerial::rx_timout_period_ms, [this](){ this->rx_timeout_handler(); });
}

void DaikinSerial::rx_timeout_handler() {
  this->disable_loop();
  this->get_parent()->handle_serial_result(Result::Timeout);
  this->set_busy_timeout(DaikinSerial::rx_timout_period_ms);  // 2x rx_timout_period_ms in total before retry
}

} // namespace esphome::daikin_s21
