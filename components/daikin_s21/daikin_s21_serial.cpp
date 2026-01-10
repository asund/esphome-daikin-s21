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
 * If the loop is active the state machine is actively communicating.
 */
void DaikinSerial::loop() {
  // Handle delays
  const auto now = millis();
  const bool delay_expired = timestamp_passed(now, this->next_event_ms);
  switch (this->comm_state) {
    case CommState::DelayAck: // Waiting for turnaround time to send Ack
      if (delay_expired) {
        this->uart.write_byte(ACK);
        this->comm_state = CommState::DelayIdle;
        this->next_event_ms = now + DaikinSerial::next_tx_delay_period_ms;
      }
      return;

    case CommState::DelayIdle:  // Waiting for cooldown before idle
      if (delay_expired) {
        this->disable_loop();
        this->get_parent()->handle_serial_idle();
      }
      return;

    default:  // Rx Timeouts
      if (delay_expired) {
        this->comm_state = CommState::DelayIdle;
        this->next_event_ms = now + DaikinSerial::rx_timout_period_ms; // 2x rx_timout_period_ms in total before retry
        this->get_parent()->handle_serial_result(Result::Timeout);
        return;
      }
      break;  // still have time to receive bytes below
  }

  // Otherwise, we're trying the receive data from the unit
  uint8_t rx_bytes = 0;
  while (this->uart.available()) {
    uint8_t byte;
    this->uart.read_byte(&byte);
    rx_bytes++;
    this->next_event_ms = now + DaikinSerial::rx_timout_period_ms;  // received a byte, reset the character timeout

    // handle the byte
    switch (this->comm_state) {
      case CommState::QueryAck:
      case CommState::CommandAck:
        switch (byte) {
          case ACK:
            if (this->comm_state == CommState::QueryAck) {
              this->comm_state = CommState::QueryStx; // query results text to follow
            } else {
              this->comm_state = CommState::DelayIdle;
              this->next_event_ms = now + DaikinSerial::next_tx_delay_period_ms;
              this->get_parent()->handle_serial_result(Result::Ack);
              return;
            }
            break;

          case NAK:
            this->comm_state = CommState::DelayIdle;
            this->next_event_ms = now + DaikinSerial::next_tx_delay_period_ms;
            this->get_parent()->handle_serial_result(Result::Nak);
            return;

          default:
            ESP_LOGW(TAG, "Rx ACK: Unexpected 0x%02" PRIX8, byte);
            this->comm_state = CommState::DelayIdle;
            this->next_event_ms = now + DaikinSerial::error_delay_period_ms;
            this->get_parent()->handle_serial_result(Result::Error);
            return;
        }
        break;

      case CommState::QueryStx:
        switch (byte) {
          case STX:
            this->comm_state = CommState::QueryEtx; // query results payload to follow
            break;

          case ACK:
            ESP_LOGV(TAG, "Rx STX: Repeated ACK, ignoring"); // on rare occasions my unit will do this, not harmful
            break;

          default:
            ESP_LOGW(TAG, "Rx STX: Unexpected 0x%02" PRIX8, byte);
            this->comm_state = CommState::DelayIdle;
            this->next_event_ms = now + DaikinSerial::error_delay_period_ms;
            this->get_parent()->handle_serial_result(Result::Error);
            return;
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
                // Reduce the delay period by the number of character times waiting for the scheduler
                // to call loop() since the final ETX was received. This is very minor.
                auto delay_period_ms = DaikinSerial::ack_delay_period_ms;
                constexpr uint32_t char_time = 1000 / (2400 / (1+8+2+1));
                const int max_bytes_per_loop = App.get_loop_interval() / char_time;
                delay_period_ms -= std::max(max_bytes_per_loop - rx_bytes, 0) * char_time;

                this->comm_state = CommState::DelayAck;
                this->next_event_ms = now + delay_period_ms;
                this->get_parent()->handle_serial_result(Result::Ack, this->response);
                return;
              } else {
                ESP_LOGW(TAG, "Rx ETX: Checksum mismatch: 0x%02" PRIX8 " != 0x%02" PRIX8 " (calc from %s)",
                    checksum, calc_checksum, hex_repr(this->response).c_str());
                this->comm_state = CommState::DelayIdle;
                this->next_event_ms = now + DaikinSerial::error_delay_period_ms;
                this->get_parent()->handle_serial_result(Result::Error);
                return;
              }
            }
            break;

          default:  // not the end, add to buffer
            this->response.push_back(byte);
            if (this->response.size() > MAX_RESPONSE_SIZE) {
              ESP_LOGW(TAG, "Rx ETX: Overflow %s %s + 0x%02" PRIX8,
                  str_repr(this->response).c_str(), hex_repr(this->response).c_str(), byte);
              this->comm_state = CommState::DelayIdle;
              this->next_event_ms = now + DaikinSerial::error_delay_period_ms;
              this->get_parent()->handle_serial_result(Result::Error);
              return;
            }
            break;
        }
        break;

      default:
        return;
    }
  }
}

void DaikinSerial::dump_config() {
  ESP_LOGCONFIG(TAG, "  Debug: %s", ONOFF(this->debug));
}

void DaikinSerial::send_frame(const std::string_view cmd, const std::span<const uint8_t> payload /*= {}*/) {
  if (cmd.size() > MAX_COMMAND_SIZE) {
    ESP_LOGE(TAG, "Tx: Command '%" PRI_SV "' too large", PRI_SV_ARGS(cmd));
    // Prevent spam by starting the state machine anyways to delay
    // this is called from DaikinS21 context when idle, trigger it again after a cooldown
    this->comm_state = CommState::DelayIdle;
    this->next_event_ms = millis() + DaikinSerial::error_delay_period_ms;
    this->enable_loop_soon_any_context();
    this->get_parent()->handle_serial_result(Result::Error);
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
  this->next_event_ms = millis() + DaikinSerial::rx_timout_period_ms;
  this->enable_loop_soon_any_context();
}

} // namespace esphome::daikin_s21
