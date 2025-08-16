#include <algorithm>
#include <cinttypes>
#include <numeric>
#include "esphome/core/application.h"
#include "s21.h"

using namespace esphome;

namespace esphome::daikin_s21 {

#define STX 2
#define ETX 3
#define ENQ 5
#define ACK 6
#define NAK 21

static const char *const TAG = "daikin_s21";

uint8_t climate_mode_to_daikin(const climate::ClimateMode mode) {
  switch (mode) {
    case climate::CLIMATE_MODE_OFF:
    case climate::CLIMATE_MODE_HEAT_COOL:
    case climate::CLIMATE_MODE_AUTO:
    default:
      return '1';
      break;
    case climate::CLIMATE_MODE_HEAT:
      return '4';
    case climate::CLIMATE_MODE_COOL:
      return '3';
    case climate::CLIMATE_MODE_FAN_ONLY:
      return '6';
    case climate::CLIMATE_MODE_DRY:
      return '2';
  }
}

climate::ClimateMode daikin_to_climate_mode(const uint8_t mode) {
  switch (mode) {
    case '0': // heat_cool cooling
    case '1': // heat_cool setting
    case '7': // heat_cool heating
    default:
      return climate::CLIMATE_MODE_HEAT_COOL;
    case '2': // dry
      return climate::CLIMATE_MODE_DRY;
    case '3': // cool
      return climate::CLIMATE_MODE_COOL;
    case '4': // heat
      return climate::CLIMATE_MODE_HEAT;
    case '6': // fan_only
      return climate::CLIMATE_MODE_FAN_ONLY;
  }
}

climate::ClimateAction daikin_to_climate_action(const uint8_t action) {
  switch (action) {
    case '1': // heat_cool
    default:
      return climate::CLIMATE_ACTION_IDLE;
    case '2': // dry
      return climate::CLIMATE_ACTION_DRYING;
    case '0': // heat_cool cooling
    case '3': // cool
      return climate::CLIMATE_ACTION_COOLING;
    case '4': // heat
    case '7': // heat_cool heating
      return climate::CLIMATE_ACTION_HEATING;
    case '6': // fan_only
      return climate::CLIMATE_ACTION_FAN;
  }
}

climate::ClimateSwingMode daikin_to_climate_swing_mode(const uint8_t mode) {
  switch (mode) {
    case '1':
      return climate::CLIMATE_SWING_VERTICAL;
    case '2':
      return climate::CLIMATE_SWING_HORIZONTAL;
    case '7':
      return climate::CLIMATE_SWING_BOTH;
    default:
      return climate::CLIMATE_SWING_OFF;
  }
}

uint8_t climate_swing_mode_to_daikin(const climate::ClimateSwingMode mode) {
  switch (mode) {
    case climate::CLIMATE_SWING_OFF:
    default:
      return '0';
    case climate::CLIMATE_SWING_BOTH:
      return '7';
    case climate::CLIMATE_SWING_VERTICAL:
      return '1';
    case climate::CLIMATE_SWING_HORIZONTAL:
      return '2';
  }
}

int16_t bytes_to_num(std::span<const uint8_t> bytes) {
  // <ones><tens><hundreds><neg/pos>
  int16_t val = 0;
  val = bytes[0] - '0';
  val += (bytes[1] - '0') * 10;
  val += (bytes[2] - '0') * 100;
  if (bytes.size() > 3 && bytes[3] == '-')
    val *= -1;
  return val;
}

DaikinSerial::DaikinSerial(uart::UARTComponent &uart) : uart(uart) {
  this->uart.set_baud_rate(2400);
  this->uart.set_stop_bits(2);
  this->uart.set_data_bits(8);
  this->uart.set_parity(uart::UART_CONFIG_PARITY_EVEN);
  this->uart.load_settings();
}

void DaikinS21::dump_config() {
  ESP_LOGCONFIG(TAG, "DaikinS21:");
  ESP_LOGCONFIG(TAG, "  Update interval: %" PRIu32, this->get_update_interval());
}

std::string hex_repr(std::span<const uint8_t> bytes) {
  return format_hex_pretty(bytes.data(), bytes.size(), ':', false);
}

// Adapated from ESPHome UART debugger
std::string str_repr(std::span<const uint8_t> bytes) {
  std::string res;
  char buf[5];
  for (const auto b : bytes) {
    if (b == 7) {
      res += "\\a";
    } else if (b == 8) {
      res += "\\b";
    } else if (b == 9) {
      res += "\\t";
    } else if (b == 10) {
      res += "\\n";
    } else if (b == 11) {
      res += "\\v";
    } else if (b == 12) {
      res += "\\f";
    } else if (b == 13) {
      res += "\\r";
    } else if (b == 27) {
      res += "\\e";
    } else if (b == 34) {
      res += "\\\"";
    } else if (b == 39) {
      res += "\\'";
    } else if (b == 92) {
      res += "\\\\";
    } else if (b < 32 || b > 127) {
      sprintf(buf, "\\x%02" PRIX8, b);
      res += buf;
    } else {
      res += b;
    }
  }
  return res;
}

DaikinSerial::Result DaikinSerial::handle_rx(const uint8_t byte) {
  Result result = Result::Busy; // default to busy, override when rx phase is over

  switch (this->comm_state) {
    case CommState::QueryAck:
    case CommState::CommandAck:
      switch (byte) {
        case ACK:
          if (this->comm_state == CommState::QueryAck) {
            this->comm_state = CommState::QueryStx;
          } else {
            this->comm_state = CommState::NextTxDelay;
            result = Result::Ack;
          }
          break;
        case NAK:
          this->comm_state = CommState::NextTxDelay;
          result = Result::Nak;
          break;
        default:
          ESP_LOGW(TAG, "Rx ACK: Unexpected 0x%02" PRIX8, byte);
          this->comm_state = CommState::ErrorDelay;
          result = Result::Error;
          break;
      }
      break;

    case CommState::QueryStx:
      if (byte == STX) {
        this->comm_state = CommState::QueryEtx;
      } else if (byte == ACK) {
        ESP_LOGD(TAG, "Rx STX: Unexpected extra ACK, ignoring"); // on rare occasions my unit will do this
      } else {
        ESP_LOGW(TAG, "Rx STX: Unexpected 0x%02" PRIX8, byte);
        this->comm_state = CommState::ErrorDelay;
        result = Result::Error;
      }
      break;

    case CommState::QueryEtx:
      if (byte != ETX) {
        // not the end, add to buffer
        this->response.push_back(byte);
        if (this->response.size() > (S21_MAX_COMMAND_SIZE + S21_PAYLOAD_SIZE + 1)) {  // +1 for checksum byte
          ESP_LOGW(TAG, "Rx ETX: Overflow %s %s + 0x%02" PRIX8,
            str_repr(this->response).c_str(), hex_repr(this->response).c_str(), byte);
          this->comm_state = CommState::ErrorDelay;
          result = Result::Error;
        }
      } else {
        // frame received, validate checksum
        const uint8_t checksum = this->response.back();
        this->response.pop_back();
        const uint8_t calc_checksum = std::reduce(this->response.begin(), this->response.end(), 0U);
        if ((calc_checksum == checksum)
            || ((calc_checksum == STX) && (checksum == ENQ))) {  // protocol avoids STX in message body
          this->comm_state = CommState::AckResponseDelay;
          result = Result::Ack;
        } else {
          ESP_LOGW(TAG, "Rx ETX: Checksum mismatch: 0x%02" PRIX8 " != 0x%02" PRIX8 " (calc from %s)",
            checksum, calc_checksum, hex_repr(this->response).c_str());
          this->comm_state = CommState::ErrorDelay;
          result = Result::Error;
        }
      }
      break;

    default:
      break;
  }

  return result;
}

DaikinSerial::Result DaikinSerial::service() {
  Result result = Result::Busy;

  // nothing to do if idle
  if (this->comm_state == CommState::Idle) {
    return Result::Idle;
  }

  // all other states have a timeout
  const auto now = millis();
  uint32_t delay_period_ms;
  switch (this->comm_state) {
    case CommState::AckResponseDelay:
      delay_period_ms = 45; // official remote delay time before ACKing a response
      {
        // Reduce the delay period by the number of character times waiting for the scheduler
        // to call loop() since the final ETX was received. This is very minor and can go away
        // once TODO this is timer event driven.
        constexpr uint32_t char_time = 1000 / (2400 / (1+8+2+1));
        const int chars_per_loop = App.get_loop_interval() / char_time;
        delay_period_ms -= std::max(chars_per_loop - this->rx_bytes_last_call, 0) * char_time;
      }
      break;
    case CommState::NextTxDelay:
      delay_period_ms = 35; // official remote delay time between commands
      break;
    case CommState::ErrorDelay:
      delay_period_ms = 3000; // cooldown time when something goes wrong
      break;
    default:  // all other states are actively receiving data from the unit
      delay_period_ms = 250;  // character timeout when expecting a response from the unit
      break;
  }
  const bool timeout = (now - this->last_event_time_ms) > delay_period_ms;

  switch (this->comm_state) {
    case CommState::AckResponseDelay:
      if (timeout) {
        this->uart.write_byte(ACK);
        this->last_event_time_ms = now;
        this->comm_state = CommState::NextTxDelay;
      }
      break;

    case CommState::NextTxDelay:
    case CommState::ErrorDelay:
      if (timeout) {
        this->comm_state = CommState::Idle;
        result = Result::Idle;
      }
      break;

    default:
      // all other states are actively receiving data from the unit
      if (timeout) {
        this->comm_state = CommState::Idle;
        result = Result::Timeout;
        break;
      }
      this->rx_bytes_last_call = 0;
      while ((result == Result::Busy) && this->uart.available()) {  // read all available bytes
        last_event_time_ms = now;
        uint8_t byte;
        this->uart.read_byte(&byte);
        result = this->handle_rx(byte);
        this->rx_bytes_last_call++;
      }
      break;
  }

  return result;
}

DaikinSerial::Result DaikinSerial::send_frame(std::string_view cmd, const std::span<const uint8_t> payload /*= {}*/) {
  if (this->comm_state != CommState::Idle) {
    return Result::Busy;
  }

  if (cmd.size() > S21_MAX_COMMAND_SIZE) {
    ESP_LOGW(TAG, "Tx: Command '%" PRI_SV "' too large", PRI_SV_ARGS(cmd));
    return Result::Error;
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

  // prepare for response
  this->response.clear();
  this->flush_input();

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
  this->last_event_time_ms = millis();
  this->comm_state = payload.empty() ? CommState::QueryAck : CommState::CommandAck;

  return Result::Ack;
}

void DaikinSerial::flush_input() {  // would be a nice ESPHome API improvement
  while (this->uart.available()) {
    uint8_t byte;
    this->uart.read_byte(&byte);
  }
}

/**
 * Trigger a query cycle
 *
 * Set a flag and reenable the service loop to handle it.
 */
void DaikinS21::trigger_cycle() {
  cycle_triggered = true;
  this->enable_loop();
}

bool DaikinS21::prune_query(std::string_view query) {
  const auto it = std::ranges::find(this->queries, query);
  if (it != this->queries.end()) {
    this->queries.erase(it);
    return true;
  } else {
    return false;
  }
}

/**
 * Refine the pool of polling queries, adding or removing them as we learn about the unit.
 */
void DaikinS21::refine_queries() {
  if (this->ready.all() == false) {
    if (this->ready[ReadyProtocolVersion] == false) {
      if (this->determine_protocol_version()) {
        this->prune_query(StateQuery::OldProtocol);
        this->prune_query(StateQuery::ModelCode);
        this->prune_query(StateQuery::NewProtocol);
        this->prune_query(MiscQuery::Model);
        this->prune_query(MiscQuery::Version);
        ESP_LOGI(TAG, "Protocol version %" PRIu8 ".%" PRIu8 " detected", this->protocol_version.major, this->protocol_version.minor);
        this->ready.set(ReadyProtocolVersion);
      }
    }
    // Some units don't support more granular sensor queries
    if (this->ready[ReadySensorReadout] == false) {
      // TODO there's a chance that communication doesn't work at all so we didn't receive a NAK and these are still active. add an "acked" tracker to the queries if this is a concern.
      if (this->is_query_active(EnvironmentQuery::InsideTemperature) && this->is_query_active(EnvironmentQuery::OutsideTemperature)) {
        this->prune_query(StateQuery::InsideOutsideTemperatures);  // support for discrete granular sensors, no need for inferior consolidated query
      }
      this->ready.set(ReadySensorReadout);
    }
    if (this->ready[ReadyCapabilities] == false) {
      if (this->G2_model_info != 0) {
        this->prune_query(StateQuery::OptionalFeatures);
        if (this->support_swing) {
          this->queries.emplace_back(EnvironmentQuery::VerticalSwingAngle);
        }
        if (this->support_humidity) {
          this->queries.emplace_back(EnvironmentQuery::IndoorHumidity);
        }
        ESP_LOGI(TAG, "Capabilities detected: model info %c", this->G2_model_info);
        this->ready.set(ReadyCapabilities);
      }
    }
  }
}

/**
 * Perform the next action when the communication state is idle
 *
 * Usually selects the next command to issue to the unit
 *
 * Handle the results of the cycle if the last command was already issued:
 * - Process and report the results if the cycle was just completed
 * - Start the next cycle if free running or triggered in polling mode
 */
void DaikinS21::do_next_action() {
  std::array<uint8_t, DaikinSerial::S21_PAYLOAD_SIZE> payload;

  // Apply any pending settings
  // Important to clear the activate flag here as another command can be queued while waiting for this one to complete
  if (this->activate_climate) {
    this->activate_climate = false;
    this->tx_command = StateCommand::PowerModeTempFan;
    payload[0] = (this->pending.mode == climate::CLIMATE_MODE_OFF) ? '0' : '1'; // power
    payload[1] = climate_mode_to_daikin(this->pending.mode);
    payload[2] = (static_cast<int16_t>(this->pending.setpoint) / 5) + 28;
    payload[3] = static_cast<char>(this->pending.fan);
    this->serial.send_frame(this->tx_command, payload);
    return;
  }

  if (activate_swing_mode) {
    this->activate_swing_mode = false;
    this->tx_command = StateCommand::LouvreSwing;
    payload[0] = climate_swing_mode_to_daikin(this->pending.swing);
    payload[1] = (this->pending.swing != climate::CLIMATE_SWING_OFF) ? '?' : '0';
    payload[2] = '0';
    payload[3] = '0';
    this->serial.send_frame(this->tx_command, payload);
    return;
  }

  if (activate_preset) {
    // potentially a two stage operation -- first disabling the old, then enabling the new
    climate::ClimatePreset preset;
    bool enable;
    if ((this->active.preset != this->pending.preset) && (this->active.preset != climate::CLIMATE_PRESET_NONE)) {
      preset = this->active.preset;
      enable = false;
      if (this->pending.preset == climate::CLIMATE_PRESET_NONE) {
        this->activate_preset = false;  // don't execute again if we're just disabling the current preset
      }
    } else {
      preset = this->pending.preset;
      enable = true;
      this->activate_preset = false;  // don't execute again if we're setting the desired preset
    }
    switch (preset) {
      case climate::CLIMATE_PRESET_BOOST:
        this->tx_command = StateCommand::Powerful;
        payload[0] = enable ? '2' : '0';
        payload[1] = '0';
        payload[2] = '0';
        payload[3] = '0';
        this->serial.send_frame(this->tx_command, payload);
        return;
      case climate::CLIMATE_PRESET_ECO:
        this->tx_command = StateCommand::Econo;
        payload[0] = '0';
        payload[1] = enable ? '2' : '0';
        payload[2] = '0';
        payload[3] = '0';
        this->serial.send_frame(this->tx_command, payload);
        return;
      case climate::CLIMATE_PRESET_NONE:
      default:
        break;
    }
  }

  // Periodic query cycle
  if (this->current_query != this->queries.end()) {
    this->tx_command = *current_query;  // query cycle underway, continue
    this->serial.send_frame(this->tx_command);
    return;
  }

  // Query cycle complete
  const auto now = millis();

  // Are there fresh results to process?
  if (this->cycle_completed) {
    this->cycle_completed = false;
    this->cycle_time_ms = now - this->cycle_time_start_ms;
    if (this->ready.all()) {
      this->action_resolved = resolve_climate_action();
      if (this->modifiers[ModifierPowerful]) {
        this->active.preset = climate::CLIMATE_PRESET_BOOST;
      } else if (this->modifiers[ModifierEcono]) {
        this->active.preset = climate::CLIMATE_PRESET_ECO;
      } else {
        this->active.preset = climate::CLIMATE_PRESET_NONE;
      }
      // signal there's fresh data
      this->binary_sensor_callback_.call(this->unit_state, this->system_state);
      this->climate_callback_.call();
    }
    if ((now - last_state_dump_ms) > (60 * 1000)) {
      last_state_dump_ms = now;
      this->dump_state();
    }
  }

  // Handle polled execution state
  if (this->is_free_run() == false) {
    if (this->cycle_triggered == false) {
      this->disable_loop(); // single cycle now complete, disable this state machine and wait for the next poll to start it again
      return;
    }
    this->cycle_triggered = false;  // single cycle triggered, kick it off below
  }

  // Start fresh polling query cycle
  this->cycle_time_start_ms = now;
  this->refine_queries();
  this->current_query = queries.begin();
  this->tx_command = *(this->current_query);
  this->serial.send_frame(this->tx_command);
}

static DaikinC10 temp_bytes_to_c10(std::span<const uint8_t> bytes) { return bytes_to_num(bytes); }
static constexpr DaikinC10 temp_f9_byte_to_c10(uint8_t byte) { return (byte - 128) * 5; }
static constexpr uint8_t ahex_digit(uint8_t digit) { return (digit >= 'A') ? (digit - 'A') + 10 : digit - '0'; }
static constexpr uint8_t ahex_u8_le(uint8_t first, uint8_t second) { return (ahex_digit(second) << 4) | ahex_digit(first); }

void DaikinS21::parse_ack() {
  std::span<const uint8_t> rcode{};
  std::span<const uint8_t> payload{};

  ESP_LOGV(TAG, "Rx: ACK from S21 for command %" PRI_SV, PRI_SV_ARGS(this->tx_command));

  // prepare response buffers for decoding
  if (serial.response.empty()) {
    // commands don't return anything except an Ack, pretend we received the command itself to provide something to distinguish handling below
    rcode = { reinterpret_cast<const uint8_t *>(this->tx_command.data()), this->tx_command.size() };
  } else {
    rcode = { this->serial.response.begin(), this->tx_command.size() };
    payload = { this->serial.response.begin() + rcode.size(), this->serial.response.end() };
    // query response reveived, move to the next one or flag if it was the last
    // pushing along the state machine in here lets us easily interleve commands in a cycle
    this->current_query++;
    if (this->current_query == this->queries.end()) {
      this->cycle_completed = true;
    }
  }

  switch (rcode[0]) {
    case 'D':  // D -> D (fake response, see above)
      switch (rcode[1]) {
        case '1': break; // climate setting applied
        case '5': break; // swing settings applied
        case '6': break; // powerful setting applied
        case '7': break; // econo setting applied
        default: break;
      }
      return;

    case 'G':  // F -> G
      switch (rcode[1]) {
        case '1':  // F1 -> G1 Basic State
          if (payload[0] == '0') {
            this->active.mode = climate::CLIMATE_MODE_OFF;
            this->action_reported = climate::CLIMATE_ACTION_OFF;
          } else {
            this->active.mode = daikin_to_climate_mode(payload[1]);
            this->action_reported = daikin_to_climate_action(payload[1]);
          }
          this->active.setpoint = (payload[2] - 28) * 5;  // Celsius * 10
          // fan mode in payload[3], silent mode not reported so prefer RG
          this->ready.set(ReadyBasic);
          return;
        case '2':
          this->support_swing = payload[0] & 0b0100;
          this->support_horizontal_swing = payload[0] & 0b1000;
          this->G2_model_info = (payload[1] & 0b1000) ? 'N': 'C';
          this->support_humidity = payload[3] & 0b0010;
          return;
        case '5':  // F5 -> G5 -- Swing state
          this->active.swing = daikin_to_climate_swing_mode(payload[0]);
          return;
        case '6':
          this->modifiers[ModifierPowerful] = (payload[0] & 0b00000010);
          this->modifiers[ModifierComfort] =  (payload[0] & 0b01000000);
          this->modifiers[ModifierQuiet] =    (payload[0] & 0b10000000);
          this->modifiers[ModifierStreamer] = (payload[1] & 0b10000000);
          this->modifiers[ModifierSensor] =   (payload[3] & 0b00001000);
          this->modifiers[ModifierLED] =      (payload[3] & 0b00001100) == 0b00001100;
          return;
        case '7':
          this->modifiers[ModifierEcono] =    (payload[1] == '2');
          return;
        case '8':  // F8 -> G8 -- Original protocol version
          std::copy_n(payload.begin(), std::min(payload.size(), this->detect_responses.G8.size()), this->detect_responses.G8.begin());
          return;
        case '9':  // F9 -> G9 -- Temperature and humidity, better granularity in RH, Ra and Re
          this->temp_inside = temp_f9_byte_to_c10(payload[0]);  // 1 degree
          this->temp_outside = temp_f9_byte_to_c10(payload[1]); // 1 degree, danijelt reports 0xFF when unsupported
          if ((payload[2] - '0') <= 100) {  // Some units report 0xFF when unsupported
            this->humidity = payload[2] - '0';  // 5% granularity
          }
          return;
        case 'C':  // FC -> GC -- Model code (hex, not reversed here)
          std::copy_n(payload.begin(), std::min(payload.size(), this->detect_responses.GC.size()), this->detect_responses.GC.begin());
          return;
        case 'Y':
          if ((rcode[2] == '0') && (rcode[3] == '0')) { // FY00 -> GY00 Newer protocol version
            this->detect_responses.GY00 = bytes_to_num(payload);
          }
          return;
        default:
          break;
      }
      break;

    case 'S':  // R -> S
      switch (rcode[1]) {
        case 'B':  // Operational mode, single character, same info as G1
          return;
        case 'C':  // Setpoint, same info as G1
          this->active.setpoint = bytes_to_num(payload) * 10;  // whole degrees C
          return;
        case 'F':  // Swing mode, same info as G5. ascii hex string: 2F herizontal 1F vertical 7F both 00 off
          return;
        case 'G':  // Fan mode, better detail than G1 (reports quiet mode)
          this->active.fan = static_cast<daikin_s21::DaikinFanMode>(payload[0]);
          return;
        case 'H':  // Inside temperature
          this->temp_inside = temp_bytes_to_c10(payload);
          return;
        case 'I':  // Coil temperature
          this->temp_coil = temp_bytes_to_c10(payload);
          return;
        case 'L':  // Fan speed
          this->fan_rpm = bytes_to_num(payload) * 10;
          return;
        case 'M':  // v_swing setpoint
          break;
        case 'N':  // Vertical swing angle
          this->swing_vertical_angle = bytes_to_num(payload);
          return;
        case 'X':  // Internal control loop target temperature
          this->temp_target = temp_bytes_to_c10(payload);
          return;
        case 'a':  // Outside temperature
          this->temp_outside = temp_bytes_to_c10(payload);
          return;
        case 'b':  // Demand, 0-15
          this->demand = bytes_to_num(payload) / 10;
          return;
        case 'd':  // Compressor frequency in hertz, idle if 0.
          this->compressor_hz = bytes_to_num(payload);
          if (this->compressor_hz == 999) {
            this->compressor_hz = 0;  // reported by danijelt
          }
          return;
        case 'e':  // Humidity, %
          this->humidity = bytes_to_num(payload);
          return;
        case 'z':
          if ((rcode[2] == 'B') && (rcode[3] == '2')) { // FzB2 -> GzB2 Unit state
            this->unit_state = ahex_digit(payload[0]);
            this->modifiers[ModifierPowerful] = this->unit_state.powerful();  // if G6 is unsupported we can still read out powerful set by remote
          }
          else if ((rcode[2] == 'C') && (rcode[3] == '3')) { // FzC3 -> GzC3 System state
            this->system_state = ahex_u8_le(payload[0], payload[1]);
          }
          return;
        default:
          if (payload.size() > 3) {
            auto temp = temp_bytes_to_c10(payload);
            ESP_LOGD(TAG, "Unknown sensor: %" PRI_SV " -> %s -> %.1f C (%.1f F)",
                     PRI_SV_ARGS(rcode),
                     hex_repr(payload).c_str(),
                     temp.f_degc(), temp.f_degf());
          }
          break;
      }
      break;

    case 'M':  // some sort of model? always "3E53" for me, regardless of head unit
      std::copy_n(payload.begin(), std::min(payload.size(), this->detect_responses.M.size()), this->detect_responses.M.begin());
      return;

    case 'V':  // purportedly another version, always "00C0" for me
      std::copy_n(payload.begin(), std::min(payload.size(), this->detect_responses.V.size()), this->detect_responses.V.begin());
      return;

    default:
      break;
  }

  // protocol decoding debug
  // note: well known responses return directly from the switch statements
  // break instead if you want to view their contents down here

  // print everything
  if (this->debug_protocol) {
    ESP_LOGD(TAG, "S21: %" PRI_SV " -> %s %s",
             PRI_SV_ARGS(rcode),
             str_repr(payload).c_str(),
             hex_repr(payload).c_str());
  }

  // print changed values
  if (this->debug_protocol) {
    std::string key(rcode.begin(), rcode.end());
    auto curr = std::vector<uint8_t>(payload.begin(), payload.end());
    if (val_cache[key] != curr) {
      const auto prev = val_cache[key];
      ESP_LOGI(TAG, "S21 %s changed: %s %s -> %s %s",
               key.c_str(),
               str_repr(prev).c_str(),
               hex_repr(prev).c_str(),
               str_repr(curr).c_str(),
               hex_repr(curr).c_str());
      val_cache[key] = curr;
    }
  }
}

void DaikinS21::handle_nak() {
  ESP_LOGW(TAG, "Rx: NAK from S21 for %" PRI_SV, PRI_SV_ARGS(this->tx_command));
  if (this->tx_command == *(this->current_query)) {
    ESP_LOGW(TAG, "Removing %" PRI_SV " from query pool (assuming unsupported)", PRI_SV_ARGS(this->tx_command));
    this->nak_queries.emplace_back(*(this->current_query));
    // current_query iterator will be invalidated, recover index and recreate
    const auto index = std::distance(this->queries.begin(), this->current_query);
    this->queries.erase(this->current_query);
    this->current_query = this->queries.begin() + index;
  } else {
    ESP_LOGW(TAG, "Acknowledging %" PRI_SV " command despite NAK", PRI_SV_ARGS(this->tx_command));
    this->parse_ack();  // don't get stuck retrying unsupported command
  }
}

/**
 * Determine the protocol version according to Faikin documentation.
 *
 * @note This is mostly untested, I own only one Daikin system.
 *
 * @return true protocl version was detected
 * @return false protocol version wasn't detected
 */
bool DaikinS21::determine_protocol_version() {
  static constexpr uint8_t G8_version0[4] = {'0',0,0,0};
  static constexpr uint8_t G8_version2or3[4] = {'0','2',0,0};
  static constexpr uint8_t G8_version31plus[4] = {'0','2','0','0'};

  if (std::ranges::equal(this->detect_responses.G8, G8_version0)) {
    this->protocol_version = {0,0};
    return true;
  }

  if ((is_query_active(StateQuery::NewProtocol) == false)) {
    if (std::ranges::equal(this->detect_responses.G8, G8_version2or3) || std::ranges::equal(this->detect_responses.G8, G8_version31plus)) {
      this->protocol_version = {2,0};  // NAK for NewProtocol rules out 3.0
      return true;
    }
  } else {
    switch (this->detect_responses.GY00) {
      case 300:
        if (std::ranges::equal(this->detect_responses.G8, G8_version2or3)) {
          this->protocol_version = {3,0};  // ACK for NewProtocol means 3.0 has support for this query
          return true;
        }
        if (std::ranges::equal(this->detect_responses.G8, G8_version31plus)) {
          this->protocol_version = {3,1};
          return true;
        }
        break;
      case 320:
        this->protocol_version = {3,2};
        return true;
      case 340:
        this->protocol_version = {3,4};
        return true;
      default:
        break;
    }
  }
  ESP_LOGE(TAG, "Unable to detect a protocol version, will not function: %s, %d", str_repr(this->detect_responses.G8).c_str(), this->detect_responses.GY00);
  return false; // not detected yet or unsupported
}

void DaikinS21::setup() {
  // populate initial messages to poll
  // clang-format off
  queries = {
      // Protocol version detect
      StateQuery::OldProtocol,
      StateQuery::NewProtocol,
      StateQuery::ModelCode,
      MiscQuery::Model,
      MiscQuery::Version,
      // Basic sensor support
      StateQuery::InsideOutsideTemperatures,
      EnvironmentQuery::InsideTemperature,
      EnvironmentQuery::LiquidTemperature,
      EnvironmentQuery::FanSpeed,
      EnvironmentQuery::OutsideTemperature,
      // Standard
      StateQuery::Basic,
      StateQuery::OptionalFeatures,
      StateQuery::SwingOrHumidity,
      StateQuery::SpecialModes,
      StateQuery::DemandAndEcono,
      EnvironmentQuery::FanMode,
      EnvironmentQuery::TargetTemperature,
      EnvironmentQuery::IndoorFrequencyCommandSignal,
      EnvironmentQuery::CompressorFrequency,
      // State
      EnvironmentQuery::UnitState,
      EnvironmentQuery::SystemState,
      // Redundant
      // EnvironmentQuery::PowerOnOff,
      // EnvironmentQuery::IndoorUnitMode,
      // EnvironmentQuery::TemperatureSetPoint,
      // Unused
      // StateQuery::OnOffTimer,  // use home assistant for scheduling
      // Not handled yet
      // StateQuery::ErrorStatus,
      // StateQuery::FanSetPoint,
      // StateQuery::LouvreAngleSetPoint,
  };
  // clang-format on
  current_query = this->queries.begin();
}

void DaikinS21::loop() {
  using Result = DaikinSerial::Result;

  switch (this->serial.service()) {
    case Result::Ack:
      this->parse_ack();
      break;

    case Result::Idle:
      this->do_next_action();
      break;

    case Result::Nak:
      this->handle_nak();
      break;

    case Result::Error:
      this->cycle_completed = true;
      this->activate_climate = false;
      this->activate_swing_mode = false;
      this->activate_preset = false;
      break;

    case Result::Timeout:
      ESP_LOGW(TAG, "Timeout waiting for response to %" PRI_SV, PRI_SV_ARGS(tx_command));
      break;

    default:
    break;
  }
}

/**
 * PollingComponent update loop.
 *
 * Disable the polling loop (this function) if configured to free run.
 * Trigger a cycle if configured for periodic polling.
 */
void DaikinS21::update() {
  if (this->is_free_run()) {
    this->stop_poller();
  } else {
    this->trigger_cycle();
  }
}

void DaikinS21::dump_state() {
  ESP_LOGD(TAG, "** BEGIN STATE *****************************");

  ESP_LOGD(TAG, "  Proto: v%" PRIu8 ".%" PRIu8, this->protocol_version.major, this->protocol_version.minor);
  if (this->debug_protocol) {
    ESP_LOGD(TAG, "      G8: %s  GC: %s  GY00: %" PRIu16 "  M: %s  V: %s",
      str_repr(this->detect_responses.G8).c_str(),
      str_repr(this->detect_responses.GC).c_str(),
      this->detect_responses.GY00,
      str_repr(this->detect_responses.M).c_str(),
      str_repr(this->detect_responses.V).c_str());
  }
  ESP_LOGD(TAG, "   Mode: %s  Action: %s  Preset: %s",
          LOG_STR_ARG(climate::climate_mode_to_string(this->active.mode)),
          LOG_STR_ARG(climate::climate_action_to_string(this->get_climate_action())),
          LOG_STR_ARG(climate::climate_preset_to_string(this->active.preset)));
  ESP_LOGD(TAG, "    Fan: %" PRI_SV " (%" PRIu16 " RPM)  Swing: %s",
          PRI_SV_ARGS(daikin_fan_mode_to_string_view(this->active.fan)), this->fan_rpm,
          (this->support_swing ? LOG_STR_ARG(climate::climate_swing_mode_to_string(this->active.swing)) : "unsupported"));
  ESP_LOGD(TAG, " Target: %.1f C (%.1f F)",
          this->active.setpoint.f_degc(), this->active.setpoint.f_degf());
  ESP_LOGD(TAG, " Inside: %.1f C (%.1f F)",
          this->temp_inside.f_degc(), this->temp_inside.f_degf());
  ESP_LOGD(TAG, "Outside: %.1f C (%.1f F)",
          this->temp_outside.f_degc(), this->temp_outside.f_degf());
  ESP_LOGD(TAG, "   Coil: %.1f C (%.1f F)",
          this->temp_coil.f_degc(), this->temp_coil.f_degf());
  if (this->support_humidity) {
    ESP_LOGD(TAG, "  Humid: %" PRIu8 "%%", this->get_humidity());
  }
  ESP_LOGD(TAG, " Demand: %" PRIu8, this->get_demand());
  ESP_LOGD(TAG, " Cycle Time: %" PRIu32 "ms", this->cycle_time_ms);
  ESP_LOGD(TAG, " UnitState: %" PRIX8 " SysState: %02" PRIX8, this->unit_state.raw, this->system_state.raw);
  if (this->debug_protocol) {
    const auto comma_join = [](const auto& queries) {
      std::string str;
      for (const auto q : queries) {
        str += q;
        if (q != queries.back()) {
          str += ",";
        }
      }
      return str;
    };
    ESP_LOGD(TAG, LOG_STR_ARG(("Queries: " + comma_join(this->queries)).c_str()));
    ESP_LOGD(TAG, LOG_STR_ARG(("  Nak'd: " + comma_join(this->nak_queries)).c_str()));
  }

  ESP_LOGD(TAG, "** END STATE *****************************");
}

void DaikinS21::set_climate_settings(const DaikinSettings &settings) {
  if ((pending.mode != settings.mode) ||
      (pending.setpoint != settings.setpoint) ||
      (pending.fan != settings.fan)) {

    // If this is the first time settings are being applied, we need to force
    // all of them to be applied to the unit so our state will be in sync with
    // the unit and thus rely on this change detection to work in the future.
    // The pending settings mode has been initialized in the object constructor
    // to the unsupported (by this project) CLIMATE_MODE_AUTO in order to detect
    // this one time condition here.
    if (pending.mode == climate::CLIMATE_MODE_AUTO) {
      activate_swing_mode = true;
      activate_preset = true;
    }

    activate_climate = true;
    this->trigger_cycle();
  }

  if (pending.swing != settings.swing) {
    activate_swing_mode = true;
    this->trigger_cycle();
  }

  if (pending.preset != settings.preset) {
    activate_preset = true;
    this->trigger_cycle();
  }

  pending = settings;
}

climate::ClimateAction DaikinS21::resolve_climate_action() {
  switch (this->get_climate_mode()) {
    // modes where the unit is trying to reach a temperature
    case climate::CLIMATE_MODE_HEAT_COOL:
    case climate::CLIMATE_MODE_COOL:
    case climate::CLIMATE_MODE_HEAT:
    case climate::CLIMATE_MODE_AUTO:
      if (this->demand == 0) {
        return climate::CLIMATE_ACTION_IDLE;
      }
      break;
    // modes where the fan needs to be running for the action to be active
    case climate::CLIMATE_MODE_FAN_ONLY:
      if (this->fan_rpm == 0) {
        return climate::CLIMATE_ACTION_IDLE;
      }
      break;
    case climate::CLIMATE_MODE_DRY:
    default:
      break;
  }
  return this->action_reported;
}

void DaikinS21::add_climate_callback(std::function<void(void)> &&callback) {
  this->climate_callback_.add(std::move(callback));
}

void DaikinS21::add_binary_sensor_callback(std::function<void(DaikinUnitState, DaikinSystemState)> &&callback) {
  this->binary_sensor_callback_.add(std::move(callback));
}

} // namespace esphome::daikin_s21
