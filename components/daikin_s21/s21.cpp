#include <cinttypes>
#include "s21.h"

using namespace esphome;

namespace esphome {
namespace daikin_s21 {

#define STX 2
#define ETX 3
#define ACK 6
#define NAK 21

#define S21_RESPONSE_TIMEOUT 250

static const char *const TAG = "daikin_s21";

std::string daikin_climate_mode_to_string(DaikinClimateMode mode) {
  switch (mode) {
    case DaikinClimateMode::Disabled:
      return "Disabled";
    case DaikinClimateMode::Auto:
      return "Auto";
    case DaikinClimateMode::Dry:
      return "Dry";
    case DaikinClimateMode::Cool:
      return "Cool";
    case DaikinClimateMode::Heat:
      return "Heat";
    case DaikinClimateMode::Fan:
      return "Fan";
    default:
      return "UNKNOWN";
  }
}

std::string daikin_fan_mode_to_string(DaikinFanMode mode) {
  switch (mode) {
    case DaikinFanMode::Auto:
      return "Auto";
    case DaikinFanMode::Silent:
      return "Silent";
    case DaikinFanMode::Speed1:
      return "1";
    case DaikinFanMode::Speed2:
      return "2";
    case DaikinFanMode::Speed3:
      return "3";
    case DaikinFanMode::Speed4:
      return "4";
    case DaikinFanMode::Speed5:
      return "5";
    default:
      return "UNKNOWN";
  }
}

uint8_t s21_checksum(uint8_t *bytes, uint8_t len) {
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < len; i++) {
    checksum += bytes[i];
  }
  return checksum;
}

uint8_t s21_checksum(std::vector<uint8_t> bytes) {
  return s21_checksum(&bytes[0], bytes.size());
}

int16_t bytes_to_num(uint8_t *bytes, size_t len) {
  // <ones><tens><hundreds><neg/pos>
  int16_t val = 0;
  val = bytes[0] - '0';
  val += (bytes[1] - '0') * 10;
  val += (bytes[2] - '0') * 100;
  if (len > 3 && bytes[3] == '-')
    val *= -1;
  return val;
}

int16_t bytes_to_num(std::vector<uint8_t> &bytes) {
  return bytes_to_num(&bytes[0], bytes.size());
}

int16_t temp_bytes_to_c10(uint8_t *bytes) { return bytes_to_num(bytes, 4); }

int16_t temp_bytes_to_c10(std::vector<uint8_t> &bytes) {
  return temp_bytes_to_c10(&bytes[0]);
}

int16_t temp_f9_byte_to_c10(uint8_t *bytes) { return (*bytes / 2 - 64) * 10; }

uint8_t c10_to_setpoint_byte(int16_t setpoint) {
  return (setpoint + 3) / 5 + 28;
}

void DaikinS21::set_uarts(uart::UARTComponent *tx, uart::UARTComponent *rx) {
  this->tx_uart = tx;
  this->rx_uart = rx;
}

#define S21_BAUD_RATE 2400
#define S21_STOP_BITS 2
#define S21_DATA_BITS 8
#define S21_PARITY uart::UART_CONFIG_PARITY_EVEN

void DaikinS21::check_uart_settings() {
  for (auto uart : {this->tx_uart, this->rx_uart}) {
    if (uart->get_baud_rate() != S21_BAUD_RATE) {
      ESP_LOGE(
          TAG,
          "  Invalid baud_rate: Integration requested baud_rate %u but you "
          "have %" PRIu32 "!",
          S21_BAUD_RATE, uart->get_baud_rate());
    }
    if (uart->get_stop_bits() != S21_STOP_BITS) {
      ESP_LOGE(
          TAG,
          "  Invalid stop bits: Integration requested stop_bits %u but you "
          "have %u!",
          S21_STOP_BITS, uart->get_stop_bits());
    }
    if (uart->get_data_bits() != S21_DATA_BITS) {
      ESP_LOGE(TAG,
               "  Invalid number of data bits: Integration requested %u data "
               "bits but you have %u!",
               S21_DATA_BITS, uart->get_data_bits());
    }
    if (uart->get_parity() != S21_PARITY) {
      ESP_LOGE(
          TAG,
          "  Invalid parity: Integration requested parity %s but you have %s!",
          LOG_STR_ARG(parity_to_str(S21_PARITY)),
          LOG_STR_ARG(parity_to_str(uart->get_parity())));
    }
  }
}

void DaikinS21::dump_config() {
  ESP_LOGCONFIG(TAG, "DaikinS21:");
  ESP_LOGCONFIG(TAG, "  Update interval: %" PRIu32, this->get_update_interval());
  this->check_uart_settings();
}

bool DaikinS21::wait_byte_available(uint32_t  timeout)
{
  uint32_t start = millis();
  bool reading = false;
  while (true) {
    if (millis() - start > timeout) {
      ESP_LOGW(TAG, "Timeout waiting for byte");
      return false;
    }
    if(this->rx_uart->available())
      return true;
    yield();
  }
}

bool DaikinS21::read_frame(std::vector<uint8_t> &payload) {
  uint8_t byte;
  std::vector<uint8_t> bytes;
  uint32_t start = millis();
  bool reading = false;
  while (true) {
    if (millis() - start > S21_RESPONSE_TIMEOUT) {
      ESP_LOGW(TAG, "Timeout waiting for frame");
      return false;
    }
    while (this->rx_uart->available()) {
      this->rx_uart->read_byte(&byte);
      if (byte == ACK) {
        ESP_LOGW(TAG, "Unexpected ACK waiting to read start of frame");
        continue;
      } else if (!reading && byte != STX) {
        ESP_LOGW(TAG, "Unexpected byte waiting to read start of frame: %x",
                 byte);
        continue;
      } else if (byte == STX) {
        reading = true;
        continue;
      }
      if (byte == ETX) {
        reading = false;
        uint8_t frame_csum = bytes[bytes.size() - 1];
        bytes.pop_back();
        uint8_t calc_csum = s21_checksum(bytes);
        if (calc_csum != frame_csum) {
          // This sometimes happens with G9 reply, no idea why
          if (bytes[0] == 0x47 && bytes[1] == 0x39) {
            calc_csum += 2;
          }
          if (calc_csum != frame_csum) {
            ESP_LOGW(TAG, "Checksum mismatch: %x (frame) != %x (calc from %s)",
            frame_csum, calc_csum,
            hex_repr(&bytes[0], bytes.size()).c_str());
            return false;
          }
        }
        break;
      }
      bytes.push_back(byte);
    }
    if (bytes.size() && !reading)
      break;
    yield();
  }
  payload.assign(bytes.begin(), bytes.end());
  return true;
}

void DaikinS21::write_frame(std::vector<uint8_t> frame) {
  this->tx_uart->write_byte(STX);
  this->tx_uart->write_array(frame);
  this->tx_uart->write_byte(s21_checksum(&frame[0], frame.size()));
  this->tx_uart->write_byte(ETX);
  this->tx_uart->flush();
}

bool DaikinS21::s21_query(std::vector<uint8_t> code) {
  std::string c;
  for (size_t i = 0; i < code.size(); i++) {
    c += code[i];
  }
  this->write_frame(code);

  this->wait_byte_available(S21_RESPONSE_TIMEOUT);
  uint8_t byte;
  if (!this->rx_uart->read_byte(&byte)) {
    ESP_LOGW(TAG, "Timeout waiting for %s response", c.c_str());
    return false;
  }
  if (byte == NAK) {
    ESP_LOGD(TAG, "NAK from S21 for %s query", c.c_str());
    return false;
  }
  if (byte != ACK) {
    ESP_LOGW(TAG, "No ACK from S21 for %s query (was: %d (0x%02X) '%c'", c.c_str(), byte, byte, 
        (byte >= 32 && byte <= 126) ? byte : '?');
    return false;
  }

  std::vector<uint8_t> frame;
  if (!this->read_frame(frame)) {
    ESP_LOGW(TAG, "Failed reading %s response frame", c.c_str());
    return false;
  }

  this->tx_uart->write_byte(ACK);

  std::vector<uint8_t> rcode;
  std::vector<uint8_t> payload;
  for (size_t i = 0; i < frame.size(); i++) {
    if (i < code.size()) {
      rcode.push_back(frame[i]);
    } else {
      payload.push_back(frame[i]);
    }
  }

  return parse_response(rcode, payload);
}

bool DaikinS21::parse_response(std::vector<uint8_t> rcode,
                               std::vector<uint8_t> payload) {
  if (this->debug_protocol) {
    ESP_LOGD(TAG, "S21: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
  }

  if(uint8_starts_with_str(rcode, "G")) {
    if(uint8_starts_with_str(rcode, StateResponse::Basic)) {
      // F1 -> G1 Basic State
      this->power_on = (payload[0] == '1');
      this->mode = (DaikinClimateMode) payload[1];
      this->setpoint = ((payload[2] - 28) * 5);  // Celsius * 10
      this->fan = (DaikinFanMode) payload[3];
      return true;
    } else if(uint8_starts_with_str(rcode, StateResponse::SwingOrHumidity)) {
      // F5 -> G5 -- Swing state
      this->swing_v = payload[0] & 1;
      this->swing_h = payload[0] & 2;
      return true;
    } else if(uint8_starts_with_str(rcode, StateResponse::SpecialModes)) {
      // F6 -> G6 - "powerful" mode
      this->powerful = (payload[0] == '2') ? 1 : 0;
      return true;
    } else if(uint8_starts_with_str(rcode, StateResponse::DemandAndEcono)) {
      // F7 -> G7 - "eco" mode
      this->econo = (payload[1] == '2') ? 1 : 0;
      return true;
    } else if(uint8_starts_with_str(rcode, StateResponse::OldProtocol)) {
      // F8 -> G8 - old protocol version
      if(uint8_starts_with_str(payload, OldProtocol::Protocol0)) {
        this->f8_protocol = 0;
        this->protocol_checked = true;
      } else if(uint8_starts_with_str(payload, OldProtocol::Protocol2_1)) {
        this->f8_protocol = 2;
        this->f8_protocol_variant = 1;
        if (this->fy00_protocol_major == 3) {
          this->fy00_protocol_minor = 0;
          this->protocol_checked = true;
        }
      } else if(uint8_starts_with_str(payload, OldProtocol::Protocol2_2)) {
        this->f8_protocol = 2;
        this->f8_protocol_variant = 2;
        if (this->fy00_protocol_major == 3) {
          this->fy00_protocol_minor = 1;
          this->protocol_checked = true;
        }
      } else {
        return false;
      }
      return true;
    } else if(uint8_starts_with_str(rcode, StateResponse::NewProtocol)) {
      // FY00 -> GY00 - new protocol version
      if(uint8_starts_with_str(payload, NewProtocol::Protocol3_00_or_3_10)) {
        this->fy00_protocol_major = 3;
        if (this->f8_protocol_variant == 1) {
          this->fy00_protocol_minor = 0;
          this->protocol_checked = true;
        } else if (this->f8_protocol_variant == 2) {
          this->fy00_protocol_minor = 1;
          this->protocol_checked = true;
        }
      } else if(uint8_starts_with_str(payload, NewProtocol::Protocol3_20)) {
        this->fy00_protocol_major = 3;
        this->fy00_protocol_minor = 2;
        this->protocol_checked = true;
      } else if(uint8_starts_with_str(payload, NewProtocol::Protocol3_40)) {
        this->fy00_protocol_major = 3;
        this->fy00_protocol_minor = 4;
        this->protocol_checked = true;
      } else {
        return false;
      }
      return true;
    } else if(uint8_starts_with_str(rcode, StateResponse::InsideOutsideTemperatures)) {
      // F9 -> G9 - inside and outside temperature
      this->temp_inside = temp_f9_byte_to_c10(&payload[0]);
      this->temp_outside = temp_f9_byte_to_c10(&payload[1]);
      return true;
    } else if(uint8_starts_with_str(rcode, StateResponse::OptionalFeatures)) {
      // F2 -> G2 - optional features
      if (payload.size() == 4) {
        ESP_LOGD(TAG, "Optional features: %s -> %s -> %s (%s (%s)) (%d)", str_repr(rcode).c_str(),
        str_repr(payload).c_str(), this->little_endian ? "little" : "big", bin_repr(payload, this->little_endian).c_str(), this->little_endian ? "little" : "big", payload.size());

        // https://github.com/revk/ESP32-Faikin/wiki/S21-Protocol#f2-command

        // G2 -> 4:\x00\x00 -> little (00101100 01011100 00000000 00000000 (little)) (4)

        // Response format: G2 [byte0] [byte1] [byte2] [byte3]
        // Payload bytes are bit masks:

        // byte0
        // bit 0 - Unkown. set to 1 on CTXMxxRVMA, ignored by BRP069B41
        // bit 1 - Zero
        // bit 2 - Swing (any kind) is avaiiable
        // bit 3 - Horizontal swing is available
        // bit 4 - Shield bit (0x30)
        // bit 5 - Shield bit (0x30)
        // bit 6 - Zero
        // bit 7 - Zero

        // byte1
        // bit 0 - Unkown. set to 1 on CTXMxxRVMA, ignored by BRP069B41
        // bit 1 - Awlays 1, unknown
        // bit 2 - Zero
        // bit 3 - Unknown. Reflected by BRP069B41 in aircon/model_info. 0 => type=C, 1 => type=N
        // bit 4 - Shield bit (0x30)
        // bit 5 - Shield bit (0x30)
        // bit 6 - Zero
        // bit 7 - Zero

        // byte2 - seen to be 0x00 on startup
        // bit 7 - Set to 1 by DJ command. Purpose is unknown.

        // byte3
        // bit 0 - Zero
        // bit 1 - "humidity" operation mode is available
        // bit 2 - Zero
        // bit 3 - Zero
        // bit 4 - Humidity setting is available for additional operation modes (see matrix below)
        // bit 5 - Zero
        // bit 6 - Zero
        // bit 7 - Always 1. Perhaps shield ?

        return true;
      } else {
        ESP_LOGW(TAG, "S21 issue, payload should be 4: %s -> %s (%d)", str_repr(rcode).c_str(),
        str_repr(payload).c_str(), payload.size());
        return false;
      }
    } else if(uint8_starts_with_str(rcode, StateResponse::OnOffTimer)) {
      // F3 -> G3 - on/off timer
      if (payload.size() == 4) {
        ESP_LOGD(TAG, "On/off timer: %s -> %s -> %s (%s) (%d)", str_repr(rcode).c_str(),
        str_repr(payload).c_str(), bin_repr(payload, this->little_endian).c_str(), this->little_endian ? "little" : "big", payload.size());

        // https://github.com/revk/ESP32-Faikin/wiki/S21-Protocol#f3-command

        // On-off timer
        // 0\x95\x80\x00

        // byte 0:
        // Bit 0 - On timer is set
        // Bit 1 - Off timer is set
        // Bits 4, 5 - shield bits, making up 0x30 (ASCII '0')

        // byte 1 - On timer setting

        // byte 2 - Off timer setting

        // byte 3 - Reports 0x00 after bootup, changed to 0x30 by DJ. Meaning unknown.

        // Timer settings range from 1 to 12 hours, 
        // and corresponding values are: 
        // 0x36, 0x3C, 0x42, 0x48, 0x4E, 0x54, 
        // 0x5A, 0x60, 0x66, 0x6C, 0x72, 0x78. 
        // In other words, time value starts from 0x30 (which would be 0), 
        // then one hour equals to an increment of 6. 
        // This gives an idea that perhaps timer granularity 
        // is 10 minutes (1/6 of hour), but it's unclear if the 
        // unit would accept them properly.

        // On majority of units if the timer is disabled, 
        // the respective setting bytes are set to 0xFE. 
        // However, on ATX20K2V1B and S22ZTES-W they read as 0x30 in this case.
        return true;
      } else {
        ESP_LOGW(TAG, "S21 issue, payload should be 4: %s -> %s (%d)", str_repr(rcode).c_str(),
        str_repr(payload).c_str(), payload.size());
        return false;
      }
    } else if(uint8_starts_with_str(rcode, StateResponse::ErrorStatus)) {
      // F4 -> G4 - error status
      if (payload.size() == 4) {
        ESP_LOGD(TAG, "Error status: %s -> %s -> %s (%s) (%d)", str_repr(rcode).c_str(),
        str_repr(payload).c_str(), bin_repr(payload, this->little_endian).c_str(), this->little_endian ? "little" : "big", payload.size());

        // https://github.com/revk/ESP32-Faikin/wiki/S21-Protocol#f4-command

        // 0\xB2\x80\x00 Error status: G4 -> 0\x00\x80\x00 -> 00110000 00000000 10000000 00000000 (4)

        // byte0: 0x30 ('0')
        // byte1: 0x00
        // byte2
        // Bit 5: Conditioner internal error flag. If reported as 1, BRP069B41 only polls 4 commands F1-F3-F4-F2 and reports in /aircon/model_info:
        // ret=SERIAL IF FAILURE,err=252
        // Experiments show, that once read, the bit resets to 0. It's not known which actions cause it to raise.
        // byte3: 0x30 ('0')

        return true;
      } else {
        ESP_LOGW(TAG, "S21 issue, payload should be 4: %s -> %s (%d)", str_repr(rcode).c_str(),
        str_repr(payload).c_str(), payload.size());
        return false;
      }
    } else if(uint8_starts_with_str(rcode, StateResponse::ModelCode)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else if(uint8_starts_with_str(rcode, StateResponse::IRCounter)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else if(uint8_starts_with_str(rcode, StateResponse::V2OptionalFeatures)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else if(uint8_starts_with_str(rcode, StateResponse::PowerConsumption)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else if(uint8_starts_with_str(rcode, StateResponse::LouvreAngle)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else if(uint8_starts_with_str(rcode, StateResponse::V3OptionalFeatures)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else if(uint8_starts_with_str(rcode, StateResponse::AllowedTemperatureRange)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else if(uint8_starts_with_str(rcode, StateResponse::ModelName)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else if(uint8_starts_with_str(rcode, StateResponse::ProductionInformation)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else if(uint8_starts_with_str(rcode, StateResponse::ProductionOrder)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else if(uint8_starts_with_str(rcode, StateResponse::IndoorProductionInformation)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else if(uint8_starts_with_str(rcode, StateResponse::OutdoorProductionInformation)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    }
  } else if(uint8_starts_with_str(rcode, "S")) {
    if(uint8_starts_with_str(rcode, EnvironmentResponse::InsideTemperature)) {
      // RH -> SH - inside temperature
      this->temp_inside = temp_bytes_to_c10(payload);
      return true;
    } else if(uint8_starts_with_str(rcode, EnvironmentResponse::LiquidTemperature)) {
      // RI -> SI - coil temperature
      this->temp_coil = temp_bytes_to_c10(payload);
      return true;
    } else if(uint8_starts_with_str(rcode, EnvironmentResponse::OutsideTemperature)) {
      // Ra -> Sa - outside temperature
      this->temp_outside = temp_bytes_to_c10(payload);
      return true;
    } else if(uint8_starts_with_str(rcode, EnvironmentResponse::FanSpeed)) {
      // RL -> SL - fan speed
      this->fan_rpm = bytes_to_num(payload) * 10;
      return true;
    } else if(uint8_starts_with_str(rcode, EnvironmentResponse::CompressorFrequency)) {
      // Rd -> Sd - compressor state / frequency? Idle if 0.
      this->idle =
      (payload[0] == '0' && payload[1] == '0' && payload[2] == '0');
      return true;
    } else if(uint8_starts_with_str(rcode, EnvironmentResponse::PowerOnOff)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else if(uint8_starts_with_str(rcode, EnvironmentResponse::IndoorUnitMode)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else if(uint8_starts_with_str(rcode, EnvironmentResponse::TemperatureSetPoint)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else if(uint8_starts_with_str(rcode, EnvironmentResponse::OnTimerSetting)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else if(uint8_starts_with_str(rcode, EnvironmentResponse::OffTimerSetting)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else if(uint8_starts_with_str(rcode, EnvironmentResponse::FanMode)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else if(uint8_starts_with_str(rcode, EnvironmentResponse::FanSetPoint)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else if(uint8_starts_with_str(rcode, EnvironmentResponse::LouvreAngleSetPoint)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else if(uint8_starts_with_str(rcode, EnvironmentResponse::VerticalSwingAngle)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else if(uint8_starts_with_str(rcode, EnvironmentResponse::TargetTemperature)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else if(uint8_starts_with_str(rcode, EnvironmentResponse::IndoorFrequencyCommandSignal)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else if(uint8_starts_with_str(rcode, EnvironmentResponse::IndoorHumidity)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else if(uint8_starts_with_str(rcode, EnvironmentResponse::CompressorOnOff)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else if(uint8_starts_with_str(rcode, EnvironmentResponse::UnitState)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else if(uint8_starts_with_str(rcode, EnvironmentResponse::SystemState)) {
      ESP_LOGW(TAG, "S21 unhandled: %s -> %s (%d)", str_repr(rcode).c_str(),
             str_repr(payload).c_str(), payload.size());
      return true;
    } else {
      // default
      if (payload.size() > 3) {
        int8_t temp = temp_bytes_to_c10(payload);
        ESP_LOGD(TAG, "Unknown temp: %s -> %s -> %s -> %s (%s) -> %.1f C (%.1f F)",
                 str_repr(rcode).c_str(), str_repr(payload).c_str(), hex_repr(payload).c_str(), bin_repr(payload, this->little_endian).c_str(), this->little_endian ? "little" : "big",
                 c10_c(temp), c10_f(temp));
        return false;
      }
    }
  }
  ESP_LOGD(TAG, "Unknown response %s -> %s -> %s -> %s (%s)", str_repr(rcode).c_str(), 
           str_repr(payload).c_str(), hex_repr(payload).c_str(), bin_repr(payload, this->little_endian).c_str(), this->little_endian ? "little" : "big");
  return false;
}

bool DaikinS21::run_queries(std::vector<std::string> queries) {
  bool success = true;

  for (auto q : queries) {
    success = this->run_query(q) && success;
  }

  return success;  // True if all queries successful
}

bool DaikinS21::run_query(std::string query) {
  std::vector<uint8_t> code(query.begin(), query.end());
  if(query == "FY00" && this->f8_protocol != -1){
    this->s21_query(code);
    return true; // special case where FY00 can return NAK
  }
  return this->s21_query(code);
}

/**
 * Runs the next startup query command
 * @return true if all the startup queries have been run successfully
 */
bool DaikinS21::run_next_startup_query() {
  if (startup_queries.size() == 0) {
    ESP_LOGW(TAG, "Startup query size is 0");
    return false; // special case if queries are empty
  }
  if (startup_query_index < startup_queries.size()) {
    if (this->debug_protocol) {
      ESP_LOGD(TAG, "Running startup query: %s", startup_queries[this->startup_query_index].c_str());
    }
    if(this->run_query(startup_queries[this->startup_query_index])) {
      startup_query_index++; // increment once this query is successful
    }
  } else {
    startup_complete = true; // if all queries are done, set startup_complete to true
  }

  return startup_complete;  // true if all queries successful else false
}

/**
 * Runs the next required query command
 * @return true if query was run successfully
 */
bool DaikinS21::run_next_required_query() {
  if (required_queries.size() == 0) {
    ESP_LOGW(TAG, "Required query size is 0");
    return false; // special case if queries are empty
  }
  if (required_query_index >= required_queries.size()){
    required_query_index = 0; // reset the index to 0
  }
  if (this->debug_protocol) {
    ESP_LOGD(TAG, "Running required query: %s", required_queries[this->required_query_index].c_str());
  }
  bool success = this->run_query(required_queries[this->required_query_index]);
  if (success) {
    required_query_index++; // increment once this query is successful
  }
  return success;
}

/**
 * Runs the next optional query command
 * @return true if query was run successfully
 */
bool DaikinS21::run_next_optional_query() {
  if (optional_queries.size() == 0) {
    ESP_LOGW(TAG, "Optional query size is 0");
    return false; // special case if queries are empty
  }
  if (optional_query_index >= optional_queries.size()){
    optional_query_index = 0; // reset the index to 0
  }
  if (this->debug_protocol) {
    ESP_LOGD(TAG, "Running optional query: %s", optional_queries[this->optional_query_index].c_str());
  }
  bool success = this->run_query(optional_queries[this->optional_query_index]);
  if (success) {
    optional_query_index++; // increment once this query is successful
  }
  return success;
}

void DaikinS21::set_queries() {
  ESP_LOGD(TAG, "Setting queries with protocol %s", this->get_protocol_version());
  this->startup_queries = this->get_startup_queries();
  this->required_queries = this->get_required_update_queries();
  this->optional_queries = this->get_optional_update_queries();
}

// Protocol 0
//  Supported R commands: RA, RB, RC, RD, RE, RF, RG, RH, RI, RK, RL, RM, RN, RW, RX, Ra, Rb, Rd, Re, Rg, Rz
//  Supported F commands: F1, F2, F3, F4, F5, F8
//  Supported miscellaneous commands: A, M, V
//  BRP069B41 startup sequence: F2 F1 F3 F4 F5 F8 RH Ra M
//  BRP069B41 polling loop: F2 F1 F3 F4 F5 F8 [sensor]
// Protocol 2
//  Supported R commands: RA, RB, RC, RD, RE, RF, RG, RH, RI, RK, RL, RM, RN, RW, RX, Ra, Rb, Rd, Re, Rg, Rz
//  Supported miscellaneous commands: A, M, V, VS000M
//  BRP068B41 startup sequence: F2 F1 F3 F4 F5 F8 F9 F6 F7 FB FG FK FM FN FP FQ FS FT FC FY00 M DJ2010
//  BRP068B41 polling loop: F2 F1 F3 F4 F5 F8 F9 F6 F7 FB FG FK FM FN FP FQ FS FT [sensor]
// Protocol 3.0
//  BRP069B41 startup sequence: F2 F1 F3 F4 F5 F8 F9 F6 F7 FB FG FK FM FN FP FQ FS FT FC FY00 FY10 FY20 M DJ2030 DJ2010
//  BRP069B41 polling loop: F2 F1 F3 F4 F5 F8 F9 F6 F7 FB FG FK FM FN FP FQ FS FT [sensor]
// Protocol 3.1
//  BRP069B41 startup sequence: F2 F1 F3 F4 F5 F8 F9 F6 F7 FB FG FK FM FN FP FQ FS FT FC FY00 FY10 FY20 M VS000M DJ4030
//  BRP069B41 polling loop: F2 F1 F3 F4 F5 F8 F9 F6 F7 FB FG FK FM FN FP FQ FS FT [sensor]
// Protocol 3.2
//  BRP069B41 startup sequence: F2 F1 F3 F4 F5 F8 F9 F6 F7 FB FG FK FM FN FP FQ FS FT FC FY00 FY10 FY20 FU00 FU02 VS000M DJ5010 D70000
//  BRP069B41 polling loop: F2 F1 F3 F4 F5 F8 F9 F6 F7 FB FG FK FM FN FP FQ FS FT FU02 FU04 [sensor]
//  BRP069C41 startup sequence: F2 F1 F3 F4 F5 F8 F9 F6 F7 FB FG FK FM FN FP FQ FS FT FC FY00 FY10 FY20 FU00 FU02 VS000M Rd RL RH RN RI Ra RX FX00 FX10 FX20 FX30 FX40 FX50 FX60 FX70 FX80 Rz52 Rz72 FX90 FXA0 FXB0 FXC0 DY10 DY20

// Below commands, queried by BRP069B41 controller are listed in their order. 
// This gives a good overview of which commands are actually known by the controller and officially used by Daikin. 
// [sensor] denotes one R family command out of sensor query sequence (TBD). 
// I. e. the controller asks one particular each poll iteration. 
// On next iteration next sensor will be queried.

// TODO set up start up queries and then polling loop queries

const char* DaikinS21::get_protocol_version() {
  if (!this->protocol_checked)
  {
    return HumanReadableProtocol::ProtocolUnknown;
  }
  switch (f8_protocol)
  {
    case 0:
      return HumanReadableProtocol::Protocol0;
    case 2: 
      switch (fy00_protocol_minor)
      {
      case 0:
        return HumanReadableProtocol::Protocol3_0;
      case 1:
        return HumanReadableProtocol::Protocol3_1;
      case 2:
        return HumanReadableProtocol::Protocol3_2;
      case 4:
        return HumanReadableProtocol::Protocol3_4;
      default:
        return HumanReadableProtocol::Protocol2;
      }
    default:
      return HumanReadableProtocol::ProtocolUnknown;
  }
}

std::vector<std::string> DaikinS21::get_startup_queries(){
  const char* protocol = this->get_protocol_version();
  if (protocol == HumanReadableProtocol::ProtocolUnknown) {
    return {StateQuery::OldProtocol, StateQuery::NewProtocol};
  } else if (protocol == HumanReadableProtocol::Protocol0) 
  {
    //  BRP069B41 startup sequence: F2 F1 F3 F4 F5 F8 
    // RH Ra M
    return {
      StateQuery::OldProtocol, 
      StateQuery::NewProtocol,
      StateQuery::OptionalFeatures, // F2
      StateQuery::Basic, // F1
      StateQuery::OnOffTimer, // F3
      StateQuery::ErrorStatus, // F4
      StateQuery::SwingOrHumidity, //F5
      EnvironmentQuery::InsideTemperature, // RH
      EnvironmentQuery::OutsideTemperature // Ra
    };
  } else if (protocol == HumanReadableProtocol::Protocol2) {
    ESP_LOGE(TAG, "Protocol %s is not supported yet", protocol);
    this->mark_failed();
    return {StateQuery::OldProtocol, StateQuery::NewProtocol};
  } else if (protocol == HumanReadableProtocol::Protocol3_0) {
    ESP_LOGE(TAG, "Protocol %s is not supported yet", protocol);
    this->mark_failed();
    return {StateQuery::OldProtocol, StateQuery::NewProtocol};
  } else if (protocol == HumanReadableProtocol::Protocol3_1) {
    ESP_LOGE(TAG, "Protocol %s is not supported yet", protocol);
    this->mark_failed();
    return {StateQuery::OldProtocol, StateQuery::NewProtocol};
  } else if (protocol == HumanReadableProtocol::Protocol3_2) {
    ESP_LOGE(TAG, "Protocol %s is not supported yet", protocol);
    this->mark_failed();
    return {StateQuery::OldProtocol, StateQuery::NewProtocol};
  } else if (protocol == HumanReadableProtocol::Protocol3_4) {
    ESP_LOGE(TAG, "Protocol %s is not supported yet", protocol);
    this->mark_failed();
    return {StateQuery::OldProtocol, StateQuery::NewProtocol};
  } else {
    ESP_LOGE(TAG, "Protocol %s is not supported yet", protocol);
    this->mark_failed();
    return {StateQuery::OldProtocol, StateQuery::NewProtocol};
  }
}

std::vector<std::string> DaikinS21::get_required_update_queries(){
  const char* protocol = this->get_protocol_version();
  if (protocol == HumanReadableProtocol::ProtocolUnknown) {
    return {};
  } else if (protocol == HumanReadableProtocol::Protocol0) {
    //  BRP069B41 polling loop: F2 F1 F3 F4 F5 F8 [sensor]
    // RA, RB, RC, RD, RE, RF, RG, RH, RI, RK, RL, RM, RN, RW, RX, Ra, Rb, Rd, Re, Rg, Rz
    return {
      StateQuery::OptionalFeatures, // F2
      StateQuery::Basic, // F1
      StateQuery::OnOffTimer, // F3
      StateQuery::ErrorStatus, // F4
      StateQuery::SwingOrHumidity, //F5
      EnvironmentQuery::CompressorFrequency // Rd
    };
  } else if (protocol == HumanReadableProtocol::Protocol2) {
    ESP_LOGE(TAG, "Protocol %s is not supported yet", protocol);
    this->mark_failed();
    return {};
  } else if (protocol == HumanReadableProtocol::Protocol3_0) {
    ESP_LOGE(TAG, "Protocol %s is not supported yet", protocol);
    this->mark_failed();
    return {};
  } else if (protocol == HumanReadableProtocol::Protocol3_1) {
    ESP_LOGE(TAG, "Protocol %s is not supported yet", protocol);
    this->mark_failed();
    return {};
  } else if (protocol == HumanReadableProtocol::Protocol3_2) {
    ESP_LOGE(TAG, "Protocol %s is not supported yet", protocol);
    this->mark_failed();
    return {};
  } else if (protocol == HumanReadableProtocol::Protocol3_4) {
    ESP_LOGE(TAG, "Protocol %s is not supported yet", protocol);
    this->mark_failed();
    return {};
  } else {
    ESP_LOGE(TAG, "Protocol %s is not supported yet", protocol);
    this->mark_failed();
    return {};
  }
}

// These queries might fail but they won't affect the basic functionality
// F6, F7 and F9 should only be run for protocols that support it (after F8 and FY00)
// std::vector<std::string> failable_queries = {EnvironmentQuery::InsideTemperature, EnvironmentQuery::LiquidTemperature, EnvironmentQuery::OutsideTemperature, EnvironmentQuery::FanSpeed};
std::vector<std::string> DaikinS21::get_optional_update_queries(){
  const char* protocol = this->get_protocol_version();
  if (protocol == HumanReadableProtocol::ProtocolUnknown) {
    return {};
  } else if (protocol == HumanReadableProtocol::Protocol0) {
    return {
      EnvironmentQuery::InsideTemperature, 
      EnvironmentQuery::LiquidTemperature, 
      EnvironmentQuery::OutsideTemperature, 
      EnvironmentQuery::FanSpeed
    };
  } else if (protocol == HumanReadableProtocol::Protocol2) {
    ESP_LOGE(TAG, "Protocol %s is not supported yet", protocol);
    this->mark_failed();
    return {};
  } else if (protocol == HumanReadableProtocol::Protocol3_0) {
    ESP_LOGE(TAG, "Protocol %s is not supported yet", protocol);
    this->mark_failed();
    return {};
  } else if (protocol == HumanReadableProtocol::Protocol3_1) {
    ESP_LOGE(TAG, "Protocol %s is not supported yet", protocol);
    this->mark_failed();
    return {};
  } else if (protocol == HumanReadableProtocol::Protocol3_2) {
    ESP_LOGE(TAG, "Protocol %s is not supported yet", protocol);
    this->mark_failed();
    return {};
  } else if (protocol == HumanReadableProtocol::Protocol3_4) {
    ESP_LOGE(TAG, "Protocol %s is not supported yet", protocol);
    this->mark_failed();
    return {};
  } else {
    ESP_LOGE(TAG, "Protocol %s is not supported yet", protocol);
    this->mark_failed();
    return {};
  }
}

void DaikinS21::update() {
  // if protocol has not been set then we run the protocol queries
  if(!this->protocol_checked || this->f8_protocol == 2 && this->fy00_protocol_major == -1) {
    // protocol_checked is set true when:
    // - F8 returns 0
    // - F8 returns one of the 2 version 2 variants and FY00 returns 3
    // - FY00 returns protocol 3.2
    // - FY00 returns protocol 3.0/3.1 and F8 is already one of the version 2 variants
    // special case is when f8 version is 2 and fy00 is NAK, then we need to just assume protocol version 2
    this->run_next_startup_query();
    if (this->protocol_checked){ // when it first gets set to true
      this->set_queries(); // set the three sets of queries
      this->little_endian=is_little_endian(); // sets endian-ness
    }
  } else {
    if(!this->startup_complete) {
      // startup queries can run at once and block
      this->startup_complete = this->run_next_startup_query();
      ESP_LOGI(TAG, "Daikin S21 startup complete: %s", YESNO(this->startup_complete));
    } else {
      // required updates should run one per loop
      if (this->run_next_required_query()) {
        // optional updates should run one per loop and only if the required update succeeded
        this->run_next_optional_query();
        if(!this->ready) {
          ESP_LOGI(TAG, "Daikin S21 Ready");
          this->ready = true;
        }
      }
      if (this->debug_protocol) {
        this->dump_state();
      }
    }
  }

#ifdef S21_EXPERIMENTS
  ESP_LOGD(TAG, "** UNKNOWN QUERIES **");
  // auto experiments = {"F2", "F3", "F4", "F8", "F9", "F0", "FA", "FB", "FC",
  //                     "FD", "FE", "FF", "FG", "FH", "FI", "FJ", "FK", "FL",
  //                     "FM", "FN", "FO", "FP", "FQ", "FR", "FS", "FT", "FU",
  //                     "FV", "FW", "FX", "FY", "FZ"};
  // Observed BRP device querying these.
  std::vector<std::string> experiments = {"F2", "F3", "F4", "RN",
                                          "RX", "RD", "M",  "FU0F"};
  this->run_queries(experiments);
#endif
}

void DaikinS21::dump_state() {
  ESP_LOGD(TAG, "** BEGIN STATE *****************************");

  ESP_LOGD(TAG, "  Power: %s", ONOFF(this->power_on));
  ESP_LOGD(TAG, "   Mode: %s (%s)",
           daikin_climate_mode_to_string(this->mode).c_str(),
           this->idle ? "idle" : "active");
  float degc = this->setpoint / 10.0;
  float degf = degc * 1.8 + 32.0;
  ESP_LOGD(TAG, " Target: %.1f C (%.1f F)", degc, degf);
  ESP_LOGD(TAG, "    Fan: %s (%d rpm)",
           daikin_fan_mode_to_string(this->fan).c_str(), this->fan_rpm);
  ESP_LOGD(TAG, "  Swing: H:%s V:%s", YESNO(this->swing_h),
           YESNO(this->swing_h));
  ESP_LOGD(TAG, " Inside: %.1f C (%.1f F)", c10_c(this->temp_inside),
           c10_f(this->temp_inside));
  ESP_LOGD(TAG, "Outside: %.1f C (%.1f F)", c10_c(this->temp_outside),
           c10_f(this->temp_outside));
  ESP_LOGD(TAG, "   Coil: %.1f C (%.1f F)", c10_c(this->temp_coil),
           c10_f(this->temp_coil));
  ESP_LOGD(TAG, "    Protocol: %s (F8: %d [variant: %d], FY00: [major: %d, minor: %d])",
           this->get_protocol_version(), this->f8_protocol, this->f8_protocol_variant, this->fy00_protocol_major, this->fy00_protocol_minor);
  ESP_LOGD(TAG, "System is %s-endian", this->little_endian ? "little" : "big");
  ESP_LOGD(TAG, "** END STATE *****************************");
}

void DaikinS21::set_daikin_climate_settings(bool power_on,
                                            DaikinClimateMode mode,
                                            float setpoint,
                                            DaikinFanMode fan_mode) {
  // clang-format off
  std::vector<uint8_t> cmd = {
    (uint8_t)(power_on ? '1' : '0'),
    (uint8_t) mode,
    c10_to_setpoint_byte(lroundf(round(setpoint * 2) / 2 * 10.0)),
    (uint8_t) fan_mode
  };
  // clang-format on
  ESP_LOGD(TAG, "Sending basic climate CMD (D1): %s", str_repr(cmd).c_str());
  if (!this->send_cmd({'D', '1'}, cmd)) {
    ESP_LOGW(TAG, "Failed basic climate CMD");
  } else {
    this->update();
  }
}

void DaikinS21::set_swing_settings(bool swing_v, bool swing_h) {
  std::vector<uint8_t> cmd = {
      (uint8_t) ('0' + (swing_h ? 2 : 0) + (swing_v ? 1 : 0) +
                 (swing_h && swing_v ? 4 : 0)),
      (uint8_t) (swing_v || swing_h ? '?' : '0'), '0', '0'};
  ESP_LOGD(TAG, "Sending swing CMD (D5): %s", str_repr(cmd).c_str());
  if (!this->send_cmd({'D', '5'}, cmd)) {
    ESP_LOGW(TAG, "Failed swing CMD");
  } else {
    this->update();
  }
}

void DaikinS21::set_powerful_settings(bool value)
{
  std::vector<uint8_t> cmd = {
      (uint8_t) ('0' + (value ? 2 : 0)), '0', '0', '0'};
  ESP_LOGD(TAG, "Sending swing CMD (D6): %s", str_repr(cmd).c_str());
  if (!this->send_cmd({'D', '6'}, cmd)) {
    ESP_LOGW(TAG, "Failed powerful CMD");
  } else {
    this->update();
  }
}

void DaikinS21::set_econo_settings(bool value)
{
  std::vector<uint8_t> cmd = {
      '0', (uint8_t) ('0' + (value ? 2 : 0)), '0', '0'};
  ESP_LOGD(TAG, "Sending swing CMD (D7): %s", str_repr(cmd).c_str());
  if (!this->send_cmd({'D', '7'}, cmd)) {
    ESP_LOGW(TAG, "Failed econo CMD");
  } else {
    this->update();
  }
}

bool DaikinS21::send_cmd(std::vector<uint8_t> code,
                         std::vector<uint8_t> payload) {
  std::vector<uint8_t> frame;
  uint8_t byte;

  for (auto b : code) {
    frame.push_back(b);
  }
  for (auto b : payload) {
    frame.push_back(b);
  }

  this->write_frame(frame);
  this->wait_byte_available(S21_RESPONSE_TIMEOUT);
  if (!this->rx_uart->read_byte(&byte)) {
    ESP_LOGW(TAG, "Timeout waiting for ACK to %s", str_repr(frame).c_str());
    return false;
  }
  if (byte == NAK) {
    ESP_LOGW(TAG, "Got NAK for frame: %s", str_repr(frame).c_str());
    return false;
  }
  if (byte != ACK) {
    ESP_LOGW(TAG, "Unexpected byte waiting for ACK: %s",
             str_repr(&byte, 1).c_str());
    return false;
  }

  return true;
}

}  // namespace daikin_s21
}  // namespace esphome
