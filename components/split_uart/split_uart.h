#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"

namespace esphome::split_uart {

/**
 * Split UART Component
 *
 * Wraps two underlying UARTs to allow reads and writes to be directed to different sub-components from a single interface.
 */
class SplitUART : public uart::UARTComponent, public Component {
 public:
  SplitUART(uart::UARTComponent *tx, uart::UARTComponent *rx) : tx(tx), rx(rx) {}

  // Component overrides

  void setup() override {
    this->disable_loop(); // just a proxy, nothing happening here
  }

  // UARTComponent overrides

  void write_array(const uint8_t * const data, const size_t len) override {
    return this->tx->write_array(data, len);
  }

  bool peek_byte(uint8_t * const data) override {
    return this->rx->peek_byte(data);
  }

  bool read_array(uint8_t * const data, const size_t len) override {
    return this->rx->read_array(data, len);
  }

  int available() override {
    return this->rx->available();
  }

  void flush() override {
    return this->tx->flush();
  }

  void load_settings(const bool dump_config) override {
    for (auto *uart : {this->tx, this->rx}) {
      // no support for different UART settings
      uart->set_baud_rate(this->get_baud_rate());
      uart->set_stop_bits(this->get_stop_bits());
      uart->set_data_bits(this->get_data_bits());
      uart->set_parity(this->get_parity());
      uart->load_settings(dump_config);
    }
  }

  void load_settings() override {
    this->load_settings(true);
  }

 protected:
  void check_logger_conflict() override {}  // conflicts will be caught in subcomponent calls

  uart::UARTComponent *tx{};
  uart::UARTComponent *rx{};
};

}  // namespace esphome::daikin_s21
