#include "daikin_s21_queries.h"
#include "daikin_s21_serial.h"
#include "s21.h"
#include "utils.h"

namespace esphome::daikin_s21 {

static const char *const TAG = "daikin_s21.queries";

/**
 * Copy the last result into the query instance and records the length.
 */
void DaikinQuery::set_value(std::span<const uint8_t> payload) {
  // decide where to store value
  uint8_t * dest{};
  if (payload.size() <= this->buffer.internal.size()) {
    // small enough, use internal buffer
    dest = this->buffer.internal.data();
    // clean up existing buffer
    if (this->internal() == false) {
      std::free(this->buffer.external);
    }
  } else if (payload.size() <= this->size) {
    // fits, reuse existing allocation
    dest = this->buffer.external;
  } else {
    // larger, allocate external
    dest = static_cast<uint8_t *>(std::malloc(payload.size()));
    // clean up existing buffer
    if (this->internal() == false) {
      std::free(this->buffer.external);
    }
    if (dest == nullptr) {
      ESP_LOGI(TAG, "result buffer failed to allocate %" PRI_SV ": %s", PRI_SV_ARGS(this->command), hex_repr(payload).c_str());
      payload = payload.first(this->buffer.internal.size());
      dest = this->buffer.internal.data();
    } else {
      this->buffer.external = dest;
    }
  }
  std::ranges::copy(payload, dest);
  this->size = payload.size();
}

void DaikinQuery::clear() {
  this->enabled = false;
  this->set_value(unscheduled_value);
  this->naks = 0;
  this->acked = false;
}

/**
 * Handles an ACK response
 */
void DaikinQuery::ack(const std::span<const uint8_t> payload) {
  this->acked = true;
  this->naks = 0;
  if (this->is_static) {
    this->enabled = false;  // got the query result and know it won't change, disable
  }
  this->set_value(payload);
}

/**
 * Handles a NAK response
 */
void DaikinQuery::nak() {
  this->naks++;
  if (this->failed()) {
    // query failed, disable
    ESP_LOGW(TAG, "disabling %" PRI_SV " as unsupported", PRI_SV_ARGS(this->command));
    this->enabled = false;
    this->set_value(nak_value);
  }
}

} // namespace esphome::daikin_s21
