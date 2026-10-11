#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <string_view>
#include <span>
#include "daikin_s21_types.h"

namespace esphome::daikin_s21 {

/**
 * Class for holding query parameters, const information about each query.
 */
class DaikinQueryParams {
  using handler_fn = void (DaikinS21::*)(std::span<const uint8_t>);

public:
  constexpr DaikinQueryParams(const std::string_view command, const ReadoutRequests readout_requests, const ProtocolVersion protocol_version, const bool is_static = false, const uint8_t response_length = 0, const handler_fn handler = nullptr)
    : command(command), handler(handler), readout_requests(readout_requests), protocol_version(protocol_version), response_length(response_length), is_static(is_static) {}

  constexpr DaikinQueryParams(const std::string_view command, const ReadoutRequest readout_request, const ProtocolVersion protocol_version, const bool is_static = false, const uint8_t response_length = 0, const handler_fn handler = nullptr)
    : DaikinQueryParams(command, 1<<readout_request, protocol_version, is_static, response_length, handler) {}

  std::string_view command{};
  handler_fn handler{};
  ReadoutRequests readout_requests{};
  ProtocolVersion protocol_version{Protocol0};
  uint8_t response_length{};
  bool is_static{};
};

/**
 * Class for holding periodic query state.
 *
 * @note Don't move or copy once enabled as for simplicity's sake the special member functions are omitted.
 */
class DaikinQueryState {
  using handler_fn = void (DaikinS21::*)(std::span<const uint8_t>);
  static inline constexpr uint8_t unscheduled_value[] = {'N','/','A'};
  static inline constexpr uint8_t nak_value[] = {'N','A','K'};

 public:
  constexpr DaikinQueryState(const DaikinQueryParams &params) : params(params), size(std::size(unscheduled_value))
  {
    static_assert(std::size(unscheduled_value) <= sizeof(this->buffer.internal));
    std::ranges::copy(unscheduled_value, this->buffer.internal.begin());
  }

  // filter binary predicates
  static constexpr bool IsEnabled(const DaikinQueryState &query) { return query.enabled; }
  static constexpr bool IsAckedStatic(const DaikinQueryState &query) { return query.success() && query.params.is_static; }
  static constexpr bool IsFailed(const DaikinQueryState &query) { return query.failed(); }
  static constexpr bool IsDebug(const DaikinQueryState &query) { return query.is_debug; }

  // command fetching projection for std::ranges use
  static constexpr std::string_view GetCommand(const DaikinQueryState &query) { return query.params.command; }

  // access helpers
  constexpr bool internal() const { return this->size <= sizeof(this->buffer.internal); }
  constexpr const uint8_t* data() const { return internal() ? buffer.internal.data() : buffer.external; }
  constexpr std::span<const uint8_t> value() const { return { this->data(), size }; }
  constexpr uint8_t operator[](const std::size_t idx) const { return this->value()[idx]; }
  constexpr bool success() const { return this->acked; }
  constexpr bool failed() const { return this->naks >= 3; }
  constexpr bool ready() const { return this->success() || this->failed() || (this->enabled == false); }

  // actions
  void clear();
  void ack(std::span<const uint8_t> payload);
  void nak(std::span<const uint8_t> payload = nak_value);

  // state
  const DaikinQueryParams &params;

 private:
  void set_value(std::span<const uint8_t> payload);

  union {
    std::array<uint8_t, STANDARD_PAYLOAD_SIZE> internal;
    uint8_t* external;
  } buffer{};  // last received value
  uint8_t size{};
  uint8_t acked :1{};
  uint8_t naks :2{};
 public:
  uint8_t enabled :1{};
  uint8_t is_debug :1{};
};

} // namespace esphome::daikin_s21
