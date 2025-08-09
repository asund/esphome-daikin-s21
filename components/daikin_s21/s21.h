#pragma once

#include <bitset>
#include <compare>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "daikin_s21_fan_modes.h"

namespace esphome {
namespace daikin_s21 {

// printf format specifier macros for std::string_view
#define PRI_SV ".*s"
#define PRI_SV_ARGS(x) (x).size(), (x).data()

class DaikinSerial {
public:
  static constexpr uint32_t S21_MAX_COMMAND_SIZE{4};
  static constexpr uint32_t S21_PAYLOAD_SIZE{4};

  enum class Result : uint8_t {
    Idle,
    Ack,
    Nak,
    Error,
    Timeout,
    Busy,
  };
  
  DaikinSerial() {};
  DaikinSerial(uart::UARTComponent *tx, uart::UARTComponent *rx);
  
  Result service();
  Result send_frame(std::string_view cmd, std::span<const uint8_t> payload = {});
  void flush_input();
  
  std::vector<uint8_t> response{};
  bool debug{false};

private:
  Result handle_rx(uint8_t byte);

  enum class CommState : uint8_t {
    Idle = 0,
    CommandAck,
    QueryAck,
    QueryStx,
    QueryEtx,
    AckResponseDelay,
    NextTxDelay,
    ErrorDelay,
  };

  uart::UARTComponent *tx_uart{};
  uart::UARTComponent *rx_uart{};
  CommState comm_state{};
  uint32_t last_event_time_ms{};
};

/**
 * Class representing a temperature in degrees C scaled by 10, the most granular internal temperature measurement format
 */
struct DaikinC10 {
  constexpr DaikinC10() = default;

  template <typename T, typename std::enable_if_t<std::is_floating_point_v<T>, bool> = true>
  constexpr DaikinC10(const T valf) : value((static_cast<int16_t>(valf * 10 * 2) + 1) / 2) {} // round to nearest 0.1C

  template <typename T, typename std::enable_if_t<std::is_integral_v<T>, bool> = true>
  constexpr DaikinC10(const T vali) : value(vali) {}

  explicit constexpr operator float() const { return value / 10.0F; }
  explicit constexpr operator int16_t() const { return value; }
  constexpr float f_degc() const { return static_cast<float>(*this); }
  constexpr float f_degf() const { return celsius_to_fahrenheit(static_cast<float>(*this)); }

  constexpr bool operator==(const DaikinC10 &other) const = default;
  
private:
  int16_t value{};
};

/**
 * Unit state (RzB2) bitfield decoder
 */
struct DaikinUnitState {
  constexpr DaikinUnitState(const uint8_t value = 0U) : raw(value) {}
  constexpr bool powerful() const { return (this->raw & 0x1) != 0; }
  constexpr bool defrost() const { return (this->raw & 0x2) != 0; }
  constexpr bool active() const { return (this->raw & 0x4) != 0; }
  constexpr bool online() const { return (this->raw & 0x8) != 0; }
  uint8_t raw{};
};

/**
 * System state (RzC3) bitfield decoder
 */
struct DaikinSystemState {
  constexpr DaikinSystemState(const uint8_t value = 0U) : raw(value) {}
  constexpr bool locked() const { return (this->raw & 0x01) != 0; }
  constexpr bool active() const { return (this->raw & 0x04) != 0; }
  constexpr bool defrost() const { return (this->raw & 0x08) != 0; }
  constexpr bool multizone_conflict() const { return (this->raw & 0x20) != 0; }
  uint8_t raw{};
};

struct DaikinSettings {
  climate::ClimateMode mode{climate::CLIMATE_MODE_OFF};
  DaikinC10 setpoint{23};
  climate::ClimateSwingMode swing{climate::CLIMATE_SWING_OFF};
  DaikinFanMode fan{DaikinFanMode::Auto};
  climate::ClimatePreset preset{climate::CLIMATE_PRESET_NONE};

  constexpr bool operator==(const DaikinSettings &other) const = default;
};

class DaikinS21 : public PollingComponent {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;

  void set_uarts(uart::UARTComponent *tx, uart::UARTComponent *rx) { this->serial = {tx, rx}; }
  void set_debug_comms(bool set) { this->serial.debug = set; }
  void set_debug_protocol(bool set) { this->debug_protocol = set; }

  // external command action
  void set_climate_settings(const DaikinSettings &settings);
  
  void add_climate_callback(std::function<void(void)> &&callback);
  void add_binary_sensor_callback(std::function<void(DaikinUnitState, DaikinSystemState)> &&callback);

  // value accessors
  bool is_ready() { return this->ready.all(); }
  const DaikinSettings& get_climate_settings() { return this->active; };
  climate::ClimateMode get_climate_mode() { return this->active.mode; }
  climate::ClimateAction get_climate_action() { return this->action_resolved; }
  auto get_setpoint() { return this->active.setpoint.f_degc(); }
  auto get_temp_inside() { return this->temp_inside.f_degc(); }
  auto get_temp_outside() { return this->temp_outside.f_degc(); }
  auto get_temp_coil() { return this->temp_coil.f_degc(); }
  auto get_fan_rpm() { return this->fan_rpm; }
  auto get_swing_vertical_angle() { return this->swing_vertical_angle; }
  auto get_compressor_frequency() { return this->compressor_hz; }
  auto get_humidity() { return this->humidity; }
  auto get_demand() { return this->demand; }

 protected:
  DaikinSerial serial;

  void dump_state();
  void refine_queries();
  void tx_next();
  void parse_ack();
  void handle_nak();

  CallbackManager<void(void)> climate_callback_{};
  CallbackManager<void(DaikinUnitState, DaikinSystemState)> binary_sensor_callback_{};

  enum ReadyCommand : uint8_t {
    ReadyProtocolVersion,
    ReadySensorReadout,
    ReadyCapabilities,
    ReadyBasic,
    ReadyCount, // just for bitset sizing
  };
  std::bitset<ReadyCount> ready{};

  // communication state
  bool is_query_active(std::string_view query);
  bool prune_query(std::string_view query);
  std::vector<std::string_view> queries{};
  std::vector<std::string_view>::iterator current_query{};
  std::string_view tx_command{};  // used when matching responses - backing value must have persistent lifetime across serial state machine runs
  
  // debugging support
  bool debug_protocol{false};
  std::unordered_map<std::string, std::vector<uint8_t>> val_cache{};
  std::vector<std::string_view> nak_queries{};
  uint32_t cycle_time_start_ms{};
  uint32_t cycle_time_ms{};

  // settings
  DaikinSettings active{};
  DaikinSettings pending{ .mode = climate::CLIMATE_MODE_AUTO }; // unsupported sentinel value, see set_climate_settings
  bool activate_climate{false};
  bool activate_swing_mode{false};
  bool activate_preset{false};

  // current values
  climate::ClimateAction action_reported = climate::CLIMATE_ACTION_OFF; // raw readout
  climate::ClimateAction action_resolved = climate::CLIMATE_ACTION_OFF; // corrected at end of scan
  DaikinC10 temp_inside{};
  DaikinC10 temp_target{};
  DaikinC10 temp_outside{};
  DaikinC10 temp_coil{};
  uint16_t fan_rpm{};
  int16_t swing_vertical_angle{};
  uint8_t compressor_hz{};
  uint8_t humidity{50};
  uint8_t demand{};
  DaikinUnitState unit_state{};
  DaikinSystemState system_state{};
  enum Modifier : uint8_t {
    ModifierQuiet,    // outdoor unit fan/compressor limit
    ModifierEcono,    // limits demand for power consumption
    ModifierPowerful, // maximum output (20 minute timeout), mutaully exclusive with quiet and econo
    ModifierComfort,  // fan angle depends on heating/cooling action
    ModifierStreamer, // electron emitter decontamination?
    ModifierSensor,   // "intelligent eye" PIR occupancy setpoint offset
    ModifierLED,      // the sensor LED is on
    ModifierCount,    // just for bitset sizing
  };
  std::bitset<ModifierCount> modifiers{};

  // protocol support
  bool determine_protocol_version();
  struct DetectResponses {
    std::array<uint8_t,4> G8{};
    std::array<uint8_t,4> GC{};
    uint16_t GY00{};
    std::array<uint8_t,4> M{};
    std::array<uint8_t,4> V{};
  } detect_responses;
  struct ProtocolVersion {
    uint8_t major{std::numeric_limits<uint8_t>::max()};
    uint8_t minor{std::numeric_limits<uint8_t>::max()};
  } protocol_version{};
  char G2_model_info{};
  bool support_swing{};
  bool support_horizontal_swing{};
  bool support_humidity{};

  // helpers
  climate::ClimateAction resolve_climate_action();
};

}  // namespace daikin_s21
}  // namespace esphome
