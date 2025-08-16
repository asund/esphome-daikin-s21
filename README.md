# esphome-daikin-s21

ESPHome component to control Daikin indoor mini-split units with s21 ports.

Many thanks to the work by [revk][1] on the fantastic [Faikin][2] project, which
was the primary inspiration and guide for building this ESPHome component. In
addition, the very active and resourceful community that project has fostered
that has decoded much of the protocol.

A huge thanks to [joshbenner][4], the original author of this integration. His
[repository][5] would be my preferred place to commit patches to avoid
fragmentation, but he lacks the time at the moment to manage the project.

## Features

Climate:
- Setpoint temperature.
- Selectable climate modes OFF, HEAT_COOL, COOL, HEAT, FAN_ONLY and DRY.
- Independent climate action reporting. See what your unit is trying to do, e.g. heating while in HEAT_COOL.
- Fan modes auto, silent and 1-5.
- Swing modes horizontal, vertical, and both.
- Untested support for Powerful and Econo presets ("Boost" and "Eco").
- Optional humidity reporting.

Sensor:
* Inside temperature (usually measured at indoor air handler return)
* Outside temperature (outside exchanger)
* Coil temperature (indoor air handler's coil)
* Fan speed
* Vertical swing angle (directional flap)
* Compressor frequency (outside exchanger)
* Humidity (not supported on all units, will report a consistent 50% if not present)
* Unit demand from outside exchanger

Binary Sensor:
* New, extracted from the unit and system state bitfields. Still need to observe to see how valuable these are.

On multihead systems the outdoor values will be the same (accounting for sampling jitter). It
could be cleaner to only configure these sensors on your "primary" ESPhome device. ESPHome is
adding support for multiple devices, when this is released I will document how to do this.

## Limitations

**NOTE:** Currently there's a serious issue when using the Arduino framework.
If flashed OTA you may lose communication and require a physical reflashing
(annoying if your board in inside your air handler). Please stick to the ESP-IDF
PlatformIO framework for now (Arduino is an extra shim over the ESP-IDF SDK anyways).
See the framework selection in the configuration example.

* This code has only been tested on ESP32 pico and ESP32-S3.
* Tested with 4MXL36TVJU outdoor unit and CTXS07LVJU, FTXS12LVJU, FTXS15LVJU indoor units.
* Powerful and econo modes are untested (no hardware).
* Does not support comfort or presence detection features on some models.
* Does not interact with the indoor units schedules (do that with HA instead).
* Currently targets Version 0 protocol support due to the equipment available to the author.

## Hardware

### S21 Port

**NOTE:** The Daikin S21 port provides >5V, so if you intend to power your
board on this pin, be sure to test its output and regulate voltage accordingly.

On my Daikin units, the S21 port has the following pins:

* 1 - Unused
* 2 - TX (5V)
* 3 - RX (5V)
* 4 - VCC (>5V!!)
* 5 - GND

The S21 plug is JST `EHR-5` and related header `B5B-EH-A(LF)(SN)`, though the
plug pins are at standard 2.5mm pin header widths.

### PCB Option 1

joshbenner uses the board designed by [revk][1] available [here][3]. Note that
revk's design includes a FET that inverts the logic levels on the ESP's RX pin,
which required using two separate UART devices to get around an ESPHome limit
on having pins inverted differently using the Arduino framework.

### PCB Option 2

I am instead using ESP32-S3 mini dev boards and directly wiring communication to
the S21 port. The Daikin unit pulls our TX line up to 5V, so the pin should be
configured as open drain. Instead of driving the line to 0V, it shorts it to
ground through their external pullup resistor. Your Daikin unit may not have this
pullup and may need to be added externally to your circuit. Alternatively there
are cheap level shifter modules available where conventionally driven outputs
will work. The RX line relies on the ESP32's 5V tolerant GPIO pins. Communication
works reliably for me. For power I am using a cheap 5V -> 3.3V switching module
wired into Vcc on the dev board. Don't forget to wire up ground as well.

[1]: https://github.com/revk
[2]: https://github.com/revk/ESP32-Faikin
[3]: https://github.com/revk/ESP32-Faikin/tree/main/PCB/Faikin
[4]: https://github.com/joshbenner
[5]: https://github.com/joshbenner/esphome-daikin-s21

## Contributing

If you can write C++, great. Even if you're more of a YAML writer and user, your
assistance with this project would be helpful. Here are some possible ways:

* Report your experince with different Daikin units. Turning on protocol debugging
and getting the protocol detection output values would be the first step if you
encounter issues and help me learn more about the output of different models.
* Let me know how useful the binary sensor values are and which just shadow other
sensor values.
* If you have a revk module with an inverting RX pin, let me know if using a single
UART configuration and inverting the RX line with ESPHome's pin schema works with
ESP-IDF and (when it's working again) the Arduino framework. i.e. Not using the
split_uart component. If so, I can remove this custom code and simplify configuration.

See existing issues or open a new one with your findings. Thanks.

## Configuration Example

```yaml
esphome:
  min_version: "2025.7"

esp32:
  framework:
   type: esp-idf

logger:
  baud_rate: 0  # Disable UART logger if using UART0 (pins 1,3)

external_components:
  - source: github://asund/esphome-daikin-s21@main
    components: [ daikin_s21, split_uart ]

uart:
  - id: s21_uart
    tx_pin: GPIO1
    rx_pin: GPIO3
    baud_rate: 2400

# The parent UART communication hub platform.
daikin_s21:
  uart_id: s21_uart
  # update_interval: 15s  # supports periodic polling instead of more responsive free run

climate:
  - name: My Daikin
    platform: daikin_s21
    # Settings from ESPHome Climate component:
    # visual:
    #   target_temperature: 1
    #   current_temperature: 0.5
    # Settings from DaikinS21Climate:
    supported_modes:  # off and heat_cool are always supported
      - cool
      - heat
      - dry
      - fan_only
    supported_presets:  # none is always supported
      - eco
      - boost
    supports_humidity: true # If your unit supports humidity, it can be reported in the climate component
    # Optional HA sensor used to alter setpoint.
    room_temperature_sensor: room_temp  # See homeassistant sensor below
    setpoint_interval: 300s # Interval used to adjust the unit's setpoint if the room temperature sensor is specified

# Optional additional sensors.
sensor:
  - platform: daikin_s21
    inside_temperature:
      name: My Daikin Inside Temperature
    outside_temperature:
      name: My Daikin Outside Temperature
    coil_temperature:
      name: My Daikin Coil Temperature
    fan_speed:
      name: My Daikin Fan Speed
    swing_vertical_angle:
      name: Swing Vertical Angle
    compressor_frequency:
      name: Compressor Frequency
    humidity:
      name: Humidity
    demand:
      name: Demand  # 0-15 demand units, use filter to map to %
      filters:
        - multiply: !lambda return 100.0F / 15.0F;
  - platform: homeassistant
    id: room_temp
    entity_id: sensor.office_temperature
    unit_of_measurement: °F

binary_sensor:
  - platform: daikin_s21
    powerful:
      name: Powerful
    defrost:
      name: Defrost
    online:
      name: Online
    valve:
      name: Valve
    short_cycle:
      name: Short Cycle
    system_defrost:
      name: System Defrost
    multizone_conflict:
      name: Multizone Conflict
```

Here is an example of how daikin_s21 can be used with one inverted UART pin:

```yaml
uart:
  - id: s21_tx
    tx_pin: 26
    baud_rate: 2400
    data_bits: 8
    parity: EVEN
    stop_bits: 2

  - id: s21_rx
    rx_pin:
      number: 27
      inverted: true
    baud_rate: 2400
    data_bits: 8
    parity: EVEN
    stop_bits: 2

split_uart:
  id: split_uart_1
  tx_uart: s21_tx
  rx_uart: s21_rx

daikin_s21:
  uart_id: split_uart_1
```

Here's an example of a single UART using direct wiring (you can leave out the split_uart component import as well):

```yaml
uart:
  - id: s21_uart
    tx_pin:
      number: GPIO43
      mode:
        output: true
        open_drain: true  # daikin pulls up to 5V internally
    rx_pin: GPIO44
    baud_rate: 2400

daikin_s21:
  uart_id: s21_uart
```