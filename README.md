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

* This code has only been tested on ESP32 pico and ESP32-S3.
* Tested with 4MXL36TVJU outdoor unit and CTXS07LVJU, FTXS12LVJU, FTXS15LVJU indoor units.
* Does not detect nor support powerful or econo modes.
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

### PCB

I've been using the board designed by [revk][1] available [here][3]. Note that
revk's design includes a FET that inverts the logic levels on the ESP's RX pin,
which required using two separate UART devices to get around an ESPHome limit
on having pins inverted differently.

[1]: https://github.com/revk
[2]: https://github.com/revk/ESP32-Faikin
[3]: https://github.com/revk/ESP32-Faikin/tree/main/PCB/Faikin
[4]: https://github.com/joshbenner
[5]: https://github.com/joshbenner/esphome-daikin-s21

## Configuration Example

```yaml
esphome:
  min_version: "2025.7"

logger:
  baud_rate: 0  # Disable UART logger if using UART0 (pins 1,3)

external_components:
  - source: github://asund/esphome-daikin-s21@main
    components: [ daikin_s21 ]

uart:
  - id: s21_uart
    tx_pin: GPIO1
    rx_pin: GPIO3
    baud_rate: 2400

# The base UART communication hub.
daikin_s21:
  tx_uart: s21_uart
  rx_uart: s21_uart

climate:
  - name: My Daikin
    platform: daikin_s21
    # Settings from ESPHome Climate component:
    # visual:
    #   target_temperature: 1
    #   current_temperature: 0.5
    # Settings from DaikinS21Climate:
    supports_humidity: true # If your unit supports humidity, it can be reported in the climate component
    # Optional HA sensor used to alter setpoint.
    room_temperature_sensor: room_temp  # See homeassistant sensor below
    setpoint_interval: 300s # Interval used to adjust the unit's setpoint if the room temperature sensor is specified
    supported_modes:  # off and heat_cool are always supported
      - cool
      - heat
      - dry
      - fan_only

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
    valve:
      name: Valve
    short_cycle:
      name: Short Cycle
    system_defrost:
      name: System Defrost
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

daikin_s21:
  tx_uart: s21_tx
  rx_uart: s21_rx
```
