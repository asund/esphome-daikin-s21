# esphome-daikin-s21

ESPHome component to control Daikin indoor mini-split units using the wired
protocol available over S21 and related ports.

A big thanks to:
* [revk](https://github.com/revk) for work on the fantastic
  [Faikin](https://github.com/revk/ESP32-Faikin) project, which was the primary
  inspiration and guide for building this ESPHome component. In addition, the
  very active and resourceful community that project has fostered that has
  decoded much of the protocol.
* [joshbenner](https://github.com/joshbenner), the original author of this
  integration. His [repository](https://github.com/joshbenner/esphome-daikin-s21)
  would be my preferred place to commit patches to avoid fragmentation, but he
  lacks the time at the moment to manage the project.
* The users of this project who have contributed, tested or offered feedback.

## Features

### Climate
* Setpoint temperature.
* Selectable climate modes OFF, HEAT_COOL, COOL, HEAT, FAN_ONLY and DRY.
* Climate action reporting. See what your unit is trying to do, e.g.
  heating while in HEAT_COOL.
* Fan modes auto, silent and 1-5.
* Swing modes horizontal, vertical, and both.
* Untested support for Powerful and Econo presets ("Boost" and "Eco").
  My unit doesn't support these over the wired protocol.
* Optional humidity reporting.
* Limits for commanded setpoints. Defaults should work fine, but if your
  unit is different they can be overridden.

The standard Daikin control loop has a few deficiencies:
* The setpoint value has 1.0°C granularity.
* The current temperature feedback value has 0.5°C granularity.
* The current temperature sensor is located in the unit, in most cases
  a wall unit placed higher in a room. This reading doesn't always reflect
  the temperature felt by occupants.

Because of these, the climate component implements a separate loop on top of
the Daikin internal one. An external temperature sensor can be used to provide
a more accurate and precise reference and an offset applied to the internal
Daikin setpoint. The internal temperature sensor can also be used for this
functionality to drive the coarse setpoint around the more granular
temperature sensor. The rate at which this secondary loop runs must be
configured with the update interval when using this mode. All of this is
downstream of Daikin's reported temperature precision and relatively broad
control loop hysteresis, so it's far from ideal but can compensate for a
difference in temperature between the unit and your space.

### Sensor
* Inside temperature (usually measured at indoor air handler return)
* Outside temperature (outside exchanger)
* Coil temperature (indoor air handler's coil)
* Fan speed
* Vertical swing angle (directional flap)
* Compressor frequency (outside exchanger)
* Humidity (not supported on all units, can report a consistent 50% if
  not present)
* Unit's demand from outside exchanger

### Binary Sensor

New, extracted from the unit and system state bitfields. Still need to observe
to see how valuable these are. I may remove the redundant ones in the future.

* Powerful
* Defrost
* Active (Actively controlling climate, climate action can tell us this)
* Online (In a climate controlling mode, climate mode can tell us this)
* Refrigerant Valve (shadows active?)
* Short Cycle Lock (3 minute compressor lockout)
* System Defrost (shadows defrost?)
* Multizone settings conflict

## Limitations

**NOTE:** Currently there's a serious issue when using the Arduino framework.
If flashed OTA you may lose communication and require a physical reflashing
(annoying if your board in inside your air handler). Please stick to the
ESP-IDF PlatformIO framework for now (Arduino is an extra shim over the ESP-IDF
SDK anyways). See the framework selection in the configuration example.

* This code has only been tested on ESP32 pico and ESP32-S3.
* Tested with 4MXL36TVJU outdoor unit and CTXS07LVJU, FTXS12LVJU, FTXS15LVJU
  indoor units.
* Powerful and econo modes are untested (no v2 hardware).
* Does not support comfort or presence detection features on some models.
* Does not interact with the indoor units schedules (do that with HA instead).
* Higher protocol versions have limited support due to the equipment available
  to me, though I'm happy to try to work with you.
* Daikin's internal R&D departments must be a bit chaotic. The latest models
  might have inferior command sets to those released years ago.

## Hardware

Please see the Faikin [wiring](https://github.com/revk/ESP32-Faikin/wiki/Wiring)
page for detailed documentation including pinouts, alternate connectors with
images. The below is just a quick reference overview.

**NOTE:** The Daikin connector's Vcc provides >5V, so if you intend to power
your board with this pin, be sure to test its output and regulate voltage
accordingly. Mine supplies 14.5V.

For communications pins, I found mine pulled up internally to 5V by the Daikin
board. I've read some users don't see this, so verify with a multimeter. You may
need to add your own external pullups or make use of a level shifting circuit.
Cheap level shifter modules are available that can make use of the provided 5V
reference to do this if you'd prefer. If you are powering your device
externally you should still connect a ground reference for the communications.

### S21 Port

On my Daikin units, the S21 port has the following pins:

1. Vref (5V) Reference for communications, not to be used for powering your
  device. Sometimes N/C.
2. TX (5V)
3. RX (5V)
4. Vcc (>5V!)
5. GND

The S21 plug is JST `EHR-5` and related header `B5B-EH-A(LF)(SN)`, though the
plug pins are at standard 2.5mm pin header widths. I was able to locate headers
and precrimped wire leads on Aliexpress for a low cost.

### S403 Port

The project has been reported to work on a unit with the S403 connecter as
This has the same communication interface with a little additional
functionality, and most importantly to note, mains voltage exposed on a pin.
Do not connect anything to pin 10. Don't populate it in your plug.

1. Vcc (>5V!)
2. JEM-A OUT (pull down = ON)
3. JEM-A IN (pull down = toggle switch)
4. TX (5V)
5. RX (5V)
6. GND
7. GND
8. N/C
9. N/C
10. ~327 VDC (Dangerous!)

Housing: JST `XAP-10V-1`
Contacts: JST `SXA-001T-P0.6`

### PCB Option 1

joshbenner uses the board designed by revk available [here](https://github.com/revk/ESP32-Faikin/tree/main/PCB/Faikin).
Note that revk's design includes a FET that inverts the logic levels on the
ESP's RX pin, which required using two separate UART devices to get around an
ESPHome limit on having pins inverted differently using the Arduino framework.
This handling is now moved behind the provided split_uart component.

### PCB Option 2

I am using ESP32-S3 mini dev boards and directly wiring communication to the
S21 port. My Daikin unit pulls our TX line up to 5V, so I've configured my pin
as open drain to work with it. The RX line relies on the ESP32's 5V tolerant
GPIO pins. For power I am using a cheap 5V -> 3.3V switching module wired into
Vcc on the dev board.

## Contributing

If you can write C++, great. Even if you're more of a YAML writer and user,
your assistance with this project would be helpful. Here are some possible
ways:

* Report your experince with different Daikin units. Turning on protocol
  debugging and getting the protocol detection output values would be the first
  step if you encounter issues and help me learn more about the output of
  different models.
* Let me know how useful the binary sensor values are and which just shadow
  other sensor values.
* If you have a revk module with an inverting RX pin, let me know if using a
  single UART configuration and inverting the RX line with ESPHome's pin schema
  works with ESP-IDF and (when it's working again) the Arduino framework. i.e.
  Not using the split_uart component. If so, I can remove this custom code and
  simplify configuration.

Please consider documenting characteristics of your unit in the appropriate
place in the Faikin wiki. I don't want to make an inferior copy of this
information if possible. Please don't report issues with this component to them
as we don't share a codebase. Some of the readout values here are not
byteswapped if we don't otherwise use them so unknown static fields may appear
reversed compared to that project. Please verify this when contributing.

See existing issues, open a new one or post in the discussions section with
your findings. Thanks.

## Configuration

On multihead systems some values that come from the outdoor unit will be the
same (accounting for sampling jitter). It's recommended to use ESPHome's
subdevice feature to only configure these sensors on your "primary" ESPHome
instance as a subdevice to keep individual indoor unit representations cleaner.
See the configuration example below. This feature seems designed for the
one-to-many case, whereas this system is more of a many-to-one configuration.
Locally I have two configuration template files, one for indoor only units and
one for a single indoor unit with an outdoor unit subdevice. If I can find a
better solution I'll update the example.

When using an external temperature sensor as a room reference instead of the
internal Daikin value (which may be misleading due to placement of the unit)
it's recommended to change the target_temperature step to 0.5°C to reflect the
granularity of the alternate control loop provided. The default is 1.0°C to
match Daikin's internal granularity.

```yaml
esphome:
  min_version: "2025.8"
  devices:
    - id: daikin_outdoor
      name: "Daikin Compressor"

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
  uart: s21_uart
  # update_interval: 15s  # also supports periodic polling instead of more responsive free run

climate:
  - name: My Daikin
    platform: daikin_s21
    # Settings from ESPHome Climate component:
    # Visual limits and steps can be adjusted to match the granularity of your external sensor:
    # visual:
    #   temperature_step:
    #     target_temperature: 1
    #     current_temperature: 0.5
    # Settings from DaikinS21Climate:
    supported_modes:  # off and heat_cool are always supported
      - cool
      - heat
      - dry
      - fan_only
    supported_presets:  # none is always supported
      - eco
      - boost
    # Optional sensors to use for temperature and humidity references
    # sensor: room_temp  # External, see homeassistant sensor below
    humidity_sensor: daikin_humidity  # Internal, see humidity sensor below
    # humidity_sensor: room_humidity  # External, see homeassistant sensor below
    # or leave unconfigured if unsupported to omit reporting
    # update_interval: 60s # Interval used to adjust the unit's setpoint using finer grained control
    # Daikin supported temperature range setpoints. Defaults should be fine unless your unit differs (see your manual):
    # max_temperature: 32 # maximum setpoint when cool
    # max_heat_temperature: 30  # maximum setpoint when heat or heat_cool
    # min_cool_temperature: 18  # minimum setpoint when cool or heat_cool
    # min_temperature: 10 # minimum setpoint when heat

# Optional additional sensors.
sensor:
  - platform: daikin_s21
    inside_temperature:
      name: Inside Temperature
    outside_temperature:
      name: Outside Temperature
      device_id: daikin_outdoor
    coil_temperature:
      name: Coil Temperature
    fan_speed:
      name: Fan Speed
    swing_vertical_angle:
      name: Swing Vertical Angle
    compressor_frequency:
      name: Compressor Frequency
      device_id: daikin_outdoor
    humidity:
      id: daikin_humidity
      name: Humidity
    demand:
      name: Demand  # 0-15 demand units, use filter to map to %
      filters:
        - multiply: !lambda return 100.0F / 15.0F;
    # Protocol Version 2:
    ir_counter:
      name: IR Counter
    power_consumption:
      name: Power Consumption
  # optional external reference sensors
  - platform: homeassistant
    id: room_temp
    entity_id: sensor.office_temperature
    unit_of_measurement: °F
    accuracy_decimals: 1
  - platform: homeassistant
    id: room_humidity
    entity_id: sensor.office_humidity
    unit_of_measurement: "%"
    accuracy_decimals: 1

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
      device_id: daikin_outdoor
    system_defrost:
      name: System Defrost
      device_id: daikin_outdoor
    multizone_conflict:
      name: Multizone Conflict
      device_id: daikin_outdoor
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
  uart: split_uart_1
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
  uart: s21_uart
```
