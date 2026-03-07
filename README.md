# esphome-daikin-s21

ESPHome component to control Daikin indoor mini-split units using the wired
protocol available over S21 and related ports.

***Upgrade Note***: If your automatic update failed, I may have changed some
config schema. I try not to do this but this project is still evolving. Please
see the changelog below for details.

A big thanks to:
* [revk](https://github.com/revk) for work on the fantastic
  [Faikout](https://github.com/revk/ESP32-Faikout) (née Faikin) project, which
  was the primary inspiration and guide for building this ESPHome component. In
  addition, the very active and resourceful community that project has fostered
  that has decoded much of the protocol.
* [joshbenner](https://github.com/joshbenner), the original author of this
  integration. His [repository](https://github.com/joshbenner/esphome-daikin-s21)
  would be my preferred place to commit patches to avoid fragmentation, but he
  lacks the time at the moment to manage the project.
* [amzaldua](https://github.com/amzaldua) for help with multiple rounds of v3
  protocol control debugging.
* The users of this project who have contributed, tested or offered feedback.

## Recent / Breaking Changes

A short changelog of sorts, I'll keep things here where a user might encounter
breaking or significant changes.

* Added instantaneous unit power sensor.
* Fixed compressor frequency sensor scaling. Please purge your history for this
  sensor, the values will be incorrect.
* Added additional energy consumption sensors for protocol 3.20+. Renamed
  existing "power_consumption" to "energy_indoor" (briefly) to "energy". Sorry.
  Update your YAML.
* Checksum calculation was fixed. There's a faint chance that a command or
  query that was previously NAK'd actually now works.
* Custom Silent fan mode changed to standard Quiet. Custom Automatic was also
  updated to standard Auto to work around a validation issue, which results in
  a nicer icon. Update any automations to specify these new mode settings.
* Configuration schema for the climate component (specifically the unit
  temperature range limits) has changed to organize them by mode as well as
  adding a temperature offset to be applied when commanding the unit. Update
  your configurations to the new schema (see example) if your project now fails
  to compile.

## Features

The S21 platform provides a few ESPhome component types that expose the
functionality of a Daikin unit.

**NOTE:** Daikin's support for depth of unit control varies wildly over thier
product line. Your unit could have a lot of features that are exposed via the
IR remote but offer only basic control over the S21 wired bus. There's not much
this project can do for you if this is the case. As an example, I'm unable to
enable Powerful mode on my own device. ESPHome doesn't yet offer an easy way
for the unit to dynamically enable components based on feature detection, so
your best bet at the moment is to include components you'd like to use and
remove them later if not supported. With newer features I could have more
development to do (I can't test locally), so if something should be working
based on the queries available and the protocol version of your unit don't
hesitate to reach out.

## Control

### Climate

The main control interface. Supported features:

* Setpoint temperature.
* Climate modes OFF, HEAT_COOL, COOL, HEAT, FAN_ONLY and DRY. The list is
  configurable in case your unit doesn't support them all or you otherwise want
  to restrict the ones available.
* Climate action reporting.
* Fan modes auto, quiet and 1-5.
* Swing modes off, horizontal, vertical, and both. These can also be restricted
  to a specified list.
* Optional external temperature and humidity reporting and use in an secondary
  control loop. You can use a sensor placed in your living space to get a
  better reflection of the temperature you feel.
* Optional setpoint mode (HEAT_COOL, COOL, HEAT) configuration:
  * Offset to apply to commanded value. If you find your unit adequately
    conditions your living space, Daikin's control loop will overshoot the
    setpoint. It can also not achieve the desired setpoint based on the space
    and reference sensor placement. e.g. One of my rooms is 1.0C less than the
    target temperature at a steady state so I add a +1.0C offset here.
  * Range limits for values sent to the unit. Defaults should work fine, but if
    your unit is different they can be overridden.

Daikin's modes don't neatly fit into the discrete "preset" category modelled by
ESPHome as they function more like mode modifiers. Switches are provided for
individual control of these modifiers instead of climate presets. The only one
that's exclusive is Powerful, but I felt the interface should be uniform.

The standard Daikin control loop has a few deficiencies:
* The setpoint value has 1.0°C granularity.
* The current temperature feedback value has 0.5°C granularity.
* The current temperature sensor is located in the unit, in most cases
  a wall unit placed higher in a room. This reading doesn't always reflect
  the temperature felt by occupants. It will overshoot the setpoint to try to
  account for this (see your service manual for details).

Because of these, the climate component implements a separate loop on top of
the Daikin internal one. An external temperature sensor can be used to provide
a more accurate and precise reference and an offset applied to the internal
Daikin setpoint. The internal temperature sensor can also be used for this
functionality to drive the coarse setpoint around the more granular
temperature sensor. The rate at which this secondary loop runs can be
configured with the component update interval when using this mode. All of this
is downstream of Daikin's reported temperature precision and relatively broad
control loop hysteresis, so it's far from ideal but can compensate for a
difference in temperature between the unit and your space. Keep in mind the
unit will overshoot, so you may want to configure an offset for the mode based
on a measured reference value.

Climate publish rate was raised as a problem by a user. When external sensors
are used any change in temperature or humidity will cause the updated climate
state to be published over the network. The internal default Daikin sensor can
also flip between values when on the edge between steps. You can reduce this by
configuring an explicit temperature sensor (in the case of the internal sensor)
and then using ESPHome sensor filters in the sensor definitions themselves.
Any of the averaging filters can be used to flatten out a toggling internal
sensor and use of the delta filter can gate updates to coarser, more signficant
changes.

### Select

* LED brightness. v2+ may support this. The value may change when using IR
  remote modes that make use of the PIR sensor. If this control works for you,
  there's no need to enable the Sensor LED or Sensor Mode switches or binary
  sensors.

* Vertical swing setpoint. v2+ may support this. Preset values can be selected
  for the vertical louver, including the standard on and off for the varrying
  setting.

* Humidity setpoint. v2+ may support this. The operation of this isn't well
  understood. There may be protocol sequencing work required to maintain
  control in the desired mode. If you have an "Ururu Sarara" unit and want to
  help, please post your findings in the discussions section. For now the
  setting is sent to the unit when changed in HA and nothing else done.

### Switch

Mode toggle switches for protocol v2+. As a reminder: even if these modes are
supported by your unit, they are not necessarily supported via the S21
interface. In some cases support exists for reading the current value set by
the IR remote. If this is true for you, a parallel set of binary sensors are
offered as a read only indication. Use those instead if that's the case.
There's no point in having both the switch and corresponding binary sensor
active at the same time.

* Powerful Mode
* Comfort Mode
* Quiet Mode
* Streamer Mode
* Sensor LED (unverified)
* Sensor Mode (unverified)
* Econo Mode

In the Faikout code some units appear to select the brightness of the unit LEDs
via a combination of the motion sensor and LED bits. Let me know if this is the
case for you and I can try to implement better control.

### Number

* Demand Control (v2+, unverified). Select from 30%-100% of power output. As a
  reference point, Econo limits this to around 70%.

## Feedback

### Sensor

Sensors in ESPHome are geared more towards a periodic ADC value that can be
filtered as necessary. The values read from the Daikin unit are pre-processed
and won't normally require averaging filters and the like. The sensor component
supports a configurable update interval that will publish the current values
at a chosen rate. To make this reporting more responsive, the user can set this
value to zero and every update from the unit, interesting or not, will be
published. To reduce spam when doing this, please configure a delta filter on
your sensors and only changes will be published over the network. See the
example configuration.

* Inside temperature, usually measured at indoor air handler return.
* Outside temperature from the exchanger.
* Coil temperature of indoor air handler. These will be very similar for units
  sharing an outdoor compressor. You may only want to enable this on a single
  indoor unit to reduce UI clutter.
* Target temperature (internal setpoint, modified by special modes). This is
  not the user setpoint and not that interesting to me but may be useful for
  your automations.
* Fan speed of inside blower.
* Vertical swing angle of louver. This uses Daikin's reference frame.
* Compressor frequency of the outside exchanger.
* Humidity. Not supported on all units, can report a consistent 50% or 0% if
  not present.
* Unit's demand from outside exchanger.

v2+ protocol units may also support:

* IR counter that increments when the remote is used.
* Energy consumption of all indoor units in kWh.
* Outdoor unit capacity in indoor units.

v3.20+ protocol units may also support:

* Instantaneous unit power consumption in W. If you have a multihead system,
  sampling jitter may result in incorrect results when summed. Use as a rough
  indicator only and prefer energy consumption for your dashboard.
* Energy consumption in cooling and heating modes in kWh, including the outside
  unit's use, for this inside unit.

### Binary Sensor

Mode indicators that mirror the switches above for units and modes that don't
support setting via S21. There's no point in having both the switch and
corresponding binary sensor active at he same time.

* Powerful Mode. Can pull from unit state bitfield when the primary query is
  unsupported. See the note about the bitfield and support for it below.
* Comfort Mode
* Quiet Mode
* Streamer Mode
* Sensor LED (unverified)
* Sensor Mode (unverified)
* Econo Mode

On some units a memory location was identified that contains unit and system
state informtation. We still need to observe to see how valuable these are. I
may remove the redundant ones in the future. Not all units support these. If
you see nonsense output then we are likely reading random memory. Your debug
logs can let me blacklist your model and let others avoid this.

* Defrost
* Active (Actively controlling climate, climate action can tell us this)
* Online (In a climate controlling mode, climate mode can tell us this)
* Refrigerant Valve (shadows active?)
* Short Cycle Lock (3 minute compressor lockout when changing operation)
* System Defrost (shadows defrost?)
* Multizone settings conflict

Additional binary sensors:

* Serial error. Always OK if working, this is mainly for developer use.

### Text Sensor

As always, actual support may vary.

* Model Name (v3.40+, untested)
* Software Revision (v3+)
* Software Version (v2+)

If you've read the Faikout wiki you'll see many more queries available than
what this project supports. I've added a text_sensor component to read these
raw values out for debugging and protocol decoding use, without having to add
a dedicated sensor. Normally you wouldn't need to use this, but if a value
looks interesting you can see the value change over time in Home Assistant.
I'd prefer if we use this just to confirm a query works and then add proper
support, rather than trying to interpret the string via HA. Open an issue with
details if you want a sensor or control added.

## Limitations

**NOTE:** Currently there's a serious issue when using the Arduino framework.
If flashed OTA you may lose communication and require a physical reflashing
(annoying if your board in inside your air handler). Please stick to the
ESP-IDF PlatformIO framework for now (Arduino is an extra shim over the ESP-IDF
SDK anyways). See the framework selection in the configuration example. As of
this writing, independent UART pin inversion control also wasn't possible.

* Aforementioned S21 control limitations. Your unit may support a mode but
  support for controlling over S21 may not be there. See your model's
  documentation for supported wired remotes and their feature sets to confirm.
* This code has only been tested on ESP32 pico and ESP32-S3. It compiles for
  ESP8266.
* Tested with my 4MXL36TVJU outdoor unit and CTXS07LVJU, FTXS12LVJU, FTXS15LVJU
  indoor units. These are protocol v0.
* Presence detection is limited to mode configuration. I haven't found a way to
  detect the state of the sensor and expose a motion detector sensor. If you're
  handy with electronics, the PIR sensor output can be wired to a spare GPIO
  and used that way.
* Does not interact with the indoor units schedules. Do that with HA instead.
* Higher protocol versions have limited support for their features due to the
  equipment available to me, though I'm happy to try to work with you.
* Daikin's internal R&D departments must be a bit chaotic. The latest models
  might have inferior command sets to those released years ago.

## Hardware

Please see the Faikout [wiring](https://github.com/revk/ESP32-Faikout/wiki/Wiring)
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
2. TX (5V) Daikin Tx, ESPHome Rx
3. RX (5V) Daikin Rx, ESPHome Tx
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
4. TX (5V) Daikin Tx, ESPHome Rx
5. RX (5V) Daikin Rx, ESPHome Tx
6. GND
7. GND
8. N/C
9. N/C
10. ~327 VDC (Dangerous!)

Housing: JST `XAP-10V-1`
Contacts: JST `SXA-001T-P0.6`

### PCB Option 1

joshbenner uses the board designed by revk available [here](https://github.com/revk/ESP32-Faikout/tree/main/PCB/Faikout).
Note that revk's design includes a FET that inverts the logic levels on the
ESP's RX pin, and on newer revisions the TX pin as well. When interfacing
through a FET the RX line should be configured with a pullup. This handling
should be supported with the ESP-IDF framework and regular ESPHome invert pin
schema, see the example configuration.

### PCB Option 2

I am using ESP32-S3 mini dev boards and directly wiring communication to the
S21 port. My Daikin unit pulls the TX line up to 5V, so I've configured my pin
as open drain to work with it. The RX line relies on the ESP32's 5V tolerant
GPIO pins. For power I am using a cheap 5V -> 3.3V switching module wired into
Vcc on the dev board.

## Contributing

If you can write C++, great. Even if you're more of a YAML writer or user,
your assistance with this project would be helpful. Here are some possible
ways:

* Report your experince with different Daikin units. Turning on protocol
  debugging and getting the protocol and model detection output values would be
  the first step if you encounter issues and help me learn more about the
  output of different models. The 60s debug dump in the logs is valuable for
  documentation and support.
* Let me know how useful the binary sensor values are and which just shadow
  other sensor values.
* Tell me if there are any problems with the control code for your units. The
  supported features vary wildly so you may find control or even readout of
  features or sensors doesn't work for you. This is normal, especially for my
  v0 units. It's recommended that once you figure out what doesn't work for you
  that you just disable those controls or sensors. However if you find that
  your unit does support something via enabling protocol debugging or testing
  with debug queries then I would like to add support for it.
* Please consider documenting characteristics of your unit in the appropriate
  place in the Faikout wiki. I don't want to make an inferior copy of this
  information if possible. Please don't report issues with this component to
  them as we don't share a codebase. Some of the readout values here are not
  byteswapped if we don't otherwise use them so unknown static fields may
  appear reversed compared to that project. Please verify this when
  contributing.
* Check the discussions section on Github for things that may not be in this
  readme yet.

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
better solution I'll update the example. You can also use this feature to hide
away a bunch of stats in a subdevice and leave the primary interface clean.

When using an external temperature sensor as a room reference instead of the
internal Daikin value it's recommended to change the target_temperature step
to 0.5°C to reflect the granularity of the alternate control loop provided.
The default is 1.0°C to match Daikin's internal granularity.

```yaml
esphome:
  min_version: "2026.1"
  devices:
    - id: daikin_outdoor
      name: "Daikin Compressor"

esp32:
  framework:
   type: esp-idf

logger:
  baud_rate: 0  # Disable UART logger if using UART0 (pins 1,3)

preferences:
  flash_write_interval: 72h

external_components:
  - source: github://asund/esphome-daikin-s21@main
    components: [ daikin_s21 ]

uart:
  - id: s21_uart
    tx_pin: GPIO1
    rx_pin: GPIO3
    baud_rate: 2400
    data_bits: 8
    parity: EVEN
    stop_bits: 2

# The parent UART communication hub platform.
daikin_s21:
  uart: s21_uart
  # debug_protocol: true  # please enable when reporting logs!
  # update_interval: 5s  # also supports periodic polling instead of more responsive free run

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
    # supported_modes:  # optional, restricts available modes. off is always supported.
    #   - heat_cool
    #   - cool
    #   - heat
    #   - dry
    #   - fan_only
    # supported_swing_modes:  # optional, restricts available swing modes. off is always supported.
    #   - horizontal
    #   - vertical
    #   - both
    # Optional sensors to use for temperature and humidity references
    sensor: daikin_temperature  # Internal, see indoor temperature sensor below
    # sensor: room_temp  # External, see homeassistant sensor below
    humidity_sensor: daikin_humidity  # Internal, see humidity sensor below
    # humidity_sensor: room_humidity  # External, see homeassistant sensor below
    # or leave unconfigured if unsupported to omit reporting
    # update_interval: 60s # Interval used to adjust the unit's setpoint using reference sensor
    # Mode specific temperature parameters:
    # heat_cool_mode:
    #   offset: 0           # offset to apply to unit setpoint in this mode
    #   min_temperature: 18 # minimum temperature to be sent to the unit in this mode, default probably fine
    #   max_temperature: 30 # maximum temperature to be sent to the unit in this mode, default probably fine
    # cool_mode:
    #   offset: 0
    #   min_temperature: 18
    #   max_temperature: 32
    # heat_mode:
    #   offset: 0
    #   min_temperature: 10
    #   max_temperature: 30

select:
  - platform: daikin_s21
    brightness:
      name: LED Brightness
    humidity:
      name: Humidity
    vertical_swing:
      name: Vertical Swing

sensor:
  - platform: daikin_s21
    update_interval: 0s
    # setpoint_temperature: # Unit setpoint. provided for insight on climate adjustment but probably not useful for users.
    #   name: Setpoint Temperature
    #   filters:
    #     - delta: 0.0
    inside_temperature:
      id: daikin_temperature
      name: Inside Temperature
      filters:
        - delta: 0.0
    # target_temperature: # unit control loop setpoint, taking mode modifiers into account. probably only useful when debugging.
    #   name: Target Temperature
    #   filters:
    #     - delta: 0.0
    outside_temperature:
      name: Outside Temperature
      device_id: daikin_outdoor
      filters:
        - delta: 0.0
    coil_temperature:
      name: Coil Temperature
      # device_id: daikin_outdoor # values will be roughly the same on multiple indoor units, though they are independent measurements.
      filters:
        - delta: 0.0
    fan_speed:
      name: Fan Speed
      filters:
        - delta: 0.0
    swing_vertical_angle:
      name: Swing Vertical Angle
      filters:
        - delta: 0.0
    compressor_frequency:
      name: Compressor Frequency
      device_id: daikin_outdoor
      # unit_of_measurement: "RPM"  # if you would prefer RPM, uncomment the following
      # device_class: ""  # override default "frequency", RPM is not a measurement of frequency in HA
      filters:
        # - multiply: 60.0
        - delta: 0.0
    humidity:
      id: daikin_humidity
      name: Humidity
      filters:
        - delta: 0.0
    demand:
      name: Demand  # 0-15 demand units by default, use filter to map to 100%
      filters:
        - multiply: !lambda return 100.0F / 15.0F;
        - delta: 0.0
    # Protocol Version 2:
    ir_counter:
      name: IR Counter
      filters:
        - delta: 0.0
    energy:
      name: Energy Consumption
      device_id: daikin_outdoor
      filters:
        - delta: 0.0
    outdoor_capacity:
      name: Outdoor Capacity
      device_id: daikin_outdoor
      filters:
        - delta: 0.0
    # Protocol Version 3.20:
    power:
      name: Unit Power
      filters:
        - delta: 0.0
    energy_cooling:
      name: Energy Consumption Cooling
      filters:
        - delta: 0.0
    energy_heating:
      name: Energy Consumption Heating
      filters:
        - delta: 0.0
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

switch:
  - platform: daikin_s21
    # F6 and F7 mode states, not always supported
    powerful:
      name: Powerful Mode Switch
    comfort:
      name: Comfort Mode Switch
    quiet:
      name: Quiet Mode Switch
    streamer:
      name: Streamer Mode Switch
    led:
      name: Sensor LED Switch
    motion:
      name: Sensor Mode Switch
    econo:
      name: Econo Mode Switch

binary_sensor:
  - platform: daikin_s21
    # F6 and F7 mode states, not always supported
    powerful:
      name: Powerful Mode Sensor
    comfort:
      name: Comfort Mode Sensor
    quiet:
      name: Quiet Mode Sensor
    streamer:
      name: Streamer Mode Sensor
    led:
      name: Sensor LED Sensor
    motion:
      name: Sensor Mode Sensor
    econo:
      name: Econo Mode Sensor
    # unit and system state bitfield derived, not always supported or reliable:
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
    # serial_error:
    #   name: Serial Error

text_sensor:
  - platform: daikin_s21
    model:
      name: Model
    software_revision:
      name: Software Revision
    software_version:
      name: Software Version
    # raw case-sensitive query monitoring for debugging
    # queries:
    #   - RK

number:
  - platform: daikin_s21
    demand:
      name: Demand
```

Here is an example of how daikin_s21 can be used with inverted UART pins
and an RX pullup (newer Faikout PCB rev):

```yaml
uart:
  - id: s21_uart
    tx_pin:
      number: GPIO48
      inverted: true
    rx_pin:
      number: GPIO34
      inverted: true
      mode:
        input: true
        pullup: true
    baud_rate: 2400
    data_bits: 8
    parity: EVEN
    stop_bits: 2

daikin_s21:
  uart: s21_uart
```

Here's an example of ESP32 dev board using direct wiring:

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
    data_bits: 8
    parity: EVEN
    stop_bits: 2

daikin_s21:
  uart: s21_uart
```
