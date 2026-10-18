# esphome-daikin-s21

ESPHome components to control Daikin indoor mini-split units using the wired protocol available over S21 and related ports.

***Upgrade Note***: If your automatic update failed, I may have changed some config schema. I try not to do this but this project is still evolving. Please see the changelog below for a summary and the configuration reference for details and examples.

A big thanks to:
* [revk](https://codeberg.org/RevK) for work on the fantastic [Faikout](https://codeberg.org/RevK/ESP32-Faikout) (née Faikin) project, which was the primary inspiration and guide for building this ESPHome component. In addition, the very active and resourceful community that project has fostered that has decoded much of the protocol.
* [joshbenner](https://github.com/joshbenner), the original author of this integration. His [repository](https://github.com/joshbenner/esphome-daikin-s21) would be my preferred place to commit patches to avoid fragmentation, but he lacks the time at the moment to manage the project.
* [amzaldua](https://github.com/amzaldua) for help with multiple rounds of v3 protocol control debugging.
* The users of this project who have contributed, tested or offered feedback.

## Recent / Breaking Changes

A short changelog of sorts, I'll keep things here where a user might encounter breaking or significant changes, including configuration updates.

* Added `angle_scale` to the `daikin_s21` platform. If your vertical swing angles are in the 128-400 degree range you can specify a scale of 10 here.
* Converted DaikinS21 to inherit from the ESPHome UARTDevice component. The configuration key `uart` should be changed to `uart_id`. A deprecation warning has been added. `uart_id` is now optional if you have a single UART configured in your device. UART settings are now checked at the configuration stage.
* Added Ururu/Sarara humidity control modes via climate presets. The presets need to be enabled in your config to use them if you have one of these units.
* Added vertical angle setpoint emulation for units that lack the setpoint command. Horizontal swing setting is now preserved while using this control for both methods. The discrete step should be preserved in the UI afterwards if vertical swing isn't changed.
* Climate action can now report defrosting instead of a short cooling period. If this is always on while heating your unit doesn't support the state reporting bits and can be blacklisted. Open an issue.
* Added enforcement for a non-zero update interval at the configuration stage. If you have an `update_interval` of 0s specified in any polling components (s21, climate, sensor) please change this to `never` to indicate the intent is to free run.
* Modified humidity control to add support for humid heating. The setting is now also applied when switching to heating or cooling modes and only updated in those modes so the setting is preserved over mode changes.
* Updated the state class of energy sensors to total_increasing so they integrate into the Home Assistant Energy dashboard. The values still track the unit's lifetime totals; they are seeded from the Daikin's own counter and accumulated monotonically on-device, so the small backward dips the unit reports on power loss are absorbed rather than published as a decrease, and energy consumed after a dip is still counted. The totals are persisted to flash to survive reboots and power loss. If flash wear is a concern (writes are already coalesced and rate-limited by the 0.1 kWh resolution), raise the `flash_write_interval` on the esp32 platform to reduce write frequency.
* Binary sensor changes: Removed `valve`, use unit `active` instead for the same info. Renamed `multizone_conflict` to `multizone_online`, inverting logic and changing the device class. Added `system_online` to track all units on a compressor. Compressor `short_cycle` device class of lock removed; this was showing up in the HA Security dashboard. History will be inverted and should be purged if it matters to you.
* ***Important***: Updated configuration schema of climate component. The previous `update_interval` is moved to `offset_interval`. This is the period where the external reference temperature sensor offset is applied to the internal control loop. `update_interval` now means the period at which sensor updates are published to the frontend. This addresses a user request to limit the publishing rate when the system uses noisy or precise sensors with a high update rate that aren't actually that interesting. Changes to other values (action, fan mode, etc.) will still be published immediately. It can be omitted for free run operation if desired.

## Limitations

The `daikin_s21` platform provides different ESPhome component types that expose the functionality of a Daikin unit's S21 protocol bus. I've done much of what I want to, but there's always room for more features or extending protocol support. **The first thing to note:** Daikin's support for depth of unit control varies wildly over thier product line. There are very many features supported here that I can't use with my unit because it's older and doesn't support them.

* Your unit could have a lot of features that are exposed via the IR remote but offer only basic control over the S21 wired bus. Very common.
* Your unit could have a few hidden features not described in the manual but are a part of some common Daikin firmware and available over S21. More rare.
* Your unit could be newer or have protocol v3.40 support and still have an inferior S21 feature set to an older, cheaper unit.
* If your unit doesn't support an underlying command there's not much this project can do for you. As an example, I'm unable to enable Powerful mode on my own device. Daikin wired remotes for my unit don't have this option either, so everyone involved with reverse engineering efforts has concluded it's just not possible.
* Your unit could support control of an unsupported feature over S21, but this project doesn't yet. This is the exciting case for me as I would like the project as featureful as possible but it will likely require reverse engineering efforts to discover how to do it. See the [Contributing](#contributing) section.

The Faikout S21 Protocol [wiki page](https://codeberg.org/RevK/ESP32-Faikout/wiki/S21-Protocol) is the best source of information about the protocol and queries being used. If regular queries used to poll the state of the unit appear under the NAK list, they are unsupported. If you see NAK messages when you issue commands, they are likely unsupported, but it's also possible they're being rejected for invalid formatting on my part. See the wiki page to see if command support might exist and your model's documentation for supported wired remotes and their feature sets to confirm if support should exist.

* Temperature values are in Celcius. Daikin uses it, I use it, a lot of the world uses it. If you don't there's probably an easy way to filter and override it.
* Aforementioned S21 protocol support limitations.
* I only test with my ESP32-S3 hardware. It compiles for ESP8266 and has been reported to work, but only with the hardware UART. As the S21 interface only requires a slow UART, it should work on most hardware platforms.
* I only test with my 4MXL36TVJU outdoor unit and CTXS07LVJU, FTXS12LVJU, FTXS15LVJU indoor units. These are protocol v0, so anything more featureful requires your feedback and help.
* Presence detection is limited to mode configuration and even that support is spotty. No one has found a way to detect the state of the sensor and expose a motion detector sensor. If you're handy with electronics, the active low PIR sensor output can be wired to a spare GPIO and used that way.
* While support has been decoded, this project does not interact with the indoor unit schedules. Similar functionality should be easy enough to add using Home Assistant or ESPHome Time based actions if required.

Lastly, some Daikin units use other protocols this project doesn't support. Faikout is the most capable project, with support for CN_WIRED and work to add support for the S_DGC bus. There's also a CN_WIRED ESPHome project out there.

## Hardware

Please see the Faikout [wiring](https://codeberg.org/RevK/ESP32-Faikout/wiki/Wiring) page for detailed documentation including pinouts, alternate connectors with images. Below is just a quick reference overview.

**NOTE:** The Daikin connector's Vcc provides >5V, so if you intend to power your board with this pin, be sure to test its output and regulate voltage accordingly. Mine supplies 14.5V.

For communications pins, I found mine pulled up internally to 5V by the Daikin board. I've read some users don't see this, so verify with a multimeter. You may need to add your own external pullups or make use of a level shifting circuit. Cheap level shifter modules are available that can make use of the provided 5V reference to do this if you'd prefer. If you are powering your device externally you should still connect a ground reference for the communications. A word of warning about very cheap level shifters: a user reported intermittent communication, probably due to bad FETs. The ESP chip is unofficially 5V tolerant, but a passive circuit with a voltage divider on the ESP Rx side should also be possible. Said user is using 1K/2K to bring 5V down closer to 3.3V logic levels.

The ESP8266 platform is quite limited. Since the cost of boards are in the same ballpark, please use an ESP32 (or at least something with a free hardware UART). ESP8266 boards often have a USB-Serial chip wired to their only hardware UART. This chip can interfere with the Rx pin on the microcontroller, driving it to levels that override the open drain output mode used by the Daikin unit, making it unable to pull the Rx line to ground and signalling responses to the processor. If you want to use one anyways, a user has reported lifting the pads on this chip allows the single hardware UART to be used for communication with the unit. ESPHome's software UART implementation isn't viable for this application as it uses delays inside an interrupt routine. This might be accpetable at higher baud rates, but at 1200 baud the delays become too long and the system will watchdog out and reset. Just get an ESP32 board.

### Flash Usage

ESPHome preferences (and the associated flash memory wear) are used in the following ways to save information over power cycles:

* Temperature setpoints in the three main modes. Changing operation mode will restore the last used setpoint for that mode by default.
* Energy sensor totals for coherent reporting as `total_increasing` state class. Daikin preserves these internally but at a lower resolution, so to prevent a small amount of rollback if power is lost at the wrong time the current high resolution value is preserved, making your energy dashboard in Home Assistant glitch free.

If these are a concern on your platform, raise the `flash_write_interval` to reduce write frequency at the cost of losing the current state if power is lost. The flash on an ESP32 device is rated for 100000 write cycles and ESPHome provides wear leveling so this isn't thought to be a problem.

### S21 Port

On my Daikin units, the S21 port has the following pins:

1. Vref (5V) Reference for communications, not to be used for powering your device. Sometimes N/C.
2. TX (5V) Daikin Tx, ESPHome Rx
3. RX (5V) Daikin Rx, ESPHome Tx
4. Vcc (>5V!)
5. GND

The S21 plug is JST `EHR-5` and related header `B5B-EH-A(LF)(SN)`, though the plug pins are at standard 2.5mm pin header widths. I was able to locate headers and precrimped wire leads on Aliexpress for a low cost.

### S403 Port

The project has been reported to work on a unit with the S403 connecter as this has the same communication interface with a little additional functionality, and most importantly to note, mains voltage exposed on a pin. Do not connect anything to pin 10. Don't populate it in your cable.

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

### Circuit Option 1

joshbenner uses the board designed by revk available [here](https://codeberg.org/RevK/ESP32-Faikout/src/branch/main/PCB/Faikout). Note that revk's design includes a FET that inverts the logic levels on the ESP's RX pin, and on newer revisions the TX pin as well. When interfacing through a FET the RX line should be configured with a pullup. This handling should be supported with the ESP-IDF framework and regular ESPHome invert pin schema, see the example configuration.

### Ciruit Option 2

I am using ESP32-S3 mini dev boards and directly wiring communication to the S21 port, relying on the ESP32's GPIO pins being unofficially 5V tolerant. My Daikin unit pulls the TX line up to 5V, so I've configured my pin as open drain to let it float and pull it down when transmitting. For power I am using a cheap switching regulator module to take 14.5V from the Daikin unit down to 5V wired into VBUS on the dev board and the on board regulator takes it to 3.3V.

## Configuration

### DaikinS21 UART Platform Hub

At this time only a single `daikin_s21` platform can be configured on the device. It binds to a UART bus that should be set to 2400 8E2 for S21 communication. The ESPHome build process will validate the UART settings at configure time.

```yaml
# Example configuration entry
daikin_s21:
  uart_id: s21_uart
  update_interval: never
  angle_scale: 1
  debug_protocol: true
```

* **uart_id** (*Optional*, UART ID): The UART component to bind to. Required if you have more than one UART configured on your device. Defaults to the single UART on your device.
* **update_interval** (*Optional*, Time): Communication polling rate, the rate at which a complete set of registers will be polled from the unit. Set to `never` for free run mode. Defaults to `never`.
* **angle_scale** (*Optional*, int, either 1 or 10): Changes the scale factor for vertical angle readout. Some newer units report more granular angles and there's no identified way to automatically detect this. Set to `10` if your angles appear in the hundreds of degrees. Defaults to `1`.
* **debug_comms** (*Optional*, boolean): Enables low level communications logging. Transmitted commands are logged. A lot of clutter, useful for development debugging. Defaults to `false`.
* **debug_protocol** (*Optional*, boolean): Enables additional logging in the 1 minute state dump. Very useful when tracking down bugs or adding features for users, so please enable and include the dump if relevant to your issue. Enables command logging for developers. Defaults to `false`.

### Binary Sensor

A collection of read-only binary indicators. There are two main classes of binary sensor:
1. Mode indicators paired with a parallel set of switches. Some units don't support control of this functionality and can only report feedback from IR remote. Parity between these binary sensors and switches is provided to the user can choose their writablility if they're supported. There's no point in having both the switch and corresponding binary sensor active at the same time.
2. Unit and system state indicators. On some units a memory location was identified that contains unit and system state informtation. We still need to observe to see how valuable some of these are. I may remove the redundant ones in the future. Not all units support these. If you see nonsense output (e.g. your unit toggling defrost on every 10 seconds while not defrosting) then we are likely reading random memory. Your debug logs can let me blacklist your model and let others avoid this. Some sensors refer to values reported by the outdoor unit to the indoor unit and are a good candidate for an "outdoor" subdevice to collect shared readings from a single indoor unit.

These components supports other options inherited from the ESPHome [Binary Sensor Component](https://esphome.io/components/binary_sensor/).

```yaml
# Example configuration entry
binary_sensor:
  - platform: daikin_s21
    comfort:
      name: Comfort Mode Sensor
    defrost:
      name: Defrost
    econo:
      name: Econo Mode Sensor
    led:
      name: Sensor LED Sensor
    motion:
      name: Sensor Mode Sensor
    multizone_online:
      name: Multizone Online
    online:
      name: Online
    powerful:
      name: Powerful Mode Sensor
    quiet:
      name: Quiet Mode Sensor
    short_cycle:
      name: Short Cycle
      device_id: daikin_outdoor
    streamer:
      name: Streamer Mode Sensor
    system_defrost:
      name: System Defrost
      device_id: daikin_outdoor
    system_online:
      name: System Online
      device_id: daikin_outdoor
```

* **active** (*Optional*, Binary Sensor): Indicates the unit is actively controlling climate, as read out from the `Rg` or `RzB2` queries. The specific query is detected on startup. Linked with the climate component's action value.
* **comfort** (*Optional*, Binary Sensor): Indicates Comfort mode is active, as read out from the `F6` query. Supported on some v2+ units.
* **defrost** (*Optional*, Binary Sensor): Indicates the outdoor unit is in a defrost cycle, as read out from the `RzB2` query. Linked with the climate component's `DEFROSTING` action value. Shared on all indoor units.
* **econo** (*Optional*, Binary Sensor): Indicates Econo mode is active, as read out from the `F7` query. Supported on some v2+ units.
* **led** (*Optional*, Binary Sensor): Indicates PIR sensor ("Intelligent Eye") LED is active, as read out from the `F6` query. Mutually exclusive with the LED brightness select functionality. Supported on some v2+ units.
* **motion** (*Optional*, Binary Sensor): Indicates PIR sensor ("Intelligent Eye") is active, as read out from the `F6` query. Mutually exclusive with the LED brightness select functionality. Supported on some v2+ units.
* **multizone_online** (*Optional*, Binary Sensor): Indicates other indoor units are online and in compressor dependent mode, as read out from the `RzC3` query.
* **online** (*Optional*, Binary Sensor): Indicates the unit is not off, as read out from the `RzB2` query. Linked with the climate component's mode value.
* **powerful** (*Optional*, Binary Sensor): Indicates Powerful mode is active, as read out from the `F6` or `RzB2` queries. The specific query is detected on startup.
* **quiet** (*Optional*, Binary Sensor): Indicates Quiet mode is active, as read out from the `F6` query. This mode limits the outdoor compressor operation to reduce noise, it's not the lowest indoor fan speed. Supported on some v2+ units.
* **serial_error** (*Optional*, Binary Sensor): Indicates there's a serial interface error, as read out from the `F4` query. This is always OK when working, you can ignore this if you're not a developer.
* **short_cycle** (*Optional*, Binary Sensor): Indicates the short cycle protection period is active, a 120-180s delay when operational modes are changed, as read out from the `RzC3` query. Shared on all indoor units.
* **streamer** (*Optional*, Binary Sensor): Indicates Streamer mode is active, as read out from the `F6` query. Supported on some v2+ units.
* **system_defrost** (*Optional*, Binary Sensor): Indicates the outdoor unit is in a defrost cycle, as read out from the `RzC3` query. Appears to shadow the unit defrost indicator, but I have to wait for winter to confirm. Shared on all indoor units.
* **system_online** (*Optional*, Binary Sensor): Indicates any indoor unit is online, as read out from the `RzB2` and `RzC3` queries. Shared on all indoor units.

## Climate

The primary control interface. More than just a wrapper around direct control of the Daikin unit because the Daikin control loop has a few deficiencies:

* The setpoint value has 1.0°C (sometimes 0.5°C) granularity.
* The temperature feedback value has 0.5°C granularity.
* The temperature sensor is located in the unit, in most cases a wall unit placed higher in a room. This reading doesn't always reflect the temperature felt by occupants. It is programmed to overshoot the setpoint to try to account for this (see your service manual for details).

Because of these, the climate component implements a separate loop on top of the Daikin internal one. An external temperature sensor can be used to provide a more accurate and precise reference and a dynamic offset applied to the internal Daikin setpoint. The internal temperature sensor can also be used for this functionality to provide a variable offset to drive the coarse setpoint around the more granular temperature sensor. Static offsets can also be configured to reduce the overshoot that will occur even with an accurate reference sensor. All of this is downstream of Daikin's reported temperature precision and relatively broad control loop hysteresis, so it's far from ideal but can help compensate for a difference in temperature between the unit and your space.

Supported standard Climate features:
* Setpoint temperature control.
* Climate mode control.
* Fan mode control.
* Swing mode control.
* Presets for "Ururu Sarara" humidity control. Other Daikin modes don't neatly fit into the discrete "preset" category modelled by ESPHome as they function more like mode modifiers and aren't mutually exclusive. Switches are provided for individual control of these modifiers instead of climate presets.
* Climate action, current temperature and humidity reporting.

This component supports other options inherited from the ESPHome [Climate Component](https://esphome.io/components/climate/).

```yaml
# Example configuration entry
climate:
  - name: My Daikin
    platform: daikin_s21
    update_interval: never
    offset_interval: 5min
    setpoint_dither: true
    preset: true
    sensor: daikin_temperature
    humidity_sensor: daikin_humidity
    supported_modes:
      - heat_cool
      - cool
      - heat
      - dry
      - fan_only
    supported_swing_modes:
      - horizontal
      - vertical
      - both
    heat_cool_mode:
      offset: 0
      min_temperature: 18
      max_temperature: 30
    cool_mode:
      offset: 0
      min_temperature: 18
      max_temperature: 32
    heat_mode:
      offset: 0
      min_temperature: 10
      max_temperature: 30
```

* **update_interval** (*Optional*, Time): Sensor publishing rate, the current temperature and humidity values. When integrating external high precision sensors or when the current temperature causes a measurement to oscillate between two steps the system to make many largely uninteresting updates to Home Assistant. This setting can be used to limit publishing rates to an interval. As these values are derived from sensors, filters on those sensors can also be used to control the amount and signifigance of updates. Set to `never` for free run mode. Defaults to `never`.
* **offset_interval** (*Optional*, Time): Dynamic offset recalculation rate, where the difference between the reference sensor and the internal sensor is recalculated and integrated into the secondary control loop. Set to `never` for free run mode. Defaults to `5mins`.
* **setpoint_dither** (Optional, boolean): Round calculated setpoint in the direction of change when snapping to a setpoint step to try to oscillate the commanded setpoint over the ideal setpoint. Disabling this simply truncates the extra precision and consistently rounds to the nearest step. Defaults to `true`.
* **preset** (Optional, boolean): Enable preset control of "Ururu Sarara" special humidity modes on models that support them. Due to the underlying protocol format only "Ururu" Humidify is uniquely available using this control, "Sarara" Dry is linked to the regular climate mode Dry. These are humidity first modes that don't attempt to reach a target setpoint. The value is only used and updated in relevant climate modes. Use the separate `humidity` select component for the temperature subordinate Dry Cooling and Humid Heating modes. If your unit doesn't support it you can be prevented switching into heating mode if Ururu Humidify is active as the command will fail. Disable the preset first or better yet, don't enable presets at all. Defaults to `false`.
* **sensor** (*Optional*, Sensor ID): Temperature sensor to use as a reference and report as the current temperature of the climate device. Filters can be used on the underlying sensor to control the rate of updates and values reported by this component. Units must be `°C` or `°F`. The temperature sensor provided by this project, an onboard sensor provided by a different ESPHome pomponent or an external Home Assistant provided sensor are all valid options. Omitting this option will report the Daikin measured temperature, bypassing sensor filtering.
* **humidity_sensor** (*Optional*, Sensor ID): Humidity sensor to report as the current humidity of the climate device. Filters can be used on the underlying sensor to control the rate of updates and values reported by this component. Units must be `%`. The humidity sensor provided by this project, an onboard sensor provided by a different ESPHome pomponent or an external Home Assistant provided sensor are all valid options. Omitting this option will disable humidity reporting.
* **supported_modes** (*Optional*, list of strings): A list of allowed modes to present in case your unit doesn't support them all or you otherwise want to restrict availability. `OFF`, `HEAT_COOL`, `COOL`, `HEAT`, `FAN_ONLY` and `DRY` are valid options. `OFF` is always available. Defaults to all options being enabled.
* **supported_swing_modes** (*Optional*, list of strings): A list of allowed swing modes to present in case your unit doesn't support them all or you otherwise want to restrict availability. `OFF`, `HORIZONTAL`, `VERTICAL` and `BOTH` are valid options. `OFF` is always available. If only `OFF` is specified swing controls will be disabled. Defaults to all options being enabled.
* **heat_cool_mode** (*Optional*, mode parameters): Parameters to use in `HEAT_COOL` mode. Defaults to a 18°-30° range with no offset.
* **cool_mode** (*Optional*, setpoint mode parameters): Parameters to use in `COOL` and `DRY` modes. Defaults to a 18°-32° range with no offset.
* **heat_mode** (*Optional*, setpoint mode parameters): Parameters to use in `HEAT` mode. Defaults to a 10°-30° range with no offset.

Setpoint mode parameters schema, all temperatures in Celcius:
* **offset** (*Optional*, Temperature): Static offset to apply to the internal setpoint. If your system is adequately sized for your environment and consistently overshoots the setpoint by `+N` degrees, set this to `-N` to have the component issue `setpoint-N` to the unit under the hood so the controlled temperature is hopefully just `N`. Your service manual will have the control parameters listed to give you an idea of this overshoot value but looking back on your Home Assistant history will be easiest.
* **min_temperature** (*Optional*, Temperature): Minimum temperature to command the unit to in this mode. Prevents a target setpoint derived from the sensor and static offset from being out of range and failing to write.
* **max_temperature** (*Optional*, Temperature): Maximum temperature to command the unit to in this mode. Prevents a target setpoint derived from the sensor and static offset from being out of range and failing to write.

### Number

This component supports other options inherited from the ESPHome [Number Component](https://esphome.io/components/number/).

```yaml
# Example configuration entry
number:
  - platform: daikin_s21
    demand:
      name: Demand
```

* **demand** (*Optional*, Number): Modulates conditioning power ouptut between 30%-100%. As a reference point, the built-in Econo mode limits this to around 70%. Note that this isn't a direct control of energy consumption. Supported on some v2+ units. If unsupported it should revert to `100%` when changed.

### Select

These components supports other options inherited from the ESPHome [Select Component](https://esphome.io/components/select/).

```yaml
# Example configuration entry
select:
  - platform: daikin_s21
    brightness:
      name: LED Brightness
    humidity:
      name: Humidity
    vertical_swing:
      name: Vertical Swing
      cool_action: [80, 70, 60, 50, 44]
      fan_only_action: [87, 66, 46, 26, 5]
      heat_action: [66, 53, 40, 27, 14]
```

* **brightness** (*Optional*, Select): Selects LED brightness. The value may change when using IR remote modes that make use of the PIR sensor. If this control works for you, it's probably a good indication there's no need to enable the Sensor LED or Sensor Mode switches or binary sensors as the functions are mutually exclusive in the protocol. Supported on some v2+ units.
* **humidity** (*Optional*, Select): Selects humidity control operation strength while in heating and cooling modes to provide dry cooling or humid heating. The operation of this isn't well understood due to lack of user feedback. If you want to help, please post your findings in the discussions section or open an issue if you're willing to assist further development. Supported on some v2+ units, though you may need one advertising the "Ururu Sarara" feature set.
* **vertical_swing** (*Optional*, Select): Selects the vertical louvre swing setpoint. This functionality is implemented in an additional control layer on top of Daikin's commands so this is only going to work well if the core query rate is free running or the polled angle values will be stale. Horizontal swing is preserved while active. Some units (v2+) may support a command to do this directly. On those that don't, vertical swing is temporarily enabled and an attempt is made to pause the louvre in the general area of the desired angle. These angles can be optionally configured per action and are only used when there's no command support. The range may vary on your unit, turn on vertical swing and monitor the angles for your mode and unit to come up with your presets. You may find that stopping vertical swing causes this control to snap to the nearest discrete setpoint.
  * **cool_action** (*Optional*, list of 5 uint8_t angles): Angles to use for the Select setpoints in `COOLING` and `DRY` actions, from from top to bottom, when direct command support is missing. Defaults to [80, 70, 60, 50, 44].
  * **fan_only_action** (*Optional*, list of 5 uint8_t angles): Angles to use for the Select setpoints in `FAN_ONLY` action, from from top to bottom, when direct command support is missing. Defaults to [87, 66, 46, 26, 5].
  * **heat_action** (*Optional*, list of 5 uint8_t angles): Angles to use for the Select setpoints in `HEATING` action, from from top to bottom, when direct command support is missing. Defaults to [66, 53, 40, 27, 14].

### Sensor

Sensors in ESPHome are geared more towards a periodic ADC value that can be filtered as necessary than the pre-processed values from the Daikin unit that won't normally require averaging filters and the like. This makes sensors quite noisy on the network by default as every sample is published. See the configuration for ways to work with this.

Some sensors refer to values reported by the outdoor unit to the indoor unit and are a good candidate for an "outdoor" subdevice to collect shared readings from a single indoor unit.

These components supports other options inherited from the ESPHome [Sensor Component](https://esphome.io/components/sensor/).

```yaml
# Example configuration entry
sensor:
  - platform: daikin_s21
    inside_temperature:
      id: daikin_temperature
      name: Inside Temperature
      filters:
        - delta: 0.0
    outside_temperature:
      name: Outside Temperature
      device_id: daikin_outdoor
      filters:
        - delta: 0.0
    coil_temperature:
      name: Coil Temperature
      # device_id: daikin_outdoor
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
      # unit_of_measurement: "RPM"  # if you would prefer RPM, uncomment the following lines
      # device_class: ""  # override default "frequency", RPM is not a measurement of frequency in Home Assistant
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
```

* **update_interval** (*Optional*, Time): Sensor publishing rate, limits publishing over the network to an interval. To make this reporting more responsive, the user can set this value to `never` and every reading from the unit will be published. To reduce spam when doing this you can configure a delta filter on your sensors and only changes will be published over the network. Set to `never` for free run mode and publish every sensor on a polling interval (~3s), unchanged or uninteresting. Defaults to `never`.
* **coil_temperature** (*Optional*, Sensor): The temperature reading of the liquid coil of the indoor unit. Values will be roughly the same on all active indoor units (though they are independent measurements) so this is a good candidate to consider as shared on all indoor units.
* **compressor_frequency** (*Optional*, Sensor): The frequency of the compressor of the outdoor unit in Hz. You could convert this to RPM if you choose, see the example configuration. Shared on all indoor units.
* **demand** (*Optional*, Sensor): Unitless demand signal from the indoor unit to the outdoor unit from 0-15. This is an indication of the pull of the indoor unit and used to determine the overall compressor frequency to meet that demand. I find this nice to view as a percentage, see the example.
* **energy** (*Optional*, Sensor): Total energy consumption of indoor units, in kWh. Shared on all indoor units. Supported on some v2+ units.
* **energy_cooling** (*Optional*, Sensor): Total unit energy consumption in cooling modes in kWh. Supported on some v3.20+ units.
* **energy_heating** (*Optional*, Sensor): Total unit energy consumption in heating modes in kWh. Supported on some v3.20+ units.
* **fan_speed** (*Optional*, Sensor): Blower fan speed in RPM.
* **humidity** (*Optional*, Sensor): Relative humidity reading in %. Can report a consistent 50% or 0% if not present. Not supported on all units.
* **ir_counter** (*Optional*, Sensor): A value that increments when the IR remote is used. Supported on some v2+ units.
* **inside_temperature** (*Optional*, Sensor): The ambient temperature measured at the indoor unit air return.
* **outdoor_capacity** (*Optional*, Sensor): Outdoor unit capacity in indoor units. Static value, so not very interesting. Shared on all indoor units. Supported on some v2+ units.
* **outside_temperature** (*Optional*, Sensor): The ambient temperature measured at the outdoor compressor. Shared on all indoor units.
* **power** (*Optional*, Sensor): The instantaneous power draw of the unit in W. If you have a multihead system, sampling jitter may result in incorrect results when summed into energy consumption. Use as a rough indicator only and prefer the real energy consumption sensors for your dashboard. Supported on some v3.20+ units.
* **setpoint_temperature** (*Optional*, Sensor): The setpoint temperature after taking any Climate provided offsets into account.
* **swing_vertical_angle** (*Optional*, Sensor): The angle of the vertical louvres in degrees. This uses Daikin's reference frame, 90 is horizontal and 0 is vertical for my wall units.
* **target_temperature** (*Optional*, Sensor): The internal target temperature after taking special modes into account. This is not the user setpoint and usually not that interesting but may help with debugging or your automations.

### Switch

Mode toggle switches. In some cases read only support exists for reporting the current value set by the IR remote. If this is true for you, a parallel set of binary sensors are offered as a read only indication. Use those instead if you're unable to control them. There's no point in having both the switch and corresponding binary sensor active at the same time.

These components supports other options inherited from the ESPHome [Switch Component](https://esphome.io/components/switch/).

```yaml
# Example configuration entry
switch:
  - platform: daikin_s21
    comfort:
      name: Comfort Mode Switch
    econo:
      name: Econo Mode Switch
    led:
      name: Sensor LED Switch
    motion:
      name: Sensor Mode Switch
    powerful:
      name: Powerful Mode Switch
    quiet:
      name: Quiet Mode Switch
    streamer:
      name: Streamer Mode Switch
```

* **comfort** (*Optional*, Switch): Toggles Comfort mode via the `D6` command. Supported on some v2+ units.
* **econo** (*Optional*, Switch): Toggles Econo mode via the `D7` command. Supported on some v2+ units.
* **led** (*Optional*, Switch): Toggles PIR sensor ("Intelligent Eye") LED via the `D6` command. Mutually exclusive with the LED brightness select functionality. Supported on some v2+ units.
* **motion** (*Optional*, Switch): Toggles PIR sensor ("Intelligent Eye") mode via the `D6` command. Mutually exclusive with the LED brightness select functionality. Supported on some v2+ units.
* **powerful** (*Optional*, Switch): Toggles Powerful mode via the `D6` command. Supported on some v2+ units.
* **quiet** (*Optional*, Switch): Toggles Quiet mode via the `D6` command. This mode limits the outdoor compressor operation to reduce noise, it's not the lowest indoor fan speed. Supported on some v2+ units.
* **streamer** (*Optional*, Switch): Toggles Streamer mode via the `D6` command. Supported on some v2+ units.

### Text Sensor

Text sensors that supply diagnostic information. Only published when their values change.

These components support other options inherited from the ESPHome [Text Sensor Component](https://esphome.io/components/text_sensor/).

```yaml
# Example configuration entry
text_sensor:
  - platform: daikin_s21
    model:
      name: Model
    software_revision:
      name: Software Revision
    software_version:
      name: Software Version
    queries:
      - RK
```

* **model** (*Optional*, Text Sensor): Model name as read out from the unit's `FU05` query. Supported on some v3.40+ units. If unsupported the state will be `"unknown"`.
* **software_revision** (*Optional*, Text Sensor): Software revision as read out from the unit's `FY10` query. Supported on some v3+ units. If unsupported the state will be `"unknown"`.
* **software_version** (*Optional*, Text Sensor): Software version as read out from the unit's `VS000M` query. Not supported on all units. If unsupported the state will be `"unknown"`.
* **queries** (*Optional*, list of strings): A list of raw, case sensitive query strings to poll from the unit. Useful for debugging internal state and protocol reverse engineering. Text sensors will be created for each query name in the list.

If you've read the Faikout wiki you'll see many more queries available than what this project supports. The `queries` option is intended to read these raw values out without having to add a dedicated sensor in code. Normally you wouldn't need to use this, but if a query looks interesting you can see the value change over time in Home Assistant. I'd prefer if we use this to confirm a query works and then add proper support for what it represents, rather than trying to interpret the string via Home Assistant. Open an issue with details if you want a sensor or control added.

### Recommendations

On multihead systems some values that come from the outdoor unit will be the same (accounting for sampling jitter). It's recommended to use ESPHome's subdevice feature to only configure these sensors on your "primary" ESPHome instance as a subdevice to keep individual indoor unit representations cleaner. See the configuration example below where I wire these up to the `daikin_outdoor` subdevice I configure. This feature seems designed for the one-to-many case, whereas this system is more of a many-to-one configuration. Locally I have two configuration template files, one for indoor only units and one for a single indoor unit with an outdoor unit subdevice. If I can find a better solution I'll update the example. You could also use this feature to hide away a bunch of stats in a subdevice and leave the primary interface clean.

When using an external temperature sensor as a room reference instead of the internal Daikin value it's recommended to change the target_temperature step to 0.5°C to reflect the granularity of the alternate control loop provided. The default is 1.0°C to match Daikin's common internal granularity.

ESPHome doesn't offer an easy way for someone of my familiarity to dynamically enable components based on feature detection from the unit, so your best bet at the moment is to include components you'd like to use and remove them later if they're not supported. With newer features I could have more development to do (I can't test locally), so if something should be working based on the queries available and the protocol version of your unit don't hesitate to reach out.

If something isn't working, connect to your ESPHome device to read out logs. You should enable the `protocol_debug` option in the configuration to provide additional information I will require to help you. Every minute you'll see a general state debug dump printed with lists of queries.

If your unit is failing to poll `F8` and `FY00` over and over you most likely have a basic hardware communications problem, either wiring, UART interface circuitry or UART interface issue. Please review the [Hardware](#hardware) section for more information.

### Examples

```yaml
# Example configuration
esphome:
  min_version: "2026.7.0"
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
    components: [ daikin_s21 ]

uart:
  tx_pin: GPIO1
  rx_pin: GPIO3
  baud_rate: 2400
  data_bits: 8
  parity: EVEN
  stop_bits: 2

# The parent UART communication hub platform.
daikin_s21:

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
    # supported_modes:  # optional, restricts available climate modes. off is always supported.
    #   - heat_cool
    #   - cool
    #   - heat
    #   - dry
    #   - fan_only
    # supported_swing_modes:  # optional, restricts available swing modes. off is always supported.
    #   - horizontal
    #   - vertical
    #   - both
    # update_interval: never # Interval used to limit sensor publishing rate, 'never' for free run
    # offset_interval: 5min # Interval used to adjust the unit's setpoint using finer grained control, 'never' for free run
    # preset: true  # Enable preset selection for Ururu Sarara humidity control modes
    # Optional sensors to use for temperature and humidity references
    sensor: daikin_temperature  # Internal, see indoor temperature sensor below
    # sensor: room_temp  # External, see homeassistant sensor below
    humidity_sensor: daikin_humidity  # Internal, see humidity sensor below
    # humidity_sensor: room_humidity  # External, see homeassistant sensor below
    # or leave unconfigured if unsupported to omit humidity reporting
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
      # angles for vertical swing setpoint angles, from top to bottom
      # cool_action: [80, 70, 60, 50, 44] # shared with dry
      # fan_only_action: [87, 66, 46, 26, 5]
      # heat_action: [66, 53, 40, 27, 14]

sensor:
  - platform: daikin_s21
    # update_interval: never  # Publishing interval, 'never' for immediate updates but may be noisy if you don't use filters
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
    multizone_online:
      name: Multizone Online
    online:
      name: Online
    # serial_error:
    #   name: Serial Error
    short_cycle:
      name: Short Cycle
      device_id: daikin_outdoor
    system_defrost:
      name: System Defrost
      device_id: daikin_outdoor
    system_online:
      name: System Online
      device_id: daikin_outdoor

text_sensor:
  - platform: daikin_s21
    model:
      name: Model
    software_revision:
      name: Software Revision
    software_version:
      name: Software Version

number:
  - platform: daikin_s21
    demand:
      name: Demand
```

Here is an example of a UART configuration for a newer Faikout PCB revision with inverted UART pins and a receiver with an internal pullup:

```yaml
uart:
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
```

Here's an example of a UART configuration for an ESP32 dev board using direct wiring, relying on Daikin's pullups and an open drain transmitter:

```yaml
uart:
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
```

## Contributing

If you can write C++, great. Even if you're more of a YAML writer or user, your assistance with this project would be helpful. Here are some possible ways:

* Report your experince with different Daikin units. Turning on protocol debugging and getting the protocol and model detection output values would be the first step if you encounter issues and help me learn more about the output of different models. The 1 minute debug state dump in the logs with `protocol_debug` enabled is valuable for documentation and support.
* Let me know how useful the binary sensor values are and which just shadow other sensor values.
* Tell me if there are any problems with the control code for your units. The supported features vary wildly so you may find control or even readout of features or sensors doesn't work for you. This is normal, especially for my v0 units. If you find that your unit does support something via enabling protocol debugging or testing with debug queries then I would like to add support for it.
* Please consider documenting characteristics of your unit in the appropriate place in the Faikout wiki. I don't want to make an inferior copy of this information if possible. Please don't report issues with this component to them as we don't share a codebase. Some of the readout values here are not byteswapped if we don't otherwise use them so unknown static fields may appear reversed compared to that project. Please verify this when contributing.
* Check the discussions section on Github for things that may not be in this readme yet.

See existing issues, open a new one or post in the discussions section with your findings. Thanks.
