"""
Daikin S21 Mini-Split ESPHome component config validation & code generation.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import (
  CONF_ID,
  CONF_LED,
  CONF_MOTION,
  ICON_ACCOUNT_CHECK,
)

DEPENDENCIES = ["uart"]

CONF_DAIKIN_SERIAL_ID = "daikin_serial_id"
CONF_S21_ID = "s21_id"
CONF_UART = "uart"
CONF_DEBUG_COMMS = "debug_comms"
CONF_DEBUG_PROTOCOL = "debug_protocol"

# used by subcomponents
CONF_COMFORT = "comfort"
CONF_ECONO = "econo"
CONF_POWERFUL = "powerful"
CONF_QUIET = "quiet"
CONF_STREAMER = "streamer"
ICON_COMFORT = ICON_ACCOUNT_CHECK
ICON_ECONO = "mdi:leaf"
ICON_POWERFUL = "mdi:arm-flex"
ICON_QUIET = "mdi:volume-minus"
ICON_STREAMER = "mdi:creation"
ICON_VERTICAL_SWING = "mdi:pan-vertical"

daikin_s21_ns = cg.esphome_ns.namespace("daikin_s21")
DaikinS21 = daikin_s21_ns.class_("DaikinS21", cg.PollingComponent)
DaikinSerial = daikin_s21_ns.class_("DaikinSerial", cg.Component)
DaikinS21Modes = daikin_s21_ns.enum("DaikinMode")
DAIKIN_MODE_CONFIG_ENUMS = {
    CONF_POWERFUL: DaikinS21Modes.ModePowerful,
    CONF_COMFORT: DaikinS21Modes.ModeComfort,
    CONF_QUIET: DaikinS21Modes.ModeQuiet,
    CONF_STREAMER: DaikinS21Modes.ModeStreamer,
    CONF_LED: DaikinS21Modes.ModeSensorLED,
    CONF_MOTION: DaikinS21Modes.ModeMotionSensor,
    CONF_ECONO: DaikinS21Modes.ModeEcono,
}
uart_ns = cg.esphome_ns.namespace("uart")
UARTComponent = uart_ns.class_("UARTComponent")

CONFIG_SCHEMA = (
    cv.COMPONENT_SCHEMA
    .extend({cv.GenerateID(): cv.declare_id(DaikinS21)})
    .extend(cv.polling_component_schema("0s"))
    .extend({
      cv.GenerateID(CONF_DAIKIN_SERIAL_ID): cv.declare_id(DaikinSerial),
      cv.Required(CONF_UART): cv.use_id(UARTComponent),
      cv.Optional(CONF_DEBUG_COMMS, default=False): cv.boolean,
      cv.Optional(CONF_DEBUG_PROTOCOL, default=False): cv.boolean,
    })
)

S21_PARENT_SCHEMA = cv.Schema({cv.GenerateID(CONF_S21_ID): cv.use_id(DaikinS21)})

async def to_code(config):
    uart = await cg.get_variable(config[CONF_UART])
    serial = cg.new_Pvariable(config[CONF_DAIKIN_SERIAL_ID], uart)
    cg.add(serial.set_debug(config[CONF_DEBUG_COMMS]))
    await cg.register_component(serial, {})
    s21 = cg.new_Pvariable(config[CONF_ID], serial)
    await cg.register_component(s21, config)
    await cg.register_parented(serial, config[CONF_ID])
    cg.add(s21.set_debug(config[CONF_DEBUG_PROTOCOL]))
