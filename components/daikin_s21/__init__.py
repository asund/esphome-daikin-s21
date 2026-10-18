"""
Daikin S21 Mini-Split ESPHome component config validation & code generation.
"""

import esphome.codegen as cg
from esphome.components import uart
import esphome.config_validation as cv
from esphome.const import (
  CONF_ID,
  CONF_LED,
  CONF_MOTION,
  CONF_NEVER,
  CONF_UART_ID,
  CONF_UPDATE_INTERVAL,
  ICON_ACCOUNT_CHECK,
)
import logging

DEPENDENCIES = ["uart"]

CONF_S21_ID = "s21_id"
CONF_ANGLE_SCALE = "angle_scale"
CONF_DEBUG_COMMS = "debug_comms"
CONF_DEBUG_PROTOCOL = "debug_protocol"
CONF_UART = "uart"  # deprecated

# used by platform components
CONF_COMFORT = "comfort"
CONF_ECONO = "econo"
CONF_POWERFUL = "powerful"
CONF_QUIET = "quiet"
CONF_STREAMER = "streamer"
ICON_COMFORT = ICON_ACCOUNT_CHECK
ICON_ECONO = "mdi:leaf"
ICON_LED_ON = "mdi:led-on"
ICON_POWERFUL = "mdi:arm-flex"
ICON_QUIET = "mdi:volume-minus"
ICON_STREAMER = "mdi:creation"
ICON_VERTICAL_SWING = "mdi:pan-vertical"

daikin_s21_ns = cg.esphome_ns.namespace("daikin_s21")
DaikinS21 = daikin_s21_ns.class_("DaikinS21", cg.PollingComponent, uart.UARTDevice)
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

# ensure polling intervals aren't 0ms or fixed up by ESPHome to 1ms
validate_interval = cv.All(cv.update_interval, cv.Range(min=cv.TimePeriodMilliseconds(milliseconds=2)))

CONFIG_SCHEMA = (
    cv.COMPONENT_SCHEMA
    .extend({cv.GenerateID(): cv.declare_id(DaikinS21)})
    .extend(cv.polling_component_schema(CONF_NEVER))
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend({
      cv.Optional(CONF_ANGLE_SCALE): cv.one_of(1, 10, int=True),
      cv.Optional(CONF_UPDATE_INTERVAL): validate_interval,
      cv.Optional(CONF_UPDATE_INTERVAL): validate_interval,
      cv.Optional(CONF_UART): cv.use_id(UARTComponent), # deprecated
      cv.Optional(CONF_DEBUG_COMMS): cv.boolean,
      cv.Optional(CONF_DEBUG_PROTOCOL): cv.boolean,
    })
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "daikin_s21",
    baud_rate=2400,
    require_tx=True,
    require_rx=True,
    data_bits=8,
    parity="EVEN",
    stop_bits=2,
)

S21_PARENT_SCHEMA = cv.Schema({cv.GenerateID(CONF_S21_ID): cv.use_id(DaikinS21)})

async def to_code(config):
    if CONF_UART in config:
      logging.getLogger(__name__).warning("`daikin_s21` config schema change: use `uart_id` if using more than one UART. Support for `uart` will be removed in the future.")
      config[CONF_UART_ID] = config.pop(CONF_UART)

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_ANGLE_SCALE in config:
      cg.add(var.set_angle_scale(config[CONF_ANGLE_SCALE]))
    if CONF_DEBUG_COMMS in config:
      cg.add(var.set_debug_comms(config[CONF_DEBUG_COMMS]))
    if CONF_DEBUG_PROTOCOL in config:
      cg.add(var.set_debug_protocol(config[CONF_DEBUG_PROTOCOL]))
