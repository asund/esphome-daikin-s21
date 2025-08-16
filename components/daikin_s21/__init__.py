"""
Daikin S21 Mini-Split ESPHome component config validation & code generation.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import (
  CONF_ID,
  CONF_UART_ID,
)

DEPENDENCIES = ["uart"]

CONF_S21_ID = "s21_id"
CONF_DEBUG_COMMS = "debug_comms"
CONF_DEBUG_PROTOCOL = "debug_protocol"

daikin_s21_ns = cg.esphome_ns.namespace("daikin_s21")
DaikinS21 = daikin_s21_ns.class_("DaikinS21", cg.PollingComponent)
uart_ns = cg.esphome_ns.namespace("uart")
UARTComponent = uart_ns.class_("UARTComponent")

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(DaikinS21),
        cv.Required(CONF_UART_ID): cv.use_id(UARTComponent),
        cv.Optional(CONF_DEBUG_COMMS, default=False): cv.boolean,
        cv.Optional(CONF_DEBUG_PROTOCOL, default=False): cv.boolean,
    }
).extend(cv.polling_component_schema("0s"))

S21_PARENT_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_S21_ID): cv.use_id(DaikinS21),
    }
)

async def to_code(config):
    uart = await cg.get_variable(config[CONF_UART_ID])
    var = cg.new_Pvariable(config[CONF_ID], uart)
    await cg.register_component(var, config)
    cg.add(var.set_debug_comms(config[CONF_DEBUG_COMMS]))
    cg.add(var.set_debug_protocol(config[CONF_DEBUG_PROTOCOL]))
