"""
Split UART component for ESPHome. Combines two UARTs into one to workaround an Arduino framework limitation with different pin invert settings.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]

CONF_TX_UART = "tx_uart"
CONF_RX_UART = "rx_uart"

split_uart_component_ns = cg.esphome_ns.namespace("split_uart")
SplitUART = split_uart_component_ns.class_("SplitUART", uart.UARTComponent, cg.Component)
uart_ns = cg.esphome_ns.namespace("uart")
UARTComponent = uart_ns.class_("UARTComponent")

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SplitUART),
            cv.Required(CONF_TX_UART): cv.use_id(UARTComponent),
            cv.Required(CONF_RX_UART): cv.use_id(UARTComponent),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    tx_uart = await cg.get_variable(config[CONF_TX_UART])
    rx_uart = await cg.get_variable(config[CONF_RX_UART])
    var = cg.new_Pvariable(config[CONF_ID], tx_uart, rx_uart)
    await cg.register_component(var, config)
