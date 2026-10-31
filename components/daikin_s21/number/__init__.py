"""
Number component for daikin_s21.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    CONF_ID,
    ICON_PERCENT,
    UNIT_PERCENT,
)

from .. import (
    daikin_s21_ns,
    CONF_S21_ID,
    S21_PARENT_SCHEMA,
)

DaikinS21Number = daikin_s21_ns.class_("DaikinS21Number", cg.Component)
DaikinS21NumberDemand = daikin_s21_ns.class_("DaikinS21NumberDemand", number.Number)

CONF_DEMAND = "demand"

CONFIG_SCHEMA = (
    cv.COMPONENT_SCHEMA
    .extend({cv.GenerateID(): cv.declare_id(DaikinS21Number)})
    .extend(S21_PARENT_SCHEMA)
    .extend({
        cv.Optional(CONF_DEMAND): number.number_schema(
            DaikinS21NumberDemand,
            unit_of_measurement=UNIT_PERCENT,
            icon=ICON_PERCENT,
        ),
    })
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_S21_ID])
    numbers = number.sub_numbers(config, parent=config[CONF_S21_ID])
    await numbers(CONF_DEMAND, var.set_demand, min_value=30, max_value=100, step=1)
