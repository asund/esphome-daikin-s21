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

    numbers = (
        (CONF_DEMAND, var.set_demand, 30, 100, 1),
    )
    for key, func, min_value, max_value, step in numbers:
        if key in config:
            num = await number.new_number(
                config[key],
                min_value=min_value,
                max_value=max_value,
                step=step,
            )
            await cg.register_parented(num, config[CONF_S21_ID])
            cg.add(func(num))
