"""
Select component for daikin_s21.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select
from esphome.const import (
    CONF_ID,
    CONF_HUMIDITY,
    ICON_WATER,
)

from .. import (
    daikin_s21_ns,
    CONF_S21_ID,
    S21_PARENT_SCHEMA,
    ICON_VERTICAL_SWING,
)

DaikinS21Select = daikin_s21_ns.class_("DaikinS21Select", cg.Component)
DaikinS21SelectHumidity = daikin_s21_ns.class_("DaikinS21SelectHumidity", select.Select)
DaikinS21SelectVerticalSwing = daikin_s21_ns.class_("DaikinS21SelectVerticalSwing", select.Select)

CONF_VERTICAL_SWING = "vertical_swing"

CONFIG_SCHEMA = (
    cv.COMPONENT_SCHEMA
    .extend({cv.GenerateID(): cv.declare_id(DaikinS21Select)})
    .extend(S21_PARENT_SCHEMA)
    .extend({
        cv.Optional(CONF_HUMIDITY): select.select_schema(
            DaikinS21SelectHumidity,
            icon=ICON_WATER,
        ).extend(S21_PARENT_SCHEMA),
        cv.Optional(CONF_VERTICAL_SWING): select.select_schema(
            DaikinS21SelectVerticalSwing,
            icon=ICON_VERTICAL_SWING,
        ).extend(S21_PARENT_SCHEMA),
    })
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_S21_ID])

    selects = (
        (CONF_HUMIDITY, var.set_humidity_select, ["Off", "Low", "Standard", "High", "Continuous"]),
        (CONF_VERTICAL_SWING, var.set_vertical_swing_select, ["Off", "Top", "Upper", "Middle", "Lower", "Bottom", "On"]),
    )
    for key, func, options in selects:
        if key in config:
            sel = await select.new_select(config[key], options=options)
            await cg.register_parented(sel, config[CONF_S21_ID])
            cg.add(func(sel))