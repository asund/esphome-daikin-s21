"""
Select component for daikin_s21.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select
from esphome.const import (
    CONF_ID,
)

from .. import (
    daikin_s21_ns,
    CONF_S21_ID,
    S21_PARENT_SCHEMA,
    ICON_VERTICAL_SWING,
)

DaikinS21Select = daikin_s21_ns.class_("DaikinS21Select", cg.Component)
DaikinS21SelectSwing = daikin_s21_ns.class_("DaikinS21SelectSwing", select.Select)
DaikinS21VerticalSwingMode = daikin_s21_ns.enum("DaikinVerticalSwingMode")

SWING_MODES = {
    "Off": DaikinS21VerticalSwingMode.Off,
    "Top": DaikinS21VerticalSwingMode.Top,
    "Upper": DaikinS21VerticalSwingMode.Upper,
    "Middle": DaikinS21VerticalSwingMode.Middle,
    "Lower": DaikinS21VerticalSwingMode.Lower,
    "Bottom": DaikinS21VerticalSwingMode.Bottom,
    "On": DaikinS21VerticalSwingMode.On,
}

CONF_VERTICAL_SWING = "vertical_swing"

CONFIG_SCHEMA = (
    cv.COMPONENT_SCHEMA
    .extend({cv.GenerateID(): cv.declare_id(DaikinS21Select)})
    .extend(S21_PARENT_SCHEMA)
    .extend({
        cv.Optional(CONF_VERTICAL_SWING): select.select_schema(
            DaikinS21SelectSwing,
            icon=ICON_VERTICAL_SWING,
        ).extend(S21_PARENT_SCHEMA),
    })
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_S21_ID])

    selects = (
        (CONF_VERTICAL_SWING, var.set_swing_select, list(SWING_MODES.keys())),
    )
    for key, func, options in selects:
        if key in config:
            sel = await select.new_select(config[key], options=options)
            await cg.register_parented(sel, config[CONF_S21_ID])
            cg.add(func(sel))