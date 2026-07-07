"""
Select component for daikin_s21.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, select
from esphome.const import (
    CONF_BRIGHTNESS,
    CONF_COOL_ACTION,
    CONF_FAN_ONLY_ACTION,
    CONF_HEAT_ACTION,
    CONF_HUMIDITY,
    CONF_ID,
    ICON_WATER,
)

from .. import (
    daikin_s21_ns,
    CONF_S21_ID,
    S21_PARENT_SCHEMA,
    ICON_LED_ON,
    ICON_VERTICAL_SWING,
)

ClimateAction = cg.esphome_ns.namespace("climate").enum("ClimateAction")
DaikinS21Select = daikin_s21_ns.class_("DaikinS21Select", cg.Component)
DaikinS21SelectLEDBrightness = daikin_s21_ns.class_("DaikinS21SelectLEDBrightness", select.Select)
DaikinS21SelectHumidity = daikin_s21_ns.class_("DaikinS21SelectHumidity", select.Select)
DaikinS21SelectVerticalSwing = daikin_s21_ns.class_("DaikinS21SelectVerticalSwing", select.Select)

CONF_VERTICAL_SWING = "vertical_swing"

ANGLE_SEPOINT_VALIDATOR = cv.All(cv.ensure_list(cv.uint8_t), cv.Length(min=5, max=5))

CONFIG_SCHEMA = (
    cv.COMPONENT_SCHEMA
    .extend({cv.GenerateID(): cv.declare_id(DaikinS21Select)})
    .extend(S21_PARENT_SCHEMA)
    .extend({
        cv.Optional(CONF_BRIGHTNESS): select.select_schema(DaikinS21SelectLEDBrightness, icon=ICON_LED_ON)
        .extend(S21_PARENT_SCHEMA),
        cv.Optional(CONF_HUMIDITY): select.select_schema(DaikinS21SelectHumidity, icon=ICON_WATER)
        .extend(S21_PARENT_SCHEMA),
        cv.Optional(CONF_VERTICAL_SWING): select.select_schema(DaikinS21SelectVerticalSwing, icon=ICON_VERTICAL_SWING)
        .extend(S21_PARENT_SCHEMA)
        .extend({
            cv.Optional(CONF_COOL_ACTION, default=[80, 70, 60, 50, 44]): ANGLE_SEPOINT_VALIDATOR,
            cv.Optional(CONF_FAN_ONLY_ACTION, default=[87, 66, 46, 26, 5]): ANGLE_SEPOINT_VALIDATOR,
            cv.Optional(CONF_HEAT_ACTION, default=[66, 53, 40, 27, 14]): ANGLE_SEPOINT_VALIDATOR,
        }),
    })
)

async def to_code(config):
    parent = await cg.get_variable(config[CONF_S21_ID])
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_S21_ID])

    selects = (
        (CONF_BRIGHTNESS, var.set_brightness_select, ["High", "Low", "Off"]),
        (CONF_HUMIDITY, var.set_humidity_select, ["Off", "Low", "Standard", "High", "Continuous"]),
        (CONF_VERTICAL_SWING, var.set_vertical_swing_select, ["Off", "Top", "Upper", "Middle", "Lower", "Bottom", "On"]),
    )
    for key, func, options in selects:
        if key in config:
            sel = await select.new_select(config[key], options=options)
            await cg.register_parented(sel, config[CONF_S21_ID])
            cg.add(func(sel))

    cg.add(parent.set_vertical_angle_setpoints(ClimateAction.CLIMATE_ACTION_COOLING, config[CONF_VERTICAL_SWING][CONF_COOL_ACTION]))
    cg.add(parent.set_vertical_angle_setpoints(ClimateAction.CLIMATE_ACTION_FAN, config[CONF_VERTICAL_SWING][CONF_FAN_ONLY_ACTION]))
    cg.add(parent.set_vertical_angle_setpoints(ClimateAction.CLIMATE_ACTION_HEATING, config[CONF_VERTICAL_SWING][CONF_HEAT_ACTION]))