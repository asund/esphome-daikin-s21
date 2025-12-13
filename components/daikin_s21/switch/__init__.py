"""
Switch component for daikin_s21.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import (
    CONF_ID,
    CONF_MOTION,
    ICON_MOTION_SENSOR,
)

from .. import (
    daikin_s21_ns,
    CONF_S21_ID,
    S21_PARENT_SCHEMA,
    DaikinS21Modes,
    CONF_ECONO,
    CONF_COMFORT,
    CONF_POWERFUL,
    CONF_QUIET,
    CONF_STREAMER,
    ICON_ECONO,
    ICON_COMFORT,
    ICON_POWERFUL,
    ICON_QUIET,
    ICON_STREAMER,
)

DaikinS21Switch = daikin_s21_ns.class_("DaikinS21Switch", cg.Component)
DaikinS21SwitchMode = daikin_s21_ns.class_("DaikinS21SwitchMode", switch.Switch)

CONFIG_SCHEMA = (
    cv.COMPONENT_SCHEMA
    .extend({cv.GenerateID(): cv.declare_id(DaikinS21Switch)})
    .extend(S21_PARENT_SCHEMA)
    .extend({
        cv.Optional(CONF_POWERFUL): switch.switch_schema(
            DaikinS21SwitchMode,
            icon=ICON_POWERFUL,
        ).extend(S21_PARENT_SCHEMA),
        cv.Optional(CONF_COMFORT): switch.switch_schema(
            DaikinS21SwitchMode,
            icon=ICON_COMFORT,
        ).extend(S21_PARENT_SCHEMA),
        cv.Optional(CONF_QUIET): switch.switch_schema(
            DaikinS21SwitchMode,
            icon=ICON_QUIET,
        ).extend(S21_PARENT_SCHEMA),
        cv.Optional(CONF_STREAMER): switch.switch_schema(
            DaikinS21SwitchMode,
            icon=ICON_STREAMER,
        ).extend(S21_PARENT_SCHEMA),
        cv.Optional(CONF_MOTION): switch.switch_schema(
            DaikinS21SwitchMode,
            icon=ICON_MOTION_SENSOR,
        ).extend(S21_PARENT_SCHEMA),
        cv.Optional(CONF_ECONO): switch.switch_schema(
            DaikinS21SwitchMode,
            icon=ICON_ECONO,
        ).extend(S21_PARENT_SCHEMA),
    })
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_S21_ID])

    mode_switches = (
      (CONF_POWERFUL, DaikinS21Modes.ModePowerful),
      (CONF_COMFORT, DaikinS21Modes.ModeComfort),
      (CONF_QUIET, DaikinS21Modes.ModeQuiet),
      (CONF_STREAMER, DaikinS21Modes.ModeStreamer),
      (CONF_MOTION, DaikinS21Modes.ModeMotionSensor),
      (CONF_ECONO, DaikinS21Modes.ModeEcono),
    )
    for key, mode in mode_switches:
        if key in config:
            sw = await switch.new_switch(config[key], mode)
            await cg.register_parented(sw, config[CONF_S21_ID])
            cg.add(var.set_mode_switch(sw))
