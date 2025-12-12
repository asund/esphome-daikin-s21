"""
Switch component for daikin_s21.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import (
    CONF_ID,
)

from .. import (
    daikin_s21_ns,
    CONF_S21_ID,
    S21_PARENT_SCHEMA,
)

DaikinS21Switch = daikin_s21_ns.class_("DaikinS21Switch", cg.Component)
DaikinS21SwitchMode = daikin_s21_ns.class_("DaikinS21SwitchMode", switch.Switch)

CONF_POWERFUL = "powerful"
CONF_COMFORT = "comfort"
CONF_QUIET = "quiet"
CONF_STREAMER = "streamer"
CONF_SENSOR = "sensor"
CONF_ECONO = "econo"

CONFIG_SCHEMA = (
    cv.COMPONENT_SCHEMA
    .extend({cv.GenerateID(): cv.declare_id(DaikinS21Switch)})
    .extend(S21_PARENT_SCHEMA)
    .extend({
        cv.Optional(CONF_POWERFUL): switch.switch_schema(
            DaikinS21SwitchMode,
            icon="mdi:arm-flex",
        ).extend(S21_PARENT_SCHEMA),
        cv.Optional(CONF_COMFORT): switch.switch_schema(
            DaikinS21SwitchMode,
            icon="mdi:account-check",
        ).extend(S21_PARENT_SCHEMA),
        cv.Optional(CONF_QUIET): switch.switch_schema(
            DaikinS21SwitchMode,
            icon="mdi:volume-minus",
        ).extend(S21_PARENT_SCHEMA),
        cv.Optional(CONF_STREAMER): switch.switch_schema(
            DaikinS21SwitchMode,
            icon="mdi:creation",
        ).extend(S21_PARENT_SCHEMA),
        cv.Optional(CONF_SENSOR): switch.switch_schema(
            DaikinS21SwitchMode,
            icon="mdi:motion-sensor",
        ).extend(S21_PARENT_SCHEMA),
        cv.Optional(CONF_ECONO): switch.switch_schema(
            DaikinS21SwitchMode,
            icon="mdi:leaf",
        ).extend(S21_PARENT_SCHEMA),
    })
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_S21_ID])

    switches = (
      (CONF_POWERFUL, var.set_powerful_switch),
      (CONF_COMFORT, var.set_comfort_switch),
      (CONF_QUIET, var.set_quiet_switch),
      (CONF_STREAMER, var.set_streamer_switch),
      (CONF_SENSOR, var.set_sensor_switch),
      (CONF_ECONO, var.set_econo_switch),
    )
    for key, func in switches:
        if key in config:
            sw = await switch.new_switch(config[key])
            await cg.register_parented(sw, config[CONF_S21_ID])
            cg.add(func(sw))
