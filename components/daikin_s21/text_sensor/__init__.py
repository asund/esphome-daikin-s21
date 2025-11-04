"""
Text sensor component for daikin_s21.
"""

import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_DISABLED_BY_DEFAULT,
    CONF_ENTITY_CATEGORY,
    CONF_ICON,
    CONF_ID,
    CONF_NAME,
    ICON_BUG,
)
from esphome.types import ConfigType

from .. import (
    daikin_s21_ns,
    CONF_S21_ID,
    S21_PARENT_SCHEMA,
)

DaikinS21TextSensor = daikin_s21_ns.class_(
    "DaikinS21TextSensor", cg.Component
)

CONF_QUERIES = "queries"

CONFIG_SCHEMA = (
    cv.COMPONENT_SCHEMA
    .extend({cv.GenerateID(): cv.declare_id(DaikinS21TextSensor)})
    .extend(S21_PARENT_SCHEMA)
    .extend({cv.Required(CONF_QUERIES): cv.ensure_list(cv.string, cv.Length(min=1, max=5))})
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_S21_ID])

    sensors = []
    for query in config[CONF_QUERIES]:
        # fake a config for each text sensor -- probably a better way to do this
        cfg = ConfigType({
            cv.GenerateID(): cv.use_id(text_sensor.TextSensor)(f'text_sensor_textsensor_{query}'),
            CONF_NAME: query,
            CONF_DISABLED_BY_DEFAULT: False,
            CONF_ICON: ICON_BUG,
            CONF_ENTITY_CATEGORY: cg.EntityCategory.ENTITY_CATEGORY_DIAGNOSTIC,
        })
        sensors.append(await text_sensor.new_text_sensor(cfg))
    cg.add(var.set_debug_query_sensors(sensors))
