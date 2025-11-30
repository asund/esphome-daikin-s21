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
    ENTITY_CATEGORY_DIAGNOSTIC,
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
CONF_SOFTWARE_VERSION = "software_version"
ICON_TEXT = "mdi:text"

CONFIG_SCHEMA = (
    cv.COMPONENT_SCHEMA
    .extend({cv.GenerateID(): cv.declare_id(DaikinS21TextSensor)})
    .extend(S21_PARENT_SCHEMA)
    .extend({
        cv.Optional(CONF_SOFTWARE_VERSION): text_sensor.text_sensor_schema(
            icon=ICON_TEXT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    })
    .extend({cv.Optional(CONF_QUERIES): cv.ensure_list(cv.string, cv.Length(min=1, max=5))})
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_S21_ID])

    sensors = (
        (CONF_SOFTWARE_VERSION, var.set_software_version_sensor),
    )
    for key, func in sensors:
        if key in config:
            sens = await text_sensor.new_text_sensor(config[key])
            cg.add(func(sens))

    if (CONF_QUERIES in config):
        debug_sensors = []
        for query in config[CONF_QUERIES]:
            # fake a config for each text sensor -- probably a better way to do this
            cfg = ConfigType({
                cv.GenerateID(): cv.use_id(text_sensor.TextSensor)(f'text_sensor_textsensor_{query}'),
                CONF_NAME: query,
                CONF_DISABLED_BY_DEFAULT: False,
                CONF_ICON: ICON_BUG,
                CONF_ENTITY_CATEGORY: cg.EntityCategory.ENTITY_CATEGORY_DIAGNOSTIC,
            })
            debug_sensors.append(await text_sensor.new_text_sensor(cfg))
        cg.add(var.set_debug_query_sensors(debug_sensors))
