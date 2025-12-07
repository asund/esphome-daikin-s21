"""
Binary sensor component for daikin_s21.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_COLD,
    DEVICE_CLASS_LOCK,
    DEVICE_CLASS_OPENING,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_RUNNING,
)

from .. import (
    daikin_s21_ns,
    CONF_S21_ID,
    S21_PARENT_SCHEMA,
)

DaikinS21BinarySensor = daikin_s21_ns.class_("DaikinS21BinarySensor", cg.Component)

CONF_POWERFUL = "powerful"
CONF_DEFROST = "defrost"
CONF_ACTIVE = "active"
CONF_ONLINE = "online"
CONF_VALVE = "valve"
CONF_SHORT_CYCLE = "short_cycle"
CONF_SYSTEM_DEFROST = "system_defrost"
CONF_MULTIZONE_CONFLICT = "multizone_conflict"

CONFIG_SCHEMA = (
    cv.COMPONENT_SCHEMA
    .extend({cv.GenerateID(): cv.declare_id(DaikinS21BinarySensor)})
    .extend(S21_PARENT_SCHEMA)
    .extend({
        cv.Optional(CONF_POWERFUL): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_POWER,
        ),
        cv.Optional(CONF_DEFROST): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_COLD,
        ),
        cv.Optional(CONF_ACTIVE): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_RUNNING,
        ),
        cv.Optional(CONF_ONLINE): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_POWER,
        ),
        cv.Optional(CONF_VALVE): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_OPENING,
        ),
        cv.Optional(CONF_SHORT_CYCLE): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_LOCK,
        ),
        cv.Optional(CONF_SYSTEM_DEFROST): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_COLD,
        ),
        cv.Optional(CONF_MULTIZONE_CONFLICT): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_LOCK,
        ),
    })
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_S21_ID])

    binary_sensors = (
        (CONF_POWERFUL, var.set_powerful_sensor),
        (CONF_DEFROST, var.set_defrost_sensor),
        (CONF_ACTIVE, var.set_active_sensor),
        (CONF_ONLINE, var.set_online_sensor),
        (CONF_VALVE, var.set_valve_sensor),
        (CONF_SHORT_CYCLE, var.set_short_cycle_sensor),
        (CONF_SYSTEM_DEFROST, var.set_system_defrost_sensor),
        (CONF_MULTIZONE_CONFLICT, var.set_multizone_conflict_sensor),
    )
    for key, func in binary_sensors:
        if key in config:
            sens = await binary_sensor.new_binary_sensor(config[key])
            cg.add(func(sens))
