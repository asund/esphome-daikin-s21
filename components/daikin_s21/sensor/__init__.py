"""
Sensor component for daikin_s21.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_HUMIDITY,
    CONF_ID,
    UNIT_CELSIUS,
    UNIT_HERTZ,
    UNIT_PERCENT,
    UNIT_REVOLUTIONS_PER_MINUTE,
    ICON_FAN,
    DEVICE_CLASS_FREQUENCY,
    DEVICE_CLASS_HUMIDITY,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_WIND_DIRECTION,
    STATE_CLASS_MEASUREMENT,
)

from .. import (
    daikin_s21_ns,
    CONF_S21_ID,
    S21_CLIENT_SCHEMA,
    DaikinS21Client,
)

DaikinS21Sensor = daikin_s21_ns.class_(
    "DaikinS21Sensor", cg.PollingComponent, DaikinS21Client
)

CONF_S21_ID = "s21_id"
CONF_INSIDE_TEMP = "inside_temperature"
CONF_OUTSIDE_TEMP = "outside_temperature"
CONF_COIL_TEMP = "coil_temperature"
CONF_FAN_SPEED = "fan_speed"
CONF_SWING_VERTICAL_ANGLE = "swing_vertical_angle"
CONF_COMPRESSOR_FREQUENCY = "compressor_frequency"
CONF_DEMAND = "demand"

CONFIG_SCHEMA = (
    cv.COMPONENT_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(DaikinS21Sensor),
            cv.Optional(CONF_INSIDE_TEMP): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_OUTSIDE_TEMP): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_COIL_TEMP): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_FAN_SPEED): sensor.sensor_schema(
                unit_of_measurement=UNIT_REVOLUTIONS_PER_MINUTE,
                icon=ICON_FAN,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_FREQUENCY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_SWING_VERTICAL_ANGLE): sensor.sensor_schema(
                icon="mdi:pan-vertical",
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_WIND_DIRECTION,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_COMPRESSOR_FREQUENCY): sensor.sensor_schema(
                unit_of_measurement=UNIT_HERTZ,
                icon="mdi:pump",
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_FREQUENCY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_HUMIDITY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_DEMAND): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                icon="mdi:thermometer-chevron-up",
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
        }
    )
    .extend(S21_CLIENT_SCHEMA)
    .extend(cv.polling_component_schema("10s"))
)


async def to_code(config):
    """Generate main.cpp code"""

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    s21_var = await cg.get_variable(config[CONF_S21_ID])
    cg.add(var.set_s21(s21_var))

    sensors = (
        (CONF_INSIDE_TEMP, var.set_temp_inside_sensor),
        (CONF_OUTSIDE_TEMP, var.set_temp_outside_sensor),
        (CONF_COIL_TEMP, var.set_temp_coil_sensor),
        (CONF_FAN_SPEED, var.set_fan_speed_sensor),
        (CONF_SWING_VERTICAL_ANGLE, var.set_swing_vertical_angle_sensor),
        (CONF_COMPRESSOR_FREQUENCY, var.set_compressor_frequency_sensor),
        (CONF_HUMIDITY, var.set_humidity_sensor),
        (CONF_DEMAND, var.set_demand_sensor),
    )
    for key, func in sensors:
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(func(sens))
