"""
Daikin S21 Mini-Split ESPHome climate component config validation & code generation.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, sensor
from esphome.const import (
    CONF_COOL_MODE,
    CONF_HEAT_MODE,
    CONF_HEAT_COOL_MODE,
    CONF_HUMIDITY_SENSOR,
    CONF_MAX_TEMPERATURE,
    CONF_MIN_TEMPERATURE,
    CONF_OFFSET,
    CONF_SENSOR,
    CONF_SUPPORTED_MODES,
    CONF_SUPPORTED_SWING_MODES,
)
from .. import (
    daikin_s21_ns,
    CONF_S21_ID,
    S21_PARENT_SCHEMA,
)

AUTO_LOAD = ["sensor"]

DaikinS21Climate = daikin_s21_ns.class_("DaikinS21Climate", climate.Climate, cg.PollingComponent)

SUPPORTED_CLIMATE_MODES_OPTIONS = {
    "OFF": climate.ClimateMode.CLIMATE_MODE_OFF,  # always available
    "HEAT_COOL": climate.ClimateMode.CLIMATE_MODE_HEAT_COOL,
    "COOL": climate.ClimateMode.CLIMATE_MODE_COOL,
    "HEAT": climate.ClimateMode.CLIMATE_MODE_HEAT,
    "FAN_ONLY": climate.ClimateMode.CLIMATE_MODE_FAN_ONLY,
    "DRY": climate.ClimateMode.CLIMATE_MODE_DRY,
}

validate_supported_climate_mode = cv.enum(SUPPORTED_CLIMATE_MODES_OPTIONS, upper=True)

CONFIG_MODE_SCHEMA = cv.Schema({
    cv.Optional(CONF_OFFSET, default="0"): cv.temperature,
    cv.Optional(CONF_MAX_TEMPERATURE, default="30"): cv.temperature,
    cv.Optional(CONF_MIN_TEMPERATURE, default="18"): cv.temperature,
})

CONFIG_SCHEMA = (
    climate.climate_schema(DaikinS21Climate)
    .extend(cv.polling_component_schema("0s"))
    .extend(S21_PARENT_SCHEMA)
    .extend({
        cv.Optional(CONF_SENSOR): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_HUMIDITY_SENSOR): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_SUPPORTED_MODES, default=list(SUPPORTED_CLIMATE_MODES_OPTIONS)): cv.ensure_list(validate_supported_climate_mode),
        cv.Optional(CONF_SUPPORTED_SWING_MODES, default=list(climate.CLIMATE_SWING_MODES)): cv.ensure_list(climate.validate_climate_swing_mode),
        cv.Optional(CONF_HEAT_COOL_MODE, default={}): CONFIG_MODE_SCHEMA,
        cv.Optional(CONF_COOL_MODE, default={CONF_MAX_TEMPERATURE:"32", CONF_MIN_TEMPERATURE:"18"}): CONFIG_MODE_SCHEMA,
        cv.Optional(CONF_HEAT_MODE, default={CONF_MAX_TEMPERATURE:"30", CONF_MIN_TEMPERATURE:"10"}): CONFIG_MODE_SCHEMA,
    })
)

async def to_code(config):
    var = await climate.new_climate(config)
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_S21_ID])

    if CONF_SENSOR in config:
        sens = await cg.get_variable(config[CONF_SENSOR])
        cg.add(var.set_temperature_reference_sensor(sens))

    if CONF_HUMIDITY_SENSOR in config:
        sens = await cg.get_variable(config[CONF_HUMIDITY_SENSOR])
        cg.add(var.set_humidity_reference_sensor(sens))

    if "OFF" not in config[CONF_SUPPORTED_MODES]:
        config[CONF_SUPPORTED_MODES].append(validate_supported_climate_mode("OFF"))   # always supported
    if len(config[CONF_SUPPORTED_MODES]) > 1:   # don't generate code if just OFF, this is already the default
        cg.add(var.set_supported_modes(config[CONF_SUPPORTED_MODES]))

    if "OFF" not in config[CONF_SUPPORTED_SWING_MODES]:
        config[CONF_SUPPORTED_SWING_MODES].append(climate.validate_climate_swing_mode("OFF"))   # always supported
    if len(config[CONF_SUPPORTED_SWING_MODES]) > 1: # don't generate code if just OFF, leave empty to avoid UI clutter
        cg.add(var.set_supported_swing_modes(config[CONF_SUPPORTED_SWING_MODES]))

    setpoint_modes = (
        (climate.ClimateMode.CLIMATE_MODE_HEAT_COOL, config[CONF_HEAT_COOL_MODE]),
        (climate.ClimateMode.CLIMATE_MODE_COOL, config[CONF_COOL_MODE]),
        (climate.ClimateMode.CLIMATE_MODE_HEAT, config[CONF_HEAT_MODE]),
    )
    for mode, cfg in setpoint_modes:
        cg.add(var.set_setpoint_mode_config(mode, cfg[CONF_OFFSET], cfg[CONF_MIN_TEMPERATURE], cfg[CONF_MAX_TEMPERATURE]))
