"""
Daikin S21 Mini-Split ESPHome climate component config validation & code generation.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, sensor
from esphome.components.climate import ClimateMode, ClimatePreset
from esphome.const import (
    CONF_HUMIDITY_SENSOR,
    CONF_SENSOR,
    CONF_SUPPORTED_MODES,
    CONF_SUPPORTED_PRESETS,
)
from .. import (
    daikin_s21_ns,
    CONF_S21_ID,
    S21_PARENT_SCHEMA,
)

DaikinS21Climate = daikin_s21_ns.class_(
    "DaikinS21Climate", climate.Climate, cg.PollingComponent
)

SUPPORTED_CLIMATE_MODES_OPTIONS = {
    "OFF": ClimateMode.CLIMATE_MODE_OFF,  # always available
    "HEAT_COOL": ClimateMode.CLIMATE_MODE_HEAT_COOL,
    "COOL": ClimateMode.CLIMATE_MODE_COOL,
    "HEAT": ClimateMode.CLIMATE_MODE_HEAT,
    "FAN_ONLY": ClimateMode.CLIMATE_MODE_FAN_ONLY,
    "DRY": ClimateMode.CLIMATE_MODE_DRY,
}

SUPPORTED_CLIMATE_PRESETS_OPTIONS = {
    "ECO": ClimatePreset.CLIMATE_PRESET_ECO,
    "BOOST": ClimatePreset.CLIMATE_PRESET_BOOST,
}

CONF_MAX_COOL_TEMPERATURE = "max_cool_temperature"
CONF_MIN_COOL_TEMPERATURE = "min_cool_temperature"
CONF_MAX_HEAT_TEMPERATURE = "max_heat_temperature"
CONF_MIN_HEAT_TEMPERATURE = "min_heat_temperature"


CONFIG_SCHEMA = cv.All(
    climate.climate_schema(DaikinS21Climate)
    .extend(
        {
            cv.Optional(CONF_SENSOR): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_HUMIDITY_SENSOR): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_SUPPORTED_MODES): cv.ensure_list(cv.enum(SUPPORTED_CLIMATE_MODES_OPTIONS, upper=True)),
            cv.Optional(CONF_SUPPORTED_PRESETS): cv.ensure_list(cv.enum(SUPPORTED_CLIMATE_PRESETS_OPTIONS, upper=True)),
            cv.Optional(CONF_MAX_COOL_TEMPERATURE, default="32"): cv.temperature,
            cv.Optional(CONF_MIN_COOL_TEMPERATURE, default="18"): cv.temperature,
            cv.Optional(CONF_MAX_HEAT_TEMPERATURE, default="30"): cv.temperature,
            cv.Optional(CONF_MIN_HEAT_TEMPERATURE, default="10"): cv.temperature,
        }
    )
    .extend(S21_PARENT_SCHEMA)
    .extend(cv.polling_component_schema("0s"))
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

    if CONF_SUPPORTED_MODES in config:
        cg.add(var.set_supported_modes_override(config[CONF_SUPPORTED_MODES]))

    if CONF_SUPPORTED_PRESETS in config:
        cg.add(var.set_supported_presets_override(config[CONF_SUPPORTED_PRESETS]))

    if CONF_MAX_COOL_TEMPERATURE in config:
        cg.add(var.set_max_cool_temperature(config[CONF_MAX_COOL_TEMPERATURE]))

    if CONF_MIN_COOL_TEMPERATURE in config:
        cg.add(var.set_min_cool_temperature(config[CONF_MIN_COOL_TEMPERATURE]))

    if CONF_MAX_HEAT_TEMPERATURE in config:
        cg.add(var.set_max_heat_temperature(config[CONF_MAX_HEAT_TEMPERATURE]))

    if CONF_MIN_HEAT_TEMPERATURE in config:
        cg.add(var.set_min_heat_temperature(config[CONF_MIN_HEAT_TEMPERATURE]))
