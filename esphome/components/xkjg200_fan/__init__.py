import esphome.codegen as cg
from esphome.components import fan
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_SPEED_COUNT

CODEOWNERS = ["@julen"]
DEPENDENCIES = []
AUTO_LOAD = ["fan"]
MULTI_CONF = False

xkjg200_fan_ns = cg.esphome_ns.namespace("xkjg200_fan")
XKJG200FanController = xkjg200_fan_ns.class_("XKJG200FanController", cg.Component, fan.Fan)

CONFIG_SCHEMA = fan.fan_schema(XKJG200FanController).extend({
    cv.Optional(CONF_SPEED_COUNT, default=3): cv.int_range(min=1, max=3),
}).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await fan.register_fan(var, config)
    cg.add(var.set_speed_count(config[CONF_SPEED_COUNT]))
