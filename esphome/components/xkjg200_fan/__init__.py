import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@julen"]
DEPENDENCIES = []
AUTO_LOAD = []
MULTI_CONF = False

xkjg200_fan_ns = cg.esphome_ns.namespace("xkjg200_fan")
XKJG200FanController = xkjg200_fan_ns.class_("XKJG200FanController", cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(XKJG200FanController),
}).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
