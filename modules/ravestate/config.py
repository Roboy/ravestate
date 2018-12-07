# Ravestate class which represents a cluster of module configurations
# for a single context instance.

from ravestate import module
from collections import defaultdict
from typing import List, Any

import yaml
from yamlinclude import YamlIncludeConstructor

YamlIncludeConstructor.add_to_loader_class(yaml.SafeLoader)

from reggol import get_logger
logger = get_logger(__name__)


class Configuration:
    """
    The Configuration class maintains a dictionary of key-value stores, which
    represent configuration entries for specific named modules. The key-value
    stores may be successively updated with consecutive yaml files, where each
    yaml document has the following content:

      ---
      module: module-name
      config:
        key-a: value-a
        key-b: value-b
        # etc
      ---
    """

    def __init__(self, paths: List[str]):
        """
        Constructor for a Configuration instance.
        :param paths: Yaml file paths, where each file contains zero or more
        yaml documents according to the layout described above.
        """
        self.parsed_config_per_module = defaultdict(lambda: {})
        self.config_per_module = {}
        for path in paths:
            self.read(path)

    def add_conf(self, mod: module.Module):
        """
        Register a set of allowed config entries for a specific module.
        Correctly typed values for allowed keys, that were previously
        parsed during construction from the yaml files, will be applied
        immediately.
        :param mod: A module object with a name and a conf dict.
        """
        if mod.name in self.config_per_module:
            logger.warning(f"add_conf called repeatedly for module name f{mod.name}!")
        self.config_per_module[mod.name] = mod.conf
        self._apply_parsed_config(mod.name)

    def get_conf(self, module_name: str):
        """
        Retrieve updated config values for a module that was previously
        registered with add_conf.
        :param module_name: The module name for which configuration should be retrieved.
        :return: A dictionary which contains exactly the keys that were contained in
        the module configuration dictionary during add_conf, or an empty dictionary
        if the module name is unknown.
        """
        if module_name not in self.parsed_config_per_module:
            logger.error(f"Bad request for unknown module config {module_name}!")
            return {}
        return self.config_per_module[module_name]

    def get(self, module_name: str, key: str) -> Any:
        """
        Gte the current value of a config entry.
        :param module_name: The module that provides the config entry.
        :param key: A config key for the module that was previously added through add_conf.
        :return: The current value, or None, if the entry does not exist.
        """
        if module_name not in self.config_per_module:
            logger.error(f"Attempt to run get() for unknown modname {module_name}!")
            return None
        target_conf = self.config_per_module[module_name]
        if key not in target_conf:
            logger.warning(f"Cannot read unknown conf key {key} for module {module_name}.")
            return None
        return target_conf[key]

    def set(self, module_name: str, key: str, value: Any):
        """
        Set the current value of a config entry.
        :param module_name: The module of the config entry.
        :param key: A config key for the module that was previously added through add_conf.
        :param value: The new value for the config entry. An error will be raised,
         if the type of the new value does not match the type of the old value.
        """
        if module_name not in self.config_per_module:
            logger.error(f"Attempt to run set() for unknown modname {module_name}!")
            return
        target_conf = self.config_per_module[module_name]
        if key not in target_conf:
            logger.warning(f"Not setting unknown conf key {key} for module {module_name}.")
            return
        current_value = target_conf[key]
        if isinstance(current_value, list):
            if not isinstance(value, list):
                value = [value]
            if len(current_value) and len(value) and not isinstance(value[0], type(current_value[0])):
                value = [type(current_value[0])(item) for item in value]
        if not isinstance(value, type(current_value)):
            logger.warning(
                f"Config entry for {module_name}.{key} has conflicting type {type(value).__name__} " +
                f"(should be {type(current_value).__name__}).")
        target_conf[key] = value

    def write(self, path: str):
        """
        Write all current config entries to a yaml file.
        :param path: The file path to write. Will be overwritten!
        """
        with open(path, mode='w') as file:
            yaml.safe_dump_all((
                {"module": module_name, "config": config_entries}
                for module_name, config_entries in self.config_per_module),
                file,
                default_flow_style=False)

    def read(self, path: str):
        """
        Loads all documents from a yaml file and tries to interpret them
        as configuration objects as described above.
        :param path: The yaml file path from which to load config documents.
        """
        with open(path, mode='r') as file:
            try:
                configs = yaml.safe_load_all(file)
            except Exception as e:
                logger.warning(f"Could not load config file {path}")
                return
            for config in configs:
                if not isinstance(config, dict) or "module" not in config or "config" not in config:
                    logger.warning(f"Skipping invalid entry for config file {path}.")
                    continue
                module_name = config["module"]
                conf = config["config"]
                if not isinstance(conf, dict):
                    logger.warning(f"Skipping invalid config for {module_name} in {path}.")
                    continue
                target_conf = self.parsed_config_per_module[module_name]
                target_conf.update(conf)

    def _apply_parsed_config(self, module_name: str):
        """
        Apply loaded config values to the actual configuration of a specific module.
        Will be called when add_conf is called with a particular module.
        :param module_name:
        :return:
        """
        if module_name not in self.parsed_config_per_module:
            return
        if module_name not in self.config_per_module:
            logger.error(f"Attempt to run _apply_parsed_config for unknown modname {module_name}!")
            return

        config = self.parsed_config_per_module[module_name]

        for key, value in config.items():
            self.set(module_name, key, value)

