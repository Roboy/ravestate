# Ravestate static implementations for registering modules


from ravestate import module
import importlib

from reggol import get_logger
logger = get_logger(__name__)

_registered_modules = dict()
_registration_callback = None


def import_module(*, module_name: str, callback):
    """
    Called by context to import a particular ravestate python module.
    :param module_name: The name of the python module to be imported (must be in pythonpath).
    :param callback: A callback which should be called when a module calls register() while it is being imported.
    """
    global _registration_callback
    _registration_callback = callback
    importlib.import_module(module_name)
    _registration_callback = None


def register(*, name: str="", props=(), states=(), config=None):
    """
    May be called to register a named set of states, properties and config entries,
    which form a coherent bundle.
    :param name: The name of the module. Will be prefixed to property and signal names like
     <modulename>:<proeprty/signal-name>.
    :param props: The properties that should be registered.
    :param states: The states that should be registered.
    :param config: A dictionary of config entries and their default values, which should be read
     from the default/user config files.
    :return:
    """
    global _registered_modules
    global _registration_callback

    if name in _registered_modules:
        logger.error(f"Attempt to add module {name} twice!")
        return

    if not config:
        config = {}

    _registered_modules[name] = module.Module(name=name, props=props, states=states, config=config)
    if _registration_callback:
        _registration_callback(_registered_modules[name])


def has_module(module_name: str):
    """
    Check whether a module with a particular name has been registered.
    :param module_name: The name which should be checked for beign registered.
    :return: True if a module with the given name has been registered, false otherwise.
    """
    global _registered_modules
    return module_name in _registered_modules


def get_module(module_name: str):
    """
    Get a registered module with a particular name
    :param module_name: The name of the moduke which should be retrieved.
    :return: The module with the given name if it was registered, false if otherwise.
    """
    global _registered_modules
    return _registered_modules[module_name] if module_name in _registered_modules else None
