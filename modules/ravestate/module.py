# Ravestate module class

from typing import Dict, Any, Union, Iterable, Callable
import importlib
from ravestate.property import PropertyBase
from ravestate.state import State
from ravestate.threadlocal import ravestate_thread_local

from reggol import get_logger
logger = get_logger(__name__)


class Module:
    """
    Atomic class, which encapsulates a named set of states, properties and config entries,
    which form a coherent bundle.

    _Example:_
    ```python
    with Module(name="my_module", config={"paramA": 42}):
        # define properties
        # define states
    ```
    """

    registered_modules: Dict[str, 'Module'] = dict()
    registration_callback: Callable[['Module'], Any] = None  # set by import_module

    def __init__(self, *, name: str, config: Dict[str, Any]=None):
        """
        Create a new module with a name and certain config entries.

        * `name`: The name of the module. Will be prefixed to property and signal names like
         <module name>:<property/signal-name>. Should be unique among all modules
         currently existing in the process.

        * `config`: A dictionary of config entries and their default values, which should be read
         from the default/user config files.
        """
        if not config:
            config = {}
        self.props = []
        self.states = []
        self.name = name
        self.conf = config
        if name in self.registered_modules:
            logger.error(f"Adding module {name} twice!")
        self.registered_modules[name] = self

    def __enter__(self):
        mod = getattr(ravestate_thread_local, 'module_under_construction', None)
        if mod:
            logger.error("Nested `with Module(...)` calls are not supported!`")
        else:
            ravestate_thread_local.module_under_construction = self
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        ravestate_thread_local.module_under_construction = None
        if self.registration_callback:
            self.registration_callback(self)

    def add(self, property_or_state: Union[PropertyBase, State, Iterable[PropertyBase], Iterable[State]]):
        try:
            for obj_to_add in property_or_state:
                self.add(obj_to_add)
        except TypeError:
            pass
        if isinstance(property_or_state, PropertyBase):
            property_or_state.set_parent_path(self.name)
            self.props.append(property_or_state)
        elif isinstance(property_or_state, State):
            property_or_state.module_name = self.name
            self.states.append(property_or_state)
        else:
            logger.error(f"Module.add() called with invalid argument {property_or_state}!")


def import_module(*, module_name: str, callback):
    """
    Called by context to import a particular ravestate python module.

    * `module_name`: The name of the python module to be imported (must be in pythonpath).

    * `callback`: A callback which should be called when a module calls register() while it is being imported.
    """
    assert not Module.registration_callback
    Module.registration_callback = callback
    importlib.import_module(module_name)
    Module.registration_callback = None


def has_module(module_name: str):
    """
    Check whether a module with a particular name has been registered.

    * `module_name`: The name which should be checked for beign registered.

    **Returns:** True if a module with the given name has been registered, false otherwise.
    """
    return module_name in Module.registered_modules


def get_module(module_name: str):
    """
    Get a registered module with a particular name

    * `module_name`: The name of the moduke which should be retrieved.

    **Returns:** The module with the given name if it was registered, false if otherwise.
    """
    return Module.registered_modules[module_name] if module_name in Module.registered_modules else None
