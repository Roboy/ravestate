# Ravestate module class

from typing import Dict, Any, Union, Iterable, Set
import inspect
from collections import defaultdict

from ravestate.constraint import Signal
from ravestate.property import Property
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

    modules_per_python_module: Dict[str, Set['Module']] = defaultdict(set)
    registered_modules: Dict[str, 'Module'] = dict()

    def __init__(self, *, name: str, config: Dict[str, Any]=None, depends: Iterable['Module']=None):
        """
        Create a new module with a name and certain config entries.

        * `name`: The name of the module. Will be prefixed to property and signal names like
         <module name>:<property/signal-name>. Should be unique among all modules
         currently existing in the process.

        * `config`: A dictionary of config entries and their default values, which should be read
         from the default/user config files.

        * `depends`: Collection of modules which must also be added to a context which
         includes this module. __Note:__ A core module dependency (referring to core module objects
        such as `sig_startup`, `prop_intent` etc.) may safely be omitted.
        """
        if not config:
            config = {}
        if not depends:
            depends = []
        self.props = []
        self.states = []
        self.signals = []
        self.name = name
        self.conf = config
        self.depends = depends
        if name in self.registered_modules:
            logger.warn(f"Redefinition of module `{name}`!")
        module = inspect.getmodule(inspect.stack()[1].frame)
        if module and module.__spec__:
            self.module_name = module.__spec__.name.split(".")[0]
            self.modules_per_python_module[self.module_name].add(self)
            logger.info(f"Registered module `{self.name}` under python module `{self.module_name}`.")
        else:
            self.module_name = "unknown"
            logger.warn(f"Could not determine python module for `{self.name}`!")
        self.registered_modules[name] = self

    def __enter__(self):
        mod = getattr(ravestate_thread_local, 'module_under_construction', None)
        if mod:
            raise RuntimeError("Nested `with Module(...)` calls are not supported!`")
        else:
            ravestate_thread_local.module_under_construction = self
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        ravestate_thread_local.module_under_construction = None

    def add(self, ownable: Union[Property, State, Signal, Iterable[Property], Iterable[State], Iterable[Signal]]):
        try:
            for obj_to_add in ownable:
                self.add(obj_to_add)
            return
        except TypeError:
            pass
        if isinstance(ownable, Property):
            ownable.set_parent_path(self.name)
            self.props.append(ownable)
        elif isinstance(ownable, State):
            ownable.module_name = self.name
            self.states.append(ownable)
        elif isinstance(ownable, Signal):
            ownable.parent_path = self.name
            self.signals.append(ownable)
        else:
            logger.error(f"Module.add() called with invalid argument {ownable}!")


def has_module(module_name: str) -> bool:
    """
    Check whether a module with a particular name has been registered.

    * `module_name`: The name which should be checked for beign registered.

    **Returns:** True if a module with the given name has been registered, false otherwise.
    """
    return module_name in Module.registered_modules or len(Module.modules_per_python_module[module_name])


def get_module(module_name: str) -> Set[Module]:
    """
    Get modules registered with a particular name.

    **Note:** If the module_name refers to a Python module,
     multiple Ravestate modules might be returned.

    * `module_name`: Python or Ravestate module name.

    **Returns:** The modules registered under the given name, empty set otherwise.
    """
    if module_name in Module.registered_modules:
        return {Module.registered_modules[module_name]}
    elif module_name in Module.modules_per_python_module:
        return Module.modules_per_python_module[module_name]
    else:
        return set()
