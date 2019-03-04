# Ravestate property classes

from threading import Lock
from typing import Dict, List, Generator
from ravestate.constraint import s, Signal
from ravestate.threadlocal import ravestate_thread_local

from reggol import get_logger
logger = get_logger(__name__)


def changed(property_name, **kwargs) -> Signal:
    """
    Returns the `changed` Signal for the given property.
    This signal is emitted, when the Property is written to,
     and the new property value is different from the old one,
     or the propertie's `always_signal_changed` flag is True.<br>
    __Hint:__ All key-word arguments of #constraint.s(...)
     (`min_age`, `max_age`, `detached`) are supported.
    """
    return s(f"{property_name}:changed", **kwargs)


def pushed(property_name, **kwargs) -> Signal:
    """
    Returns the `pushed` Signal for the given property. This signal
     is emitted, when a new child property is added to it.
     From the perspective of a state, this can be achieved
     with the `ContextWrapper.push(...)` function.<br>
    __Hint:__ All key-word arguments of #constraint.s(...)
     (`min_age`, `max_age`, `detached`) are supported.
    """
    return s(f"{property_name}:pushed", **kwargs)


def popped(property_name, **kwargs) -> Signal:
    """
    Returns the `popped` Signal for the given property. This signal
     is emitted, when a child property removed from it.
     From the perspective of a state, this can be achieved
     with the `ContextWrapper.pop(...)` function.<br>
    __Hint:__ All key-word arguments of #constraint.s(...)
     (`min_age`, `max_age`, `detached`) are supported.
    """
    return s(f"{property_name}:popped", **kwargs)


class PropertyBase:
    """
    Base class for context properties. Controls read/write/push/pop/delete permissions,
    property name basic impls. for the property value, parent/child mechanism.

    _Example (Creating a module containing a property named my_property):_
    ```python
    with Module(name="my_module"):
        my_property = PropertyBase(name="my_property")
    ```
    """

    def __init__(
            self, *,
            name="",
            allow_read=True,
            allow_write=True,
            allow_push=True,
            allow_pop=True,
            default_value=None,
            always_signal_changed=False,
            is_flag_property=False,
            wipe_on_changed=True):

        self.name = name
        self.allow_read = allow_read
        self.allow_write = allow_write
        self.allow_push = allow_push
        self.allow_pop = allow_pop
        self.value = default_value
        self.children: Dict[str, PropertyBase] = dict()
        self._lock = Lock()
        self.parent_path: str = ""
        self.always_signal_changed = always_signal_changed
        self.is_flag_property = is_flag_property
        self.wipe_on_changed = wipe_on_changed

        # add property to module in current `with Module(...)` clause
        module_under_construction = getattr(ravestate_thread_local, 'module_under_construction', None)
        if module_under_construction:
            module_under_construction.add(self)

    def id(self):
        return f'{self.parent_path}:{self.name}'

    def set_parent_path(self, path):
        """
        Set the ancestors (including modulename) for a property

        * `path`: ancestry in the form of modulename:parent_prop_name (or simply modulename)
        """
        if not self.parent_path:
            self.parent_path = path
        else:
            logger.error(f'Tried to override parent_path of {self.id()}')

    def gather_children(self) -> List['PropertyBase']:
        """
        Collect this property, and all of it's children.
        """
        result = [self]
        for child in self.children.values():
            result += child.gather_children()
        return result

    def lock(self):
        self._lock.acquire()

    def unlock(self):
        self._lock.release()

    def read(self):
        """
        Read the current property value
        """
        if not self.allow_read:
            logger.error(f"Unauthorized read access in property {self.id()}!")
            return None
        return self.value

    def write(self, value):
        """
        Write a new value to the property

        * `value`: The new value.

        **Returns:** True if the value has changed and :changed should be signaled, false otherwise.
        """
        if not self.allow_write:
            logger.error(f"Unauthorized write access in property {self.id()}!")
            return False
        if self.always_signal_changed or self.value != value:
            self.value = value
            return True
        else:
            return False

    def push(self, child: 'PropertyBase'):
        """
        Add a child to the property

        * `child`: The child object

        **Returns:** True if the child was added successfully, false otherwise.
        """
        if not self.allow_push:
            logger.error(f"Unauthorized push in property {self.id()}!")
            return False
        if child.name in self.children:
            logger.error(f"Tried to add already existing child-property {self.id()}:{child.name}")
            return False
        child.set_parent_path(self.id())
        self.children[child.name] = child
        return True

    def pop(self, child_name: str):
        """
        Remove a child from the property by it's name.

        * `child_name`: Name of the child to be removed.

        **Returns:** True if the pop was successful, False otherwise
        """
        if not self.allow_pop:
            logger.error(f"Unauthorized pop in property {self.id()}!")
            return False
        elif child_name in self.children:
            self.children.pop(child_name)
            return True
        else:
            logger.error(f"Tried to remove non-existent child-property {self.id()}:{child_name}")
            return False

    def changed_signal(self) -> Signal:
        """
        Signal that is emitted by PropertyWrapper when #write() returns True.
        """
        return changed(self.id())

    def pushed_signal(self) -> Signal:
        """
        Signal that is emitted by PropertyWrapper when #push() returns True.
        """
        return pushed(self.id())

    def popped_signal(self) -> Signal:
        """
        Signal that is emitted by PropertyWrapper when #pop() returns True.
        """
        return popped(self.id())

    def flag_true_signal(self) -> Signal:
        """
        Signal that is emitted by PropertyWrapper when it is a flag-property and #self.value is set to True.
        """
        return s(f"{self.id()}:true")

    def flag_false_signal(self) -> Signal:
        """
        Signal that is emitted by PropertyWrapper when it is a flag-property and #self.value is set to False.
        """
        return s(f"{self.id()}:false")

    def signals(self) -> Generator[Signal, None, None]:
        """
        Yields all signals that may be emitted because of
         this property, given it's write/push/pop permissions.
        """
        if self.allow_write:
            yield self.changed_signal()
        if self.allow_push:
            yield self.pushed_signal()
        if self.allow_pop:
            yield self.popped_signal()
        if self.is_flag_property:
            yield self.flag_true_signal()
            yield self.flag_false_signal()

