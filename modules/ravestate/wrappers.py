# Ravestate wrapper classes which limit a state's context access

from ravestate import property
from ravestate import state
from ravestate import icontext
from typing import Any, List

from ravestate.constraint import s

from reggol import get_logger
logger = get_logger(__name__)


class PropertyWrapper:
    """
    Encapsulates a property, and annotates it with additional r/w perms and a context.
    The context is used to trigger the proper :changed, :pushed, :popped, :deleted
    signals when the property is accessed. The wrapper also takes care of locking the property
    when it is supposed to be written to, and freezing the property's value if it is supposed to
    be read from.
    """
    def __init__(self, *, prop: property.PropertyBase, ctx: icontext.IContext, allow_read, allow_write):
        self.prop = prop
        self.ctx = ctx
        self.allow_read = allow_read and prop.allow_read
        self.allow_write = allow_write and prop.allow_write
        self.frozen_value = None

        self.prop.lock()
        if self.allow_read:
            self.frozen_value = prop.value
        if not self.allow_write:
            self.prop.unlock()

    def __del__(self):
        if self.allow_write:
            self.prop.unlock()

    def get(self, child: List[str] = None):
        """
        Read the current property value.
        """
        if not self.allow_read:
            logger.error(f"Unauthorized read access in property-wrapper for {self.prop.name}!")
            return None
        elif self.allow_write or child:  # TODO frozen_value for children?
            return self.prop.read(child=child)
        return self.frozen_value

    def set(self, value, child: List[str] = None):
        """
        Write a new value to the property.
        :param value: The new value.
        :return: True if the value has changed and :changed should be signaled, false otherwise.
        """
        if not self.allow_write:
            logger.error(f"Unauthorized write access in property-wrapper {self.prop.name}!")
            return False
        if self.prop.write(value, child=child):
            childname = ':' + ':'.join(child) if child else ''
            self.ctx.emit(s(f"{self.prop.fullname()}{childname}:changed"))
            return True
        return False

    def push(self, child: List[str], always_signal_changed: bool = False, default=None):
        return self.prop.push(child, always_signal_changed, default)

    def pop(self, child: List[str]):
        return self.prop.pop(child)


class ContextWrapper:
    """
    Encapsulates a context towards a state, only offering properties with permissions
    as declared by the state beforehand.
    """

    def __init__(self, ctx: icontext.IContext, st: state.State):
        self.st = st
        self.ctx = ctx
        self.properties = {
            propname : PropertyWrapper(
                prop=ctx.get_prop(propname), ctx=ctx,
                allow_read=propname in st.read_props,
                allow_write=propname in st.write_props)
            for propname in st.write_props+st.read_props
        }

    def __setitem__(self, key, value):
        keyList = key.split(':')
        if len(keyList) == 1 and keyList[0] in self.properties:
            return self.properties[keyList[0]].set(value)
        elif keyList[0] in self.properties:
            return self.properties[keyList[0]].set(value, child=keyList[1:])
        else:
            logger.error(f"State {self.st.name} attempted to write property {key} without permission!")

    def __getitem__(self, key) -> Any:
        keyList = key.split(':')
        if len(keyList) == 1 and keyList[0] in self.properties:
            return self.properties[keyList[0]].get()
        elif keyList[0] in self.properties:
            return self.properties[keyList[0]].get(child=keyList[1:])
        else:
            logger.error(f"State {self.st.name}` attempted to access property {key} without permission!")

    def add_state(self, st: state.State):
        self.ctx.add_state(st=st)

    def shutdown(self):
        self.ctx.shutdown()

    def shutting_down(self):
        return self.ctx.shutting_down()

    def conf(self, *, mod=None, key=None):
        if not mod:
            mod = self.st.module_name
        return self.ctx.conf(mod=mod, key=key)

    def push(self, childpath: str, always_signal_changed: bool = False, default=None):
        """

        :param childpath: Path of the child-property in the form of parent-path:child-name
        """
        try:
            parent, children = childpath.split(':', 1)
            if not children:
                raise ValueError
        except ValueError:
            logger.error(f'Could not add child-property because the format of {childpath} is incorrect')
            return False
        if parent in self.properties:
            childrenList: List[str] = children.split(':')
            self.properties[parent].push(childrenList, always_signal_changed, default)
        else:
            logger.error(f'Attempted to add child-property {children} to non-existent parent-property {parent}')
            return False

    def pop(self, childpath: str):
        try:
            parent, children = childpath.split(':', 1)
        except ValueError:
            logger.error(f'Could not remove child-property because the format of {childpath} is incorrect')
            return
        if parent in self.properties:
            childrenList: List[str] = children.split(':')
            self.properties[parent].pop(childrenList)
        else:
            logger.error(f'Attempted to remove child-property {children} from non-existent parent-property {parent}')
