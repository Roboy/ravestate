# Ravestate wrapper classes which limit a state's context access

from ravestate.property import PropertyBase
from ravestate import state
from ravestate import icontext
from ravestate.spike import Spike
from typing import Any, Generator, Set, Dict

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
    def __init__(self, *,
                 spike_parents: Set[Spike] = None,
                 prop: PropertyBase,
                 ctx: icontext.IContext,
                 allow_read: bool,
                 allow_write: bool):
        self.prop = prop
        self.ctx = ctx
        self.allow_read = allow_read and prop.allow_read
        self.allow_write = allow_write and (prop.allow_write | prop.allow_push | prop.allow_pop)
        self.frozen_value = None
        self.spike_parents = spike_parents

        self.prop.lock()
        if self.allow_read:
            self.frozen_value = prop.value
        if not self.allow_write:
            self.prop.unlock()

    def __del__(self):
        if self.allow_write:
            self.prop.unlock()

    def get(self) -> Any:
        """
        Read the current property value or the value of children of the property if child-param is given

        * `child`: top-down list of child ancestry of the child to get the value from
        """
        if not self.allow_read:
            logger.error(f"Unauthorized read access in property-wrapper for {self.prop.id()}!")
            return None
        elif self.allow_write:
            return self.prop.read()
        return self.frozen_value

    def set(self, value: Any):
        """
        Write a new value to the property.

        * `value`: The new value.

        **Returns:** True if the value has changed and :changed should be signaled, false otherwise.
        """
        if not self.allow_write:
            logger.error(f"Unauthorized write access in property-wrapper {self.prop.id()}!")
            return False
        if self.prop.write(value):
            # emit flag signals if it is a flag property
            if self.prop.is_flag_property and value is True:
                # wipe false signal, emit true signal
                self.ctx.wipe(self.prop.flag_false_signal())
                self.ctx.emit(
                    self.prop.flag_true_signal(),
                    parents=self.spike_parents,
                    wipe=self.prop.wipe_on_changed)
            if self.prop.is_flag_property and value is False:
                # wipe true signal, emit false signal
                self.ctx.wipe(self.prop.flag_true_signal())
                self.ctx.emit(
                    self.prop.flag_false_signal(),
                    parents=self.spike_parents,
                    wipe=self.prop.wipe_on_changed)

            self.ctx.emit(
                self.prop.changed_signal(),
                parents=self.spike_parents,
                wipe=self.prop.wipe_on_changed,
                payload=value)
            return True
        return False

    def push(self, child: PropertyBase):
        """
        Add a child to the property or to children of the property

        * `child`: Parent-less, child-less property object to add.
         Name of the child must be unique among existing children of this property.

        **Returns:** True if the push was successful, False otherwise
        """
        if not self.allow_write:
            logger.error(f"Unauthorized push access in property-wrapper {self.prop.id()}!")
            return False
        if self.prop.push(child):
            self.ctx.emit(
                self.prop.pushed_signal(),
                parents=self.spike_parents,
                wipe=False,
                payload=child.id())
            return True
        return False

    def pop(self, childname: str):
        """
        Remove a child from the property or from children of the property

        * `childname`: Name of the direct child to be removed

        **Returns:** True if the pop was successful, False otherwise
        """
        if not self.allow_write:
            logger.error(f"Unauthorized pop access in property-wrapper {self.prop.id()}!")
            return False
        if self.prop.pop(childname):
            self.ctx.emit(
                self.prop.popped_signal(),
                parents=self.spike_parents,
                wipe=False,
                payload=f"{self.prop.id()}:{childname}")
            return True
        return False

    def enum(self) -> Generator[str, None, None]:
        """
        Get the full paths of each of this property's children.
        """
        if not self.allow_read:
            logger.error(f"Unauthorized read access in property-wrapper for {self.prop.id()}!")
            return (_ for _ in ())
        return (child.id() for _, child in self.prop.children.items())


class ContextWrapper:
    """
    Encapsulates a context towards a state, only offering properties with permissions
    as declared by the state beforehand.
    """

    def __init__(self, *, ctx: icontext.IContext, st: state.State, spike_parents: Set[Spike]=None, spike_payloads: Dict[str, Any]=None):
        self.st = st
        self.ctx = ctx
        self.properties = dict()
        self.spike_parents = spike_parents
        self.spike_payloads = spike_payloads
        # Recursively complete properties dict with children:
        for propname in st.write_props + st.read_props:
            # May have been covered by a parent before
            if propname not in self.properties:
                prop_and_children = ctx[propname].gather_children()
                for prop in prop_and_children:
                    # Child may have been covered by a parent before
                    if prop.id() not in self.properties:
                        self.properties[prop.id()] = PropertyWrapper(
                            prop=prop, ctx=ctx,
                            spike_parents=self.spike_parents,
                            allow_read=propname in st.read_props,
                            allow_write=propname in st.write_props)

    def __setitem__(self, key, value):
        if key in self.properties:
            return self.properties[key].set(value)
        else:
            logger.error(f"State {self.st.name} attempted to write property {key} without permission!")

    def __getitem__(self, key) -> Any:
        if key in self.properties:
            return self.properties[key].get()
        elif key in self.spike_payloads:
            return self.spike_payloads[key]
        else:
            logger.error(f"State {self.st.name} attempted to access property {key} without permission!")

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

    def push(self, parentpath: str, child: PropertyBase):
        """
        Add a child to a property.
         Note: Child must not yet have a parent or children of itself.
          Write-access to parent is needed.

        * `parentpath`: Path of the parent that should receive the new child.

        * `child`: Parent-less, child-less property object to add.

        **Returns:** True if the push was successful, False otherwise
        """
        if child.parent_path:
            logger.error(f"State {self.st.name} attempted to push child property {child.name} to parent {parentpath}, but it already has parent {child.parent_path}!")
            return False
        if parentpath in self.properties:
            if self.properties[parentpath].push(child):
                self.properties[child.id()] = PropertyWrapper(
                    prop=child, ctx=self.ctx,
                    spike_parents=self.spike_parents,
                    allow_read=self.properties[parentpath].allow_read,
                    allow_write=self.properties[parentpath].allow_write)
                self.ctx.add_prop(prop=child)
                return True
        else:
            logger.error(f'State {self.st.name} attempted to add child-property {child.name} to non-accessible parent {parentpath}!')
            return False

    def pop(self, path: str):
        """
        Delete a property (remove it from context and it's parent).
         Note: Write-access to parent is needed!

        * `path`: Path to the property. Must be nested (not root-level)!

        **Returns:** True if the pop was successful, False otherwise
        """
        path_parts = path.split(":")
        if len(path_parts) < 3:
            logger.error(f"State {self.st.name}: Path to pop is not a nested property: {path}")
            return False
        parentpath = ":".join(path_parts[:-1])
        if parentpath in self.properties:
            if self.properties[parentpath].pop(path_parts[-1]):
                self.ctx.rm_prop(prop=self.properties[path].prop)
                # Remove property from own dict
                del self.properties[path]
                # Also remove the deleted propertie's children
                for childpath in list(self.properties.keys()):
                    if childpath.startswith(path+":"):
                        self.ctx.rm_prop(prop=self.properties[childpath].prop)
                        del self.properties[childpath]
                return True
            else:
                logger.error(f'State {self.st.name} attempted to remove non-existent child-property {path}')
                return False
        else:
            logger.error(f'State {self.st.name} attempted to remove child-property {path} from non-existent parent-property {parentpath}')
            return False

    def enum(self, path) -> Generator[str, None, None]:
        """
        Enumerate a propertie's children by their full pathes.
        """
        if path in self.properties:
            return self.properties[path].enum()
        else:
            logger.error(f"State {self.st.name} attempted to enumerate property {path} without permission!")

