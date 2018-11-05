# Ravestate wrapper classes which limit a state's context access

from ravestate import property
from ravestate import state
from ravestate import icontext

from ravestate import registry


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

    def get(self):
        """
        Read the current property value.
        """
        if not self.allow_read:
            print(f"Unauthorized read access in property-wrapper for {self.prop.name}!")
            return None
        elif self.allow_write:
            return self.prop.read()
        return self.frozen_value

    def set(self, value):
        """
        Write a new value to the property.
        :param value: The new value.
        :return: True if the value has changed and :changed should be signaled, false otherwise.
        """
        if not self.allow_write:
            print(f"Unauthorized write access in property-wrapper {self.prop.name}!")
            return False
        if self.prop.write(value):
            self.ctx.emit(f"{self.prop.fullname()}:changed")


class ContextWrapper:
    """
    Encapsulates a context towards a state, only offering properties with permissions
    as declared by the state beforehand.
    """

    DoNothing = 0
    EmitSignal = 1
    DeleteMe = 2

    def __init__(self, ctx: icontext.IContext, st: state.State):
        self.st = st
        self.ctx = ctx
        self.properties = {
            propname : PropertyWrapper(
                prop=ctx[propname], ctx=ctx,
                allow_read=propname in st.read_props,
                allow_write=propname in st.write_props)
            for propname in st.write_props+st.read_props
        }

    def __setitem__(self, key, value):
        if key in self.properties:
            self.properties[key].set(value)
        else:
            print(f"State {self.st.name} attempted to write property {key} without permission!")

    def __getitem__(self, key):
        if key in self.properties:
            return self.properties[key].get()
        else:
            print(f"State {self.st.name}` attempted to access property {key} without permission!")

    def add_state(self, st: state.State):
        mod = registry.get_module(self.st.module_name)
        self.ctx.add_state(mod=mod, st=st)

    def shutdown(self):
        self.ctx.shutdown()

    def shutting_down(self):
        return self.ctx.shutting_down()


if __name__ == '__main__':
    prop = property.PropertyBase(name="foo", default="Kruder")
    prop.module_name = "poo"
    assert(prop.fullname() == "poo:foo")
    assert(not prop._lock.locked())
    assert(prop.read() == "Kruder")

    # Make sure that writing to read-only wrapper is ineffective
    wrap = PropertyWrapper(prop=prop, ctx=icontext.IContext(), allow_read=True, allow_write=False)
    assert(not prop._lock.locked())
    wrap.set("Dorfmeister")
    assert(wrap.get() == "Kruder")

    # Make sure that writing to writable wrapper is effective
    wrap = PropertyWrapper(prop=prop, ctx=icontext.IContext(), allow_read=True, allow_write=True)
    assert(prop._lock.locked())
    wrap.set("Dorfmeister")
    assert(wrap.get() == "Dorfmeister")
