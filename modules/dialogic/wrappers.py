# Dialogic wrapper classes which limit a state's session access

from dialogic import property
from dialogic import state
from dialogic import isession


class PropertyWrapper:
    """
    Encapsulates a property, and annotates it with additional r/w perms and a session.
    The session is used to trigger the proper :changed, :pushed, :popped, :deleted
    signals when the property is accessed. The wrapper also takes care of locking the property
    when it is supposed to be written to, and freezing the property's value if it is supposed to
    be read from.
    """
    def __init__(self, *, prop: property.PropertyBase, sess: isession.ISession, allow_read, allow_write):
        self.prop = prop
        self.sess = sess
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
            print("Unauthorized read access in property-wrapper for {}!".format(self.prop.name))
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
            print("Unauthorized write access in property-wrapper {}!".format(self.prop.name))
            return False
        if self.prop.write(value):
            self.sess.emit("{}:changed".format(self.prop.fullname()))


class SessionWrapper:
    """
    Encapsulates a session towards a state, only offering properties with permissions
    as declared by the state beforehand.
    """
    def __init__(self, sess: isession.ISession, st: state.State):
        self.st = st
        self.properties = {
            propname : PropertyWrapper(
                prop=sess[propname], sess=sess,
                allow_read=propname in st.read_props,
                allow_write=propname in st.write_props)
            for propname in st.write_props+st.read_props
        }

    def __setitem__(self, key, value):
        if key in self.properties:
            self.properties[key].set(value)
        else:
            print("State `{}` attempted to write property `{}` without permission!".format(self.st.name, key))

    def __getitem__(self, key):
        if key in self.properties:
            return self.properties[key]
        else:
            print("State `{}` attempted to access property `{}` without permission!".format(self.st.name, key))


if __name__ == '__main__':
    prop = property.PropertyBase(name="foo", default="Kruder")
    prop.module_name = "poo"
    assert(prop.fullname() == "poo:foo")
    assert(not prop._lock.locked())
    assert(prop.read() == "Kruder")

    # Make sure that writing to read-only wrapper is ineffective
    wrap = PropertyWrapper(prop=prop, sess=isession.ISession(), allow_read=True, allow_write=False)
    assert(not prop._lock.locked())
    wrap.set("Dorfmeister")
    assert(wrap.get() == "Kruder")

    # Make sure that writing to writable wrapper is effective
    wrap = PropertyWrapper(prop=prop, sess=isession.ISession(), allow_read=True, allow_write=True)
    assert(prop._lock.locked())
    wrap.set("Dorfmeister")
    assert(wrap.get() == "Dorfmeister")
