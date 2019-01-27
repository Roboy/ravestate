from ravestate.property import PropertyBase
from ravestate import registry

_p_all = PropertyBase(name="all", allow_read=True, allow_write=False, allow_push=True, allow_pop=True)

# TODO: Make interloc:all a special property type, that only accepts ScientioNodeProperty as children
registry.register(name="interloc", props=_p_all)

p_all = _p_all.fullname()
