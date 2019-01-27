
from ravestate import registry
from ravestate.property import PropertyBase

_p_in = PropertyBase(name="in", default_value="", allow_pop=False, allow_push=False)
_p_out = PropertyBase(name="out", default_value="", allow_pop=False, allow_push=False)

registry.register(name="rawio", props=(_p_in, _p_out))

p_in = _p_in.fullname()
p_out = _p_out.fullname()
