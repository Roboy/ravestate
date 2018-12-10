
from ravestate import registry
from ravestate.property import PropertyBase


registry.register(
    name="rawio",
    props=(PropertyBase(name="in", default_value=""), PropertyBase(name="out", default_value=""))
)
