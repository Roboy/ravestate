
from ravestate import registry
from ravestate.property import PropertyBase


registry.register(
    name="rawio",
    props=(PropertyBase(name="in", default=""), PropertyBase(name="out", default=""))
)
