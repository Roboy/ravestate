
from dialogic import registry
from dialogic.property import PropertyBase


registry.register(
    name="rawio",
    props=(PropertyBase(name="in", default=""), PropertyBase(name="out", default=""))
)
