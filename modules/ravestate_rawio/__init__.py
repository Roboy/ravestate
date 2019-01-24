
from ravestate import registry
from ravestate.property import PropertyBase


registry.register(
    name="rawio",
    props=(
        PropertyBase(name="in", default_value="", allow_pop=False, allow_push=False, always_signal_changed=True),
        PropertyBase(name="out", default_value="", allow_pop=False, allow_push=False))
)
