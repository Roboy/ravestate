
from ravestate.module import Module
from ravestate.property import PropertyBase


with Module(name="rawio"):

    input = PropertyBase(
        name="in",
        default_value="",
        allow_pop=False,
        allow_push=False,
        always_signal_changed=True)

    output = PropertyBase(
        name="out",
        default_value="",
        allow_pop=False,
        allow_push=False,
        always_signal_changed=True,
        wipe_on_changed=False)

    pic_in = PropertyBase(
        name="pic_in",
        default_value=None,
        allow_pop=False,
        allow_push=False,
        always_signal_changed=True)
