
from ravestate.module import Module
from ravestate.property import Property


with Module(name="rawio"):

    input = Property(
        name="in",
        default_value="",
        allow_pop=False,
        allow_push=False,
        always_signal_changed=True)

    output = Property(
        name="out",
        default_value="",
        allow_pop=False,
        allow_push=False,
        always_signal_changed=True,
        wipe_on_changed=False)

    pic_in = Property(
        name="pic_in",
        default_value=None,
        allow_pop=False,
        allow_push=False,
        always_signal_changed=True)
