from ravestate.module import Module
from ravestate.constraint import ConfigurableAge, Signal, s
from ravestate.wrappers import ContextWrapper
from ravestate.state import state, Emit
from ravestate.context import activity_property, pressure_property

from reggol import get_logger
logger = get_logger(__name__)

IMPATIENCE_THRESHOLD_CONFIG_KEY = "impatience_threshold"
BORED_THRESHOLD_CONFIG_KEY = "bored_threshold"
CONFIG = {
    # duration in seconds how long ":pressure" should be true before getting impatient
    IMPATIENCE_THRESHOLD_CONFIG_KEY:  1.0,
    BORED_THRESHOLD_CONFIG_KEY: 1.0
}

with Module(name="idle", config=CONFIG):

    impatient = Signal(name="impatient")
    bored = Signal(name="bored")

    @state(cond=activity_property.changed_signal().min_age(ConfigurableAge(key=BORED_THRESHOLD_CONFIG_KEY)).max_age(-1),
           read=activity_property, signal=bored)
    def am_i_bored(ctx: ContextWrapper):
        """
        Emits idle:bored signal if no states are currently partially fulfilled
        """
        if ctx[activity_property] == 0:
            return Emit(wipe=True)

    @state(cond=pressure_property.flag_true_signal().
           min_age(ConfigurableAge(key=IMPATIENCE_THRESHOLD_CONFIG_KEY)).max_age(-1.),
           signal=impatient)
    def am_i_impatient(ctx: ContextWrapper):
        return Emit(wipe=True)
