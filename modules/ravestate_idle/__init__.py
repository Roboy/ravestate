from ravestate.module import Module
from ravestate.constraint import ConfigurableAge
from ravestate.constraint import s
from ravestate.wrappers import ContextWrapper
from ravestate.state import state, Emit, Delete

from reggol import get_logger

logger = get_logger(__name__)

IMPATIENCE_THRESHOLD_CONFIG_KEY = "impatience_threshold"
CONFIG = {
    # duration in seconds how long ":pressure" should be true before getting impatient
    IMPATIENCE_THRESHOLD_CONFIG_KEY:  0.1
}

with Module(name="idle", config=CONFIG):

    @state(read=":activity", signal_name="bored")
    def am_i_bored(ctx: ContextWrapper):
        """
        Emits idle:bored signal if no states are currently partially fulfilled
        """
        if ctx[":activity"] == 0:
            logger.debug("Emitting idle:bored")
            return Emit(wipe=True)


    @state(cond=s(signal_name=":pressure:true",
                  min_age=ConfigurableAge(key=IMPATIENCE_THRESHOLD_CONFIG_KEY),
                  max_age=-1.),
           signal_name="impatient")
    def am_i_impatient(ctx: ContextWrapper):
        logger.debug("Emitting idle:impatient")
        return Emit(wipe=True)


    # This state is just for testing the bored signal
    @state(cond=s("idle:bored"), write="rawio:out")
    def play_with_me(ctx: ContextWrapper):
        ctx["rawio:out"] = "Play with me, I am bored!"

