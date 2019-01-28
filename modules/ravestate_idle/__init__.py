from ravestate import registry
from ravestate.constraint import ConfigurableAge
from ravestate.constraint import s
from ravestate.wrappers import ContextWrapper
from ravestate.state import state, Emit

from reggol import get_logger
logger = get_logger(__name__)

IMPATIENCE_THRESHOLD_CONFIG_KEY = "impatience_threshold"


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
    """
    Emits idle:impatient signal if there are pressured activations for a (configurable) amount of time
    """
    logger.debug("Emitting idle:impatient")
    return Emit(wipe=True)


registry.register(name="idle", states=(am_i_bored, am_i_impatient),
                  config={
                      # duration in seconds how long ":pressure" should be true before getting impatient
                      IMPATIENCE_THRESHOLD_CONFIG_KEY:  1.
                  })
