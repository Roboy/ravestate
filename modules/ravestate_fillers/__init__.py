from ravestate import registry
from ravestate.wrappers import ContextWrapper
from ravestate.constraint import s
from ravestate.state import state

import ravestate_idle
import ravestate_verbaliser
import ravestate_phrases_basic_en


@state(cond=s("idle:impatient"), write=("verbaliser:intent",))
def impatient_fillers(ctx: ContextWrapper):
    ctx["verbaliser:intent"] = "fillers"


registry.register(name="fillers", states=(impatient_fillers,))
