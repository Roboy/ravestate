from ravestate.module import Module
from ravestate.wrappers import ContextWrapper
from ravestate.constraint import s
from ravestate.state import state

import ravestate_idle
import ravestate_verbaliser
import ravestate_phrases_basic_en


with Module(name="fillers"):

    @state(cond=s("idle:impatient"), write=("verbaliser:intent",))
    def impatient_fillers(ctx: ContextWrapper):
        ctx["verbaliser:intent"] = "fillers"
