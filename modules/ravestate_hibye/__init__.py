from ravestate.state import state
from ravestate.module import Module
from ravestate.constraint import s
from ravestate.wrappers import ContextWrapper

import ravestate_verbaliser
import ravestate_phrases_basic_en
import ravestate_interloc


with Module(name="hibye"):

    @state(cond=s("interloc:all:pushed") & s("rawio:in:changed"), write="verbaliser:intent")
    def greeting(ctx: ContextWrapper):
        ctx["verbaliser:intent"] = "greeting"

    @state(cond=s("interloc:all:popped") & s("rawio:in:changed"), write="verbaliser:intent")
    def farewell(ctx: ContextWrapper):
        ctx["verbaliser:intent"] = "farewells"
