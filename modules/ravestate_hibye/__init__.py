from ravestate.state import state
from ravestate import registry
from ravestate.constraint import s
from ravestate.wrappers import ContextWrapper

import ravestate_verbaliser
import ravestate_phrases_basic_en
import ravestate_interloc


@state(cond=s("interloc:all:pushed") & s("rawio:in:changed"), write="verbaliser:intent")
def react_to_pushed_interloc(ctx: ContextWrapper):
    ctx["verbaliser:intent"] = "greeting"


@state(cond=s("interloc:all:popped") & s("rawio:in:changed"), write="verbaliser:intent")
def react_to_popped_interloc(ctx: ContextWrapper):
    ctx["verbaliser:intent"] = "farewells"


registry.register(name="hi_goodbye", states=(react_to_pushed_interloc, react_to_popped_interloc))
