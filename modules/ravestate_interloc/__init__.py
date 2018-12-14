from ravestate.property import PropertyBase
from ravestate.state import state
from ravestate import registry

import ravestate_verbaliser
import ravestate_phrases_basic_en

from ravestate.constraint import s
from ravestate.wrappers import ContextWrapper

from reggol import get_logger
logger = get_logger(__name__)


@state(read="rawio:in", write="interloc:all")
def push_interloc(ctx: ContextWrapper):
    if ctx["rawio:in"].strip() in ['hi', 'hello']:
        ctx.push(parentpath="interloc:all", child=PropertyBase(name='x'))
        logger.debug(f"Pushed x to interloc:all")


@state(triggers=s("interloc:all:pushed"), write="verbaliser:intent")
def react_to_pushed_interloc(ctx: ContextWrapper):
    ctx["verbaliser:intent"] = "greeting"


@state(read="rawio:in", write="interloc:all")
def pop_interloc(ctx: ContextWrapper):
    # for now 'see you' instead of bye because of shutdown, when bye is sent
    if ctx["rawio:in"].strip() in ['see you']:
        ctx.pop(path="interloc:all:x")
        logger.debug(f"Popped interloc:all:x")


@state(triggers=s("interloc:all:popped"), write="verbaliser:intent")
def react_to_popped_interloc(ctx: ContextWrapper):
    ctx["verbaliser:intent"] = "farewells"


registry.register(name="interloc", states=(push_interloc, react_to_pushed_interloc, pop_interloc, react_to_popped_interloc),
                  props=PropertyBase(name="all"))
