
from ravestate.state import state
from ravestate import registry
from ravestate.constraint import s

from roboy_parlai import wildtalk
import ravestate_rawio


@state(cond=s("rawio:in:changed", max_age=-1), read="rawio:in", write="rawio:out")
def wildtalk_state(ctx):
    ctx["rawio:out"] = wildtalk(ctx["rawio:in"])


registry.register(name="wildtalk", states=(wildtalk_state,))
