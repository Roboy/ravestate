
from ravestate.state import state
from ravestate import registry

from roboy_parlai import wildtalk
import ravestate_rawio


@state(read="rawio:in", write="rawio:out")
def wildtalk_state(ctx):
    ctx["rawio:out"] = wildtalk(ctx["rawio:in"])


registry.register(name="wildtalk", states=(wildtalk_state,))
