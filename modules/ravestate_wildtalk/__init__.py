
from ravestate.state import state
from ravestate.module import Module
from ravestate.constraint import s

from roboy_parlai import wildtalk
import ravestate_rawio


with Module(name="wildtalk"):

    @state(cond=s("rawio:in:changed", max_age=-1), read="rawio:in", write="rawio:out")
    def wildtalk_state(ctx):
        text = ctx["rawio:in"]
        if not text:  # make sure that text is not empty
            text = " "
        ctx["rawio:out"] = wildtalk(text)
