
from ravestate.state import state
from ravestate.module import Module
from ravestate.constraint import s

from roboy_parlai import wildtalk
from ravestate_rawio import input as raw_in, output as raw_out


with Module(name="wildtalk"):

    @state(cond=raw_in.changed_signal().max_age(-1.), read=raw_in, write=raw_out)
    def wildtalk_state(ctx):
        text = ctx[raw_in]
        if not text:  # make sure that text is not empty
            text = " "
        ctx[raw_out] = wildtalk(text)
