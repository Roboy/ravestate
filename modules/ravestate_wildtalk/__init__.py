import ravestate as rs
import ravestate_rawio as rawio
from roboy_parlai import wildtalk


with rs.Module(name="wildtalk"):

    @rs.state(cond=rawio.prop_out.changed().max_age(-1.), read=rawio.prop_in, write=rawio.prop_out)
    def wildtalk_state(ctx):
        text = ctx[rawio.prop_in]
        if not text:  # make sure that text is not empty
            return rs.Resign()
        ctx[rawio.prop_out] = wildtalk(text)
