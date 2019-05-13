import ravestate as rs
import ravestate_rawio as rawio
from roboy_parlai import wildtalk
import re

with rs.Module(name="wildtalk"):

    fix_spaces = re.compile(r'\s*([?!.,]+(?:\s+[?!.,]+)*)\s*')

    @rs.state(cond=rawio.prop_in.changed().max_age(-1.), read=rawio.prop_in, write=rawio.prop_out)
    def wildtalk_state(ctx):
        text = ctx[rawio.prop_in]
        if not text:  # make sure that text is not empty
            return rs.Resign()
        result = wildtalk(text)
        result = fix_spaces.sub(lambda x: "{} ".format(x.group(1).replace(" ", "")), result)
        ctx[rawio.prop_out] = result
