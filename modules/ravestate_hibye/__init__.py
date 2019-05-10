import ravestate as rs
from ravestate_verbaliser import prop_intent

import ravestate_phrases_basic_en as lang
from ravestate_interloc import prop_all as interloc_all
from ravestate_rawio import prop_in

with rs.Module(name="hibye"):

    @rs.state(cond=interloc_all.pushed() & prop_in.changed(), write=prop_intent)
    def greeting(ctx: rs.ContextWrapper):
        ctx[prop_intent] = lang.intent_greeting

    @rs.state(cond=interloc_all.popped() & prop_in.changed(), write=prop_intent)
    def farewell(ctx: rs.ContextWrapper):
        ctx[prop_intent] = lang.intent_farewells
