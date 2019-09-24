import ravestate as rs

import ravestate_verbaliser as verbaliser
import ravestate_phrases_basic_en as lang
import ravestate_interloc as interloc
import ravestate_rawio as rawio


with rs.Module(name="hibye", depends=(interloc.mod, rawio.mod)) as mod:

    @rs.state(cond=interloc.prop_all.pushed() & rawio.prop_in.changed(), write=verbaliser.prop_intent)
    def greeting(ctx: rs.ContextWrapper):
        ctx[verbaliser.prop_intent] = lang.intent_greeting

    @rs.state(cond=interloc.prop_all.popped() & rawio.prop_in.changed(), write=verbaliser.prop_intent)
    def farewell(ctx: rs.ContextWrapper):
        ctx[verbaliser.prop_intent] = lang.intent_farewells
