import ravestate as rs

import ravestate_idle as idle
import ravestate_verbaliser as verbaliser
import ravestate_phrases_basic_en as lang


with rs.Module(name="fillers", depends=(idle.mod, verbaliser.mod)) as mod:

    @rs.state(cond=idle.sig_impatient, write=verbaliser.prop_intent)
    def impatient_fillers(ctx: rs.ContextWrapper):
        ctx[verbaliser.prop_intent] = lang.intent_fillers
