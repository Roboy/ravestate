import ravestate as rs

from ravestate_idle import sig_impatient
from ravestate_verbaliser import prop_intent
import ravestate_phrases_basic_en as lang


with rs.Module(name="fillers"):

    @rs.state(cond=sig_impatient, write=prop_intent)
    def impatient_fillers(ctx: rs.ContextWrapper):
        ctx[prop_intent] = lang.intent_fillers
