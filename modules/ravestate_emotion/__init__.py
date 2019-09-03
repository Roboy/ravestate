import random

import ravestate as rs
import ravestate_rawio as rawio
import ravestate_nlp as nlp
from reggol import get_logger
logger = get_logger(__name__)


SHY_PROB_KEY = "shy_probability"
SURPRISED_PROB_KEY = "surprise_probability"
HAPPY_PROB_KEY = "happy_probability"
AFFECTIONATE_PROB_KEY = "affectionate_probability"
CONFIG = {
    SHY_PROB_KEY: 0.2,  # Probability for being shy when input is about Roboy
    SURPRISED_PROB_KEY: 0.1,  # Probability for being surprised when input is question
    HAPPY_PROB_KEY: 0.1,  # Probability for being happy when output is generated
    AFFECTIONATE_PROB_KEY: 0.5,  # Probability for being affectionate when keyword is in input and input is about Roboy
}

AFFECTIONATE_LIST = ["cute", "nice", "cool", "awesome", "funny"]

with rs.Module(name="emotion", config=CONFIG):

    sig_shy = rs.Signal(name="shy")
    sig_surprise = rs.Signal(name="surprise")
    sig_happy = rs.Signal(name="happy")
    sig_affectionate = rs.Signal(name="affectionate")

    @rs.state(signal=sig_shy, cond=nlp.sig_contains_roboy)
    def is_shy(ctx: rs.ContextWrapper):
        if random.random() < ctx.conf(key=SHY_PROB_KEY):
            logger.debug(f"Emitting {sig_shy.name}")
            return rs.Emit()
        return rs.Resign()

    @rs.state(signal=sig_surprise, cond=nlp.sig_is_question)
    def is_surprised(ctx: rs.ContextWrapper):
        if random.random() < ctx.conf(key=SURPRISED_PROB_KEY):
            logger.debug(f"Emitting {sig_surprise.name}")
            return rs.Emit()
        return rs.Resign()

    @rs.state(signal=sig_happy, cond=rawio.prop_out.changed())
    def is_happy(ctx: rs.ContextWrapper):
        if random.random() < ctx.conf(key=HAPPY_PROB_KEY):
            logger.debug(f"Emitting {sig_happy.name}")
            return rs.Emit()
        return rs.Resign()

    @rs.state(cond=nlp.prop_lemmas.changed() & nlp.sig_contains_roboy, signal=sig_affectionate, read=nlp.prop_lemmas)
    def is_affectionate(ctx: rs.ContextWrapper):
        if any(l in ctx[nlp.prop_lemmas] for l in AFFECTIONATE_LIST) and \
                random.random() < ctx.conf(key=AFFECTIONATE_PROB_KEY):
            logger.debug(f"Emitting {sig_affectionate.name}")
            return rs.Emit()
        return rs.Resign()

