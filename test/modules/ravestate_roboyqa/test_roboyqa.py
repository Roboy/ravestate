import pytest
import ravestate as rs
import ravestate_interloc as interloc
import ravestate_rawio as rawio
import ravestate_persqa as persqa
import ravestate_idle as idle
import ravestate_ontology
import ravestate_verbaliser as verbaliser
import ravestate_roboyqa

from reggol import get_logger, set_default_loglevel
logger = get_logger(__name__)


def test_roboyqa():
    phrases = (
        "who are you?",
        "what is your name?",
        "how old are you?",
        "what is your age?",
        "what is your hobby?",
        "what are your hobbies?",
        "what do you like?",
        "where are you from?",
        "where do you live?",
        "who is your father/dad?",
        "who is your brother/sibling?",
        "who is your friend?",
        "what do you want to become?",
        # "what are you a member of?",
        "what can you do?",
        "what are your skills?",
        "what have you learned?",
        "what are your abilities?"
    )

    last_output = ""

    with rs.Module(name="roboyqa_test"):

        @rs.state(read=rawio.prop_out)
        def raw_out(ctx: rs.ContextWrapper):
            nonlocal last_output
            last_output = ctx[rawio.prop_out]
            logger.info(f"Output: {ctx[rawio.prop_out]}")

    ctx = rs.Context(
        "rawio",
        "ontology",
        "nlp",
        "idle",
        "verbaliser",
        "roboyqa",
        "roboyqa_test",
        "-d", "ontology", "neo4j_pw", "test"
    )

    @rs.receptor(ctx_wrap=ctx, write=rawio.prop_in)
    def say(ctx: rs.ContextWrapper, what: str):
        ctx[rawio.prop_in] = what

    ctx.emit(rs.sig_startup)
    ctx.run_once()

    for phrase in phrases:
        say(phrase)
        while not raw_out.wait(.1):
            ctx.run_once()


if __name__ == "__main__":
    test_roboyqa()
    exit()
