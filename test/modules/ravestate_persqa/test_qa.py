import pytest
import ravestate as rs
import ravestate_interloc as interloc
import ravestate_rawio as rawio
import ravestate_persqa as persqa
import ravestate_idle as idle
import ravestate_ontology
import ravestate_verbaliser as verbaliser


from reggol import get_logger, set_default_loglevel
logger = get_logger(__name__)


@pytest.mark.skip(reason="Might interfere with visionIO tests.")
def test_run_qa():
    last_output = ""

    with rs.Module(name="persqa_test"):

        @rs.state(cond=rs.sig_startup, read=interloc.prop_all)
        def persqa_hi(ctx: rs.ContextWrapper):
            ravestate_ontology.initialized.wait()
            interloc.handle_single_interlocutor_input(ctx, "hi")

        @rs.state(read=rawio.prop_out)
        def raw_out(ctx: rs.ContextWrapper):
            nonlocal last_output
            last_output = ctx[rawio.prop_out]
            logger.info(f"Output: {ctx[rawio.prop_out]}")

    ctx = rs.Context(
        "rawio",
        "ontology",
        "idle",
        "interloc",
        "nlp",
        "persqa",
        "persqa_test"
    )

    @rs.receptor(ctx_wrap=ctx, write=rawio.prop_in)
    def say(ctx: rs.ContextWrapper, what: str):
        ctx[rawio.prop_in] = what

    ctx.emit(rs.sig_startup)
    ctx.run_once()

    assert persqa_hi.wait()

    # Wait for name being asked
    while not raw_out.wait(.1):
        ctx.run_once(debug=True)
        ctx.test()
    assert last_output in verbaliser.get_question_list("NAME")

    # Say name
    say("Herbert")

    # Wait for acknowledgement of name
    while not raw_out.wait(.1):
        ctx.run_once()

    assert persqa.inference.wait(0)
    assert persqa.react.wait(0)
    assert "herbert" in last_output.lower()

    # Wait for any other question via idle:bored
    while not raw_out.wait(.1):
        ctx.run_once()

    # Roboy now asked some predicate. Let's give some response.
    say("Big Brother")

    # Wait for Roboy's reaction.
    while not raw_out.wait(.1):
        ctx.run_once()

    assert persqa.inference.wait(0)
    assert persqa.react.wait(0)

    # Unfortunately needed until Context adopts Properties as clones.
    interloc.prop_all.children.clear()


if __name__ == "__main__":
    # from hanging_threads import start_monitoring
    # monitoring_thread = start_monitoring()
    set_default_loglevel("DEBUG")
    test_run_qa()
    exit()
