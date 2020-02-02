import ravestate as rs
import ravestate_nlp as nlp
import ravestate_verbaliser as verbaliser
import ravestate_idle as idle
import ravestate_rawio as rawio
import ravestate_ontology
import ravestate_interloc as interloc
import ravestate_nlp as nlp

from os.path import realpath, dirname, join
from datetime import datetime

from scientio.ontology.ontology import Ontology
from scientio.session import Session
from scientio.ontology.node import Node

from reggol import get_logger
logger = get_logger(__name__)

verbaliser.add_file(join(dirname(realpath(__file__)), "jokes_data.yml"))

with rs.Module(
        name="funboy",
        config={},
        depends=(verbaliser.mod, rawio.mod, interloc.mod, nlp.mod)) as mod:

    sig_tell_joke = rs.Signal(name="tell_joke")
    sig_joke_told = rs.Signal(name="joke_told")
    sig_joke_finish = rs.Signal(name="joke_finish")

    prop_start_timestamp = rs.Property(name="start_timestamp")
    prop_interloc_emotion = rs.Property(name="interloc_emotion")
    prop_joke_category = rs.Property(name="joke_category")

    @rs.state(cond=rs.sig_startup,
              write=prop_start_timestamp)
    def start(ctx):
        ctx[prop_start_timestamp] = datetime.now()

    @rs.state(cond=nlp.prop_triples.changed(),
              read=(nlp.prop_triples, prop_start_timestamp),
              signal=sig_tell_joke)
    def deduce(ctx):
        triples = ctx[nlp.prop_triples]
        do_jokes = True
        if do_jokes and datetime.now() - ctx[prop_start_timestamp] > 120:
            return rs.Emit()

    @rs.state(cond=sig_tell_joke, write=prop_joke_category)
    def assess(ctx):
        session: Session = ravestate_ontology.get_session()
        ontology: Ontology = ravestate_ontology.get_ontology()

        # Write property

    @rs.state(cond=sig_tell_joke & prop_joke_category.changed(), read=(rawio.prop_in, prop_joke_category),
              write=rawio.prop_out, signal=sig_joke_told)
    def render(ctx):
        # Do GPT magic

        ctx[rawio.prop_out] = verbaliser.get_random_phrase("gpt2-jokes")
        return rs.Emit()

    # @rs.state(cond=sig_joke_told & prop_interloc_emotion.changed().max_age(0))
    # def validate(ctx):
    #     pass

    @rs.state(cond=sig_joke_told & prop_interloc_emotion.changed().max_age(0),
              read=(prop_interloc_emotion, prop_joke_category, interloc.prop_all),
              signal=sig_joke_finish)
    def attune(ctx):
        session: Session = ravestate_ontology.get_session()
        ontology: Ontology = ravestate_ontology.get_ontology()
        # Only 1 interlocutor
        for i in ctx.enum(interloc.prop_all):
            node = ctx[i]
        # interact with the Node via scientio
        return rs.Emit()

    @rs.state(cond=rs.sig_startup)
    def emotionizer(ctx):
        @rs.receptor(ctx_wrap=ctx, write=prop_interloc_emotion)
        def write_interloc_emotion(ctx, emotion):
            ctx[prop_interloc_emotion] = emotion

        write_interloc_emotion("happy")
