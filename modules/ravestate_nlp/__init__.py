
from ravestate import registry
from ravestate.constraint import s
from ravestate.property import PropertyBase
from ravestate.state import state
from ravestate.receptor import receptor

import spacy


def init_model():
    global nlp
    nlp = spacy.load('en_core_web_sm')


@state(read="rawio:in")
def nlp_preprocess(ctx):

    @receptor(ctx_wrap=ctx, write="rawio:out")
    def write_tokens(ctx_input, nlp_doc):
        ctx_input["rawio:out"] = [str(token) for token in nlp_doc]

    @receptor(ctx_wrap=ctx, write="rawio:out")
    def write_postags(ctx_input, nlp_doc):
        ctx_input["rawio:out"] = [str(token.pos_) for token in nlp_doc]

    @receptor(ctx_wrap=ctx, write="rawio:out")
    def write_lemmas(ctx_input, nlp_doc):
        ctx_input["rawio:out"] = [str(token.lemma_) for token in nlp_doc]

    @receptor(ctx_wrap=ctx, write="rawio:out")
    def write_tags(ctx_input, nlp_doc):
        ctx_input["rawio:out"] = [str(token.tag_) for token in nlp_doc]

    @receptor(ctx_wrap=ctx, write="rawio:out")
    def write_named_enities(ctx_input, nlp_doc):
        ctx_input["rawio:out"] = [str(token.label_) for token in nlp_doc.ents]

    @receptor(ctx_wrap=ctx, write="rawio:out")
    def write_roboy_detected(ctx_input, nlp_doc):
        roboy = nlp("Roboy, roboy, Robot, robot, Roboboy")
        similarity = 0.8
        for token in nlp_doc:
            for roboy_token in roboy:
                if token.similarity(roboy_token) > similarity:
                    ctx_input["rawio:out"] = ["ROBOY"]

    doc = nlp(ctx["rawio:in"])
    write_tokens(doc)
    write_postags(doc)
    write_lemmas(doc)
    write_tags(doc)
    write_named_enities(doc)
    write_roboy_detected(doc)


init_model()
registry.register(
    name="nlp",
    states=(nlp_preprocess,),
    props=(PropertyBase(name="tokens", default=""), PropertyBase(name="pos_tags", default=""))
)
