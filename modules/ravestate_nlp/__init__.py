
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

    doc = nlp(ctx["rawio:in"])
    write_tokens(doc)
    write_postags(doc)


init_model()
registry.register(
    name="nlp",
    states=(nlp_preprocess,),
    props=(PropertyBase(name="tokens", default=""), PropertyBase(name="pos_tags", default=""))
)
