
from ravestate import registry
from ravestate.constraint import s
from ravestate.property import PropertyBase
from ravestate.state import state
from ravestate.receptor import receptor

import spacy


def init_model():
    global nlp
    nlp = spacy.load('en_core_web_sm')
    roboy_getter = lambda doc: any(roboy in doc.text.lower() for roboy in ('you', 'roboy', 'robot', 'roboboy'))
    from spacy.tokens import Doc
    Doc.set_extension('about_roboy', getter=roboy_getter)


@state(read="rawio:in")
def nlp_preprocess(ctx):

    @receptor(ctx_wrap=ctx, write="nlp:tokens")
    def write_tokens(ctx_input, nlp_doc):
        ctx_input["nlp:tokens"] = [str(token) for token in nlp_doc]

    @receptor(ctx_wrap=ctx, write="nlp:postags")
    def write_postags(ctx_input, nlp_doc):
        ctx_input["nlp:postags"] = [str(token.pos_) for token in nlp_doc]

    @receptor(ctx_wrap=ctx, write="nlp:lemmas")
    def write_lemmas(ctx_input, nlp_doc):
        ctx_input["nlp:lemmas"] = [str(token.lemma_) for token in nlp_doc]

    @receptor(ctx_wrap=ctx, write="nlp:tags")
    def write_tags(ctx_input, nlp_doc):
        ctx_input["nlp:tags"] = [str(token.tag_) for token in nlp_doc]

    @receptor(ctx_wrap=ctx, write="nlp:ner")
    def write_named_enities(ctx_input, nlp_doc):
        ctx_input["nlp:ner"] = [(str(ents.text), str(ents.label_)) for ents in nlp_doc.ents]

    @receptor(ctx_wrap=ctx, write="nlp:roboy")
    def write_roboy_detected(ctx_input, nlp_doc):
        ctx_input["nlp:roboy"] = nlp_doc._.about_roboy

    doc = nlp(ctx["rawio:in"])
    write_tokens(doc)
    write_postags(doc)
    write_lemmas(doc)
    write_tags(doc)
    write_named_enities(doc)
    write_roboy_detected(doc)


@state(read="nlp:tokens")
def tokens_output(ctx):
    print('[NLP:tokens]:', ctx["nlp:tokens"])


@state(read="nlp:postags")
def postags_output(ctx):
    print('[NLP:postags]:', ctx["nlp:postags"])


@state(read="nlp:lemmas")
def lemmas_output(ctx):
    print('[NLP:lemmas]:', ctx["nlp:lemmas"])


@state(read="nlp:ner")
def ner_output(ctx):
    print('[NLP:ner]:', ctx["nlp:ner"])


@state(read="nlp:tags")
def tags_output(ctx):
    print('[NLP:tags]:', ctx["nlp:tags"])


@state(read="nlp:roboy")
def roboy_output(ctx):
    print('[NLP:roboy]:', ctx["nlp:roboy"])


init_model()
registry.register(
    name="nlp",
    states=(nlp_preprocess, tokens_output, postags_output, lemmas_output, ner_output, tags_output, roboy_output),
    props=(
        PropertyBase(name="tokens", default=""),
        PropertyBase(name="postags", default=""),
        PropertyBase(name="lemmas", default=""),
        PropertyBase(name="tags", default=""),
        PropertyBase(name="ner", default=""),
        PropertyBase(name="roboy", default="")
    )
)
