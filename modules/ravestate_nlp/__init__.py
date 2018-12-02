
from ravestate import registry
from ravestate.constraint import s
from ravestate.property import PropertyBase
from ravestate.state import state
from ravestate.receptor import receptor

import spacy
import logging

logger = logging.getLogger(__name__)

def init_model():
    global nlp
    nlp = spacy.load('en_core_web_sm')
    roboy_names = ('you', 'roboy', 'robot', 'roboboy')
    roboy_getter = lambda doc: any(roboy in doc.text.lower() for roboy in roboy_names)
    from spacy.tokens import Doc
    Doc.set_extension('about_roboy', getter=roboy_getter)


@state(read="rawio:in", write=("nlp:tokens", "nlp:postags", "nlp:lemmas", "nlp:tags", "nlp:ner", "nlp:roboy"))
def nlp_preprocess(ctx):
    nlp_doc = nlp(ctx["rawio:in"])
    ctx["nlp:tokens"] = tuple(str(token) for token in nlp_doc)
    ctx["nlp:postags"] = tuple(str(token.pos_) for token in nlp_doc)
    ctx["nlp:lemmas"] = tuple(str(token.lemma_) for token in nlp_doc)
    ctx["nlp:tags"] = tuple(str(token.tag_) for token in nlp_doc)
    ctx["nlp:ner"] = tuple((str(ents.text), str(ents.label_)) for ents in nlp_doc.ents)
    ctx["nlp:roboy"] = nlp_doc._.about_roboy


@state(read="nlp:tokens")
def tokens_output(ctx):
    logger.info('[NLP:tokens]:', ctx["nlp:tokens"])


@state(read="nlp:postags")
def postags_output(ctx):
    logger.info('[NLP:postags]:', ctx["nlp:postags"])


@state(read="nlp:lemmas")
def lemmas_output(ctx):
    logger.info('[NLP:lemmas]:', ctx["nlp:lemmas"])


@state(read="nlp:ner")
def ner_output(ctx):
    logger.info('[NLP:ner]:', ctx["nlp:ner"])


@state(read="nlp:tags")
def tags_output(ctx):
    logger.info('[NLP:tags]:', ctx["nlp:tags"])


@state(read="nlp:roboy")
def roboy_output(ctx):
    logger.info('[NLP:roboy]:', ctx["nlp:roboy"])


init_model()
registry.register(
    name="nlp",
    states=(nlp_preprocess, tokens_output, postags_output, lemmas_output, ner_output, tags_output, roboy_output),
    props=(
        PropertyBase(name="tokens", default="", always_signal_changed=True),
        PropertyBase(name="postags", default="", always_signal_changed=True),
        PropertyBase(name="lemmas", default="", always_signal_changed=True),
        PropertyBase(name="tags", default="", always_signal_changed=True),
        PropertyBase(name="ner", default="", always_signal_changed=True),
        PropertyBase(name="roboy", default="", always_signal_changed=True)
    )
)
