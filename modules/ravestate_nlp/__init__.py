
from ravestate import registry
from ravestate.constraint import s
from ravestate.property import PropertyBase
from ravestate.state import state
from ravestate.receptor import receptor

import spacy
import logging

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
    
    nlp_tokens = tuple(str(token) for token in nlp_doc)
    ctx["nlp:tokens"] = nlp_tokens
    logging.info(f"[NLP:tokens]: {nlp_tokens}")

    nlp_postags = tuple(str(token.pos_) for token in nlp_doc)
    ctx["nlp:postags"] = nlp_postags
    logging.info(f"[NLP:postags]: {nlp_postags}")

    nlp_lemmas = tuple(str(token.lemma_) for token in nlp_doc)
    ctx["nlp:lemmas"] = nlp_lemmas
    logging.info(f"[NLP:lemmas]: {nlp_lemmas}")
    
    nlp_tags = tuple(str(token.tag_) for token in nlp_doc)
    ctx["nlp:tags"] = nlp_tags
    logging.info(f"[NLP:tags]: {nlp_tags}")

    nlp_ner = tuple((str(ents.text), str(ents.label_)) for ents in nlp_doc.ents)
    ctx["nlp:ner"] = nlp_ner
    logging.info(f"[NLP:ner]: {nlp_ner}")

    nlp_roboy = nlp_doc._.about_roboy
    ctx["nlp:roboy"] = nlp_roboy
    logging.info(f"[NLP:roboy]: {nlp_roboy}")


init_model()
registry.register(
    name="nlp",
    states=(nlp_preprocess,),
    props=(
        PropertyBase(name="tokens", default="", always_signal_changed=True),
        PropertyBase(name="postags", default="", always_signal_changed=True),
        PropertyBase(name="lemmas", default="", always_signal_changed=True),
        PropertyBase(name="tags", default="", always_signal_changed=True),
        PropertyBase(name="ner", default="", always_signal_changed=True),
        PropertyBase(name="roboy", default="", always_signal_changed=True)
    )
)
