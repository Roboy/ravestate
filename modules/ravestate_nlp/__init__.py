
from ravestate import registry
from ravestate.property import PropertyBase
from ravestate.state import state
from ravestate_nlp.question_word import QuestionWord
from ravestate_nlp.triple import Triple
from ravestate_nlp.extract_triples import extract_triples
from ravestate.state import Emit
import spacy


from reggol import get_logger
logger = get_logger(__name__)


def init_model():
    # TODO: Create nlp instance in :startup state, save in context instead of global var
    global nlp, empty_token
    nlp = spacy.load('en_core_web_sm')
    empty_token = nlp(u' ')[0]

    # TODO: Make agent id configurable, rename nlp:contains-roboy to nlp:agent-mentioned
    about_roboy = ('you', 'roboy', 'robot', 'roboboy', 'your')
    def roboy_getter(doc) -> bool:
        return any(roboy in doc.text.lower() for roboy in about_roboy)

    from spacy.tokens import Doc
    Doc.set_extension('about_roboy', getter=roboy_getter)
    Doc.set_extension('empty_token', getter=lambda doc: empty_token)
    Doc.set_extension('triples', getter=extract_triples)


@state(read="rawio:in", write=("nlp:tokens", "nlp:postags", "nlp:lemmas", "nlp:tags", "nlp:ner", "nlp:triples", "nlp:roboy","nlp:triples"))
def nlp_preprocess(ctx):
    nlp_doc = nlp(ctx["rawio:in"])
    
    nlp_tokens = tuple(str(token) for token in nlp_doc)
    ctx["nlp:tokens"] = nlp_tokens
    logger.info(f"[NLP:tokens]: {nlp_tokens}")

    nlp_postags = tuple(str(token.pos_) for token in nlp_doc)
    ctx["nlp:postags"] = nlp_postags
    logger.info(f"[NLP:postags]: {nlp_postags}")

    nlp_lemmas = tuple(str(token.lemma_) for token in nlp_doc)
    ctx["nlp:lemmas"] = nlp_lemmas
    logger.info(f"[NLP:lemmas]: {nlp_lemmas}")
    
    nlp_tags = tuple(str(token.tag_) for token in nlp_doc)
    ctx["nlp:tags"] = nlp_tags
    logger.info(f"[NLP:tags]: {nlp_tags}")

    nlp_ner = tuple((str(ents.text), str(ents.label_)) for ents in nlp_doc.ents)
    ctx["nlp:ner"] = nlp_ner
    logger.info(f"[NLP:ner]: {nlp_ner}")

    nlp_triples = nlp_doc._.triples
    ctx["nlp:triples"] = nlp_triples
    logger.info(f"[NLP:triples]: {nlp_triples}")

    nlp_roboy = nlp_doc._.about_roboy
    ctx["nlp:roboy"] = nlp_roboy
    logger.info(f"[NLP:roboy]: {nlp_roboy}")


@state(signal_name="contains-roboy", read="nlp:roboy")
def nlp_contains_roboy_signal(ctx):
    if ctx["nlp:roboy"]:
        return Emit()
    return False


@state(signal_name="is-question", read="nlp:triples")
def nlp_is_question_signal(ctx):
    if ctx["nlp:triples"][0].is_question():
        return Emit()
    return False


init_model()
registry.register(
    name="nlp",
    states=(
        nlp_preprocess,
        nlp_contains_roboy_signal,
        nlp_is_question_signal
    ),
    props=(
        PropertyBase(name="tokens", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False),
        PropertyBase(name="postags", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False),
        PropertyBase(name="lemmas", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False),
        PropertyBase(name="tags", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False),
        PropertyBase(name="ner", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False),
        PropertyBase(name="triples", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False),
        PropertyBase(name="roboy", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False)
    )
)
