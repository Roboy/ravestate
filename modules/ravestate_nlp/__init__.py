
from ravestate.module import Module
from ravestate.property import PropertyBase
from ravestate.state import state
from ravestate_nlp.question_word import QuestionWord
from ravestate_nlp.triple import Triple
from ravestate_nlp.extract_triples import extract_triples
from ravestate_nlp.triple import Triple
from ravestate.state import Emit
from ravestate.constraint import s
from ravestate_nlp.yes_no import yes_no

from reggol import get_logger
logger = get_logger(__name__)


with Module(name="nlp"):

    tokens = PropertyBase(name="tokens", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False),
    postags = PropertyBase(name="postags", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False),
    lemmas = PropertyBase(name="lemmas", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False),
    tags = PropertyBase(name="tags", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False),
    ner = PropertyBase(name="ner", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False),
    triples = PropertyBase(name="triples", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False),
    roboy = PropertyBase(name="roboy", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False),
    yesno = PropertyBase(name="yesno", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False)


    @state(
        cond=s("rawio:in:changed"),
        read="rawio:in",
        write=(
                "nlp:tokens",
                "nlp:postags",
                "nlp:lemmas",
                "nlp:tags",
                "nlp:ner",
                "nlp:triples",
                "nlp:roboy",
                "nlp:yesno"
        ))
    def nlp_preprocess(ctx):
        text = str(ctx["rawio:in"]).lower()
        if not text:
            return False
        nlp_doc = nlp(text)

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

        nlp_yesno = nlp_doc._.yesno
        if nlp_yesno != "_":
            ctx["nlp:yesno"] = nlp_yesno
        logger.info(f"[NLP:yesno]: {nlp_yesno}")


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

    @state(signal_name="yes-no", read="nlp:yesno")
    def nlp_yes_no_signal(ctx):
        if ctx["nlp:yesno"] != "_":
            return Emit()
        return False

    @state(signal_name="intent-play", read="nlp:triples")
    def nlp_intent_play_signal(ctx):
        nlp_triples = ctx["nlp:triples"]
        if nlp_triples[0].match_either_lemma(pred={"play"}, obj={"game"}):
            return Emit()
        return False


def init_spacy():
    # TODO: Create nlp instance in :startup state, save in context instead of global var
    global nlp, empty_token
    try:
        import en_core_web_sm as spacy_en
    except ImportError:
        from spacy.cli import download as spacy_download
        spacy_download("en_core_web_sm")
        import en_core_web_sm as spacy_en
    nlp = spacy_en.load()
    empty_token = nlp(u" ")[0]

    # TODO: Make agent id configurable, rename nlp:contains-roboy to nlp:agent-mentioned
    about_roboy = ('you', 'roboy', 'robot', 'roboboy', 'your')

    def roboy_getter(doc) -> bool:
        return any(roboy in doc.text.lower() for roboy in about_roboy)

    from spacy.tokens import Doc
    Doc.set_extension('about_roboy', getter=roboy_getter)
    Doc.set_extension('empty_token', getter=lambda doc: empty_token)
    Doc.set_extension('triples', getter=extract_triples)
    Doc.set_extension('yesno', getter=yes_no)


init_spacy()
