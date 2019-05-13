import ravestate as rs

import ravestate_rawio as rawio
from ravestate_nlp.question_word import QuestionWord
from ravestate_nlp.triple import Triple
from ravestate_nlp.extract_triples import extract_triples
from ravestate_nlp.yes_no import yes_no

from reggol import get_logger
logger = get_logger(__name__)


def init_spacy():
    # TODO: Create nlp instance in core:startup state, save in context instead of global var
    global empty_token
    try:
        import en_core_web_sm as spacy_en
    except ImportError:
        from spacy.cli import download as spacy_download
        spacy_download("en_core_web_sm")
        import en_core_web_sm as spacy_en
    spacy_nlp_en = spacy_en.load()
    empty_token = spacy_nlp_en(u" ")[0]

    # TODO: Make agent id configurable, rename nlp:contains-roboy to nlp:agent-mentioned
    about_roboy = ('you', 'roboy', 'robot', 'roboboy', 'your')

    def roboy_getter(doc) -> bool:
        return any(roboy in doc.text.lower() for roboy in about_roboy)

    from spacy.tokens import Doc
    Doc.set_extension('about_roboy', getter=roboy_getter)
    Doc.set_extension('empty_token', getter=lambda doc: empty_token)
    Doc.set_extension('triples', getter=extract_triples)
    Doc.set_extension('yesno', getter=yes_no)
    return spacy_nlp_en


spacy_nlp_en = init_spacy()


with rs.Module(name="nlp"):

    prop_tokens = rs.Property(name="tokens", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False)
    porp_postags = rs.Property(name="postags", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False)
    prop_lemmas = rs.Property(name="lemmas", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False)
    prop_tags = rs.Property(name="tags", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False)
    prop_ner = rs.Property(name="ner", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False)
    prop_triples = rs.Property(name="triples", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False)
    prop_roboy = rs.Property(name="roboy", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False)
    prop_yesno = rs.Property(name="yesno", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False)

    sig_contains_roboy = rs.Signal(name="contains-roboy")
    sig_is_question = rs.Signal(name="is-question")
    sig_intent_play = rs.Signal(name="intent-play")


    @rs.state(read=rawio.prop_in, write=(prop_tokens, porp_postags, prop_lemmas, prop_tags, prop_ner, prop_triples, prop_roboy, prop_yesno))
    def nlp_preprocess(ctx):
        text = ctx[rawio.prop_in]
        if not text:
            return False
        text = text.lower()
        nlp_doc = spacy_nlp_en(text)

        nlp_tokens = tuple(str(token) for token in nlp_doc)
        ctx[prop_tokens] = nlp_tokens
        logger.info(f"[NLP:tokens]: {nlp_tokens}")

        nlp_postags = tuple(str(token.pos_) for token in nlp_doc)
        ctx[porp_postags] = nlp_postags
        logger.info(f"[NLP:postags]: {nlp_postags}")

        nlp_lemmas = tuple(str(token.lemma_) for token in nlp_doc)
        ctx[prop_lemmas] = nlp_lemmas
        logger.info(f"[NLP:lemmas]: {nlp_lemmas}")

        nlp_tags = tuple(str(token.tag_) for token in nlp_doc)
        ctx[prop_tags] = nlp_tags
        logger.info(f"[NLP:tags]: {nlp_tags}")

        nlp_ner = tuple((str(ents.text), str(ents.label_)) for ents in nlp_doc.ents)
        ctx[prop_ner] = nlp_ner
        logger.info(f"[NLP:ner]: {nlp_ner}")

        nlp_triples = nlp_doc._.triples
        ctx[prop_triples] = nlp_triples
        logger.info(f"[NLP:triples]: {nlp_triples}")

        nlp_roboy = nlp_doc._.about_roboy
        ctx[prop_roboy] = nlp_roboy
        logger.info(f"[NLP:roboy]: {nlp_roboy}")

        nlp_yesno = nlp_doc._.yesno
        ctx[prop_yesno] = nlp_yesno
        logger.info(f"[NLP:yesno]: {nlp_yesno}")

    @rs.state(signal=sig_contains_roboy, read=prop_roboy)
    def nlp_contains_roboy_signal(ctx):
        if ctx[prop_roboy]:
            return rs.Emit()
        return False

    @rs.state(signal=sig_is_question, read=prop_triples)
    def nlp_is_question_signal(ctx):
        if ctx[prop_triples][0].is_question():
            return rs.Emit()
        return False

    @rs.state(signal=sig_intent_play, read=prop_triples)
    def nlp_intent_play_signal(ctx):
        nlp_triples = ctx[prop_triples]
        if nlp_triples[0].match_either_lemma(pred={"play"}, obj={"game"}):
            return rs.Emit()
        return False

