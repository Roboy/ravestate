
from ravestate import registry
from ravestate.property import PropertyBase
from ravestate.state import state
from ravestate_nlp.question_words import QuestionWord
from ravestate_nlp.rdf_triple import Triple
from ravestate.state import Emit


import spacy
from spacy.tokens import Token

from reggol import get_logger
logger = get_logger(__name__)

SUBJECT_SET = {'nsubj'}
OBJECT_SET = {'dobj', 'attr', 'advmod'}
PREDICATE_SET = {'ROOT', 'conj'}
PREDICATE_AUX_SET = {'acomp', 'aux', 'xcomp'}
VERB_AUX_SET = {'VERB', 'ADJ'}


def init_model():
    global nlp
    nlp = spacy.load('en_core_web_sm')
    roboy_names = ('you', 'roboy', 'robot', 'roboboy', 'your')
    roboy_getter = lambda doc: any(roboy in doc.text.lower() for roboy in roboy_names)
    from spacy.tokens import Doc
    Doc.set_extension('about_roboy', getter=roboy_getter)
    Doc.set_extension('triple', getter=extract_triples)


def extract_triples(doc):
    """
    triple: subject, predicate, object
    finding the triple of a sentence: determine all the dependencies
    starts with predicate and searches through the dependency tree -> triple_search()
    """
    triples = []
    for token in doc:
        if token.dep_ in PREDICATE_SET:
            triple = triple_search(Triple(predicate=token), token)
            triples.append(triple)
    return triples


def triple_search(triple: Triple, token: Token): 
    """
    Recursive search through the dependency tree
    looks for triple values in each of the children and calls itself with the children nodes
    """
    question_word = None
    for word in token.children:
        if word.text.lower() in QuestionWord.question_words:
            question_word = QuestionWord(word)
            word = QuestionWord(word)
            if not triple.get_object() or triple.get_object().text == " ":
                triple.set_object(question_word)
        elif word.dep_ in OBJECT_SET:
            triple.set_object(word)
        if word.dep_ in SUBJECT_SET:
            triple.set_subject(word)
        if word.dep_ in PREDICATE_AUX_SET and word.pos_ in VERB_AUX_SET:
            triple.set_predicate_aux(word)
        if isinstance(word, Token):
            triple = triple_search(triple, word)
    if not triple.get_subject() and question_word: 
        triple.set_subject(question_word)

    # do not allow empty triples
    empty_doc = nlp(u' ')
    empty_token = empty_doc[0]
    if not triple.get_subject():
       triple.set_subject(empty_token)
    if not triple.get_predicate():
        triple.set_predicate(empty_token) 
    if not triple.get_predicate_aux():
        triple.set_predicate_aux(empty_token)
    if not triple.get_object():
        triple.set_object(empty_token)  
        
    return triple


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

    nlp_triples = nlp_doc._.triple
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
