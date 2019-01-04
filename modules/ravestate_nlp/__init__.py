
from ravestate import registry
from ravestate.constraint import s
from ravestate.property import PropertyBase
from ravestate.state import state
from ravestate.receptor import receptor
from ravestate_nlp.question_words import QuestionWord
from ravestate_nlp.rdf_triple import Triple

import spacy
from spacy.tokens import Token

from reggol import get_logger
logger = get_logger(__name__)

SUBJECT_SET = {'nsubj'}
OBJECT_SET = {'dobj', 'attr', 'advmod'}
PREDICATE_SET = {'ROOT', 'conj'}
ADJECTIVE_SET = {'acomp'}


def init_model():
    global nlp
    nlp = spacy.load('en_core_web_sm')
    roboy_names = ('you', 'roboy', 'robot', 'roboboy')
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
            word = QuestionWord(word)
            question_word = word
        if word.dep_ in SUBJECT_SET:
            triple.set_subject(word)
        if word.dep_ in OBJECT_SET:
            triple.set_object(word)
        if word.dep_ in ADJECTIVE_SET:
            triple.set_predicate_subplement(word)
        if isinstance(word, Token):
            triple = triple_search(triple, word)
    if not triple.get_subject() and question_word: 
        triple.set_subject(question_word)
    return triple


@state(read="rawio:in", write=("nlp:tokens", "nlp:postags", "nlp:lemmas", "nlp:tags", "nlp:ner", "nlp:roboy","nlp:triples"))
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
        PropertyBase(name="triples", default="", always_signal_changed=True),
        PropertyBase(name="roboy", default="", always_signal_changed=True)
    )
)
