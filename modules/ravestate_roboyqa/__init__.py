from ravestate import registry
from ravestate.state import state
from ravestate.constraint import s
from ravestate_nlp.rdf_triple import Triple
from ravestate_nlp.question_words import QuestionWord
import ravestate_ontology
from ravestate_verbaliser import verbaliser

from scientio.session import Session
from scientio.ontology.node import Node

from spacy.tokens import Token

from os.path import realpath, dirname, join
import random
import datetime

from reggol import get_logger
logger = get_logger(__name__)

verbaliser.add_folder(join(dirname(realpath(__file__)), "answering_phrases"))

# roboy 2.0 ID in the neo4j memomry graph
ROBOY_NODE_ID = 356


@state(triggers=s(":startup"), write="rawio:out")
def hello_world_roboyqa(ctx):
    ctx["rawio:out"] = "Ask me something about myself!"


# TODO get triggerd by "nlp:roboy"
@state(triggers=s("nlp:triples:changed"), read="nlp:triples", write="rawio:out")
def roboyqa(ctx):
    """
    answers question regarding roboy by retrieving the information out of the neo4j roboy memory graph
    state gets triggerd when nlp extracts a new triple: subject, predicate, object
    by analysing the triple the content of the question can be ascertained 
    the necessary information is gathered using the neo4j memory session
    if the triple combination is known and the information could be retrieved an answer will be given
    """
    sess = ravestate_ontology.get_session()
    roboy = sess.retrieve(node_id=ROBOY_NODE_ID)[0]
    triple = ctx["nlp:triples"][0]

    category = None
    memory_info = None

    if triple.is_question(QuestionWord.OBJECT):
        if triple.match_either_lemma(pred={"like"}, subj={"hobby"}):
            category = "HAS_HOBBY"
        elif triple.match_either_lemma(pred={"learn"}, subj={"skill"}):
            category = "skills"
        elif triple.match_either_lemma(pred={"can"}, subj={"ability"}):
            category = "abilities"
        elif triple.match_either_lemma(subj={"age"}):
            category = "age"
            memory_info = roboy_age(roboy.get_properties(key="birthdate"))
        elif triple.match_either_lemma(subj={"name"}):
            category = "full_name"
            memory_info = roboy.get_properties(key=category)
        elif triple.match_either_lemma(pred={"become"}):
            category = "future"   
    elif triple.is_question(QuestionWord.PLACE):
        if triple.match_either_lemma(pred={"be"}):
            category = "FROM"
        elif triple.match_either_lemma(pred={"live"}):
            category = "LIVE_IN"
    elif triple.is_question(QuestionWord.PERSON):
        if triple.match_either_lemma(obj={"father", "dad"}):
            category = "CHILD_OF"
        elif triple.match_either_lemma(obj={"brother", "sibling"}):
            category = "SIBLING_OF"
        elif triple.match_either_lemma(obj={"friend", "girlfriend"}):
            category = "FRIEND_OF"
        else:
            category = "full_name"
            memory_info = roboy.get_properties(key=category)
    elif triple.match_either_lemma(obj={"part", "member"}):
        category = "MEMBER_OF"
    elif triple.is_question(QuestionWord.FORM):
        if triple.match_either_lemma(pred={"old"}):
            category = "age"
            memory_info = roboy_age(roboy.get_properties(key="birthdate"))
        elif triple.match_either_lemma(pred={"be"}):
            category = "well_being"
    elif triple.match_either_lemma(obj={"skill"}):
        category = "skills"
    elif triple.match_either_lemma(obj={"ability"}):
        category = "abilities"

    if category and category.isupper() and not isinstance(roboy.get_relationships(key=category), dict):
        node_id = random.sample(roboy.get_relationships(key=category),1)[0]
        memory_info = sess.retrieve(node_id=int(node_id))[0].get_name()

    elif category and category.islower() and not isinstance(roboy.get_properties(key=category), dict):
        property_list = [x.strip() for x in roboy.get_properties(key=category).split(',')]
        memory_info = random.sample(property_list, 1)[0]

    if memory_info:
        ctx["rawio:out"] = verbaliser.get_random_successful_answer(category) % memory_info
    elif category == "well_being":
        ctx["rawio:out"] = verbaliser.get_random_successful_answer(category)
    else:
        ctx["rawio:out"] = "Sorry I do not know."

def roboy_age(birthdate : str):
    """
    calculates roboys age given his birthdate
    example: birthdate = "12.04.18"
    """
    # TODO
    birthdate = datetime.datetime.strptime(birthdate, "%d.%m.%Y")
    today = datetime.datetime.now()
    if today.year > birthdate.year and today.month > birthdate.month:
        age = "%d years" % (today.year - birthdate.year - \
                            ((today.month, today.day) < (birthdate.month, birthdate.day)))
    age = "%d months" % (12-birthdate.month+today.month)
    return age

registry.register(
    name="roboyqa",
    states=(hello_world_roboyqa, roboyqa)
)