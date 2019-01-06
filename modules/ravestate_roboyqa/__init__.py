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

ROBOY_NODE_ID = 356

@state(triggers=s(":startup"), write="rawio:out")
def hello_world_roboyqa(ctx):
    ctx["rawio:out"] = "Ask me something about myself!"

@state(triggers=s("nlp:triples:changed"), read="nlp:triples", write="rawio:out")
def roboyqa(ctx):
    sess = ravestate_ontology.get_session()
    roboy = sess.retrieve(node_id=ROBOY_NODE_ID)[0]
    question_subject = ctx["nlp:triples"][0].get_subject()
    question_predicate = ctx["nlp:triples"][0].get_predicate()
    question_predicate_subplement = ctx["nlp:triples"][0].get_predicate_subplement()
    question_object = ctx["nlp:triples"][0].get_object()

    category = None
    memory_info = None
    #MEMBER_OF
    # get roboy relationship info
    if question_object.text == QuestionWord._place:
        if question_predicate.text == "be":
            category = "FROM"
        elif question_predicate.text == "live":
            category = "LIVE_IN"
    elif question_object.text == QuestionWord._object:
        if question_predicate.text == "like" or question_predicate_subplement.text == "like" or question_subject.text == "hobbies":
            category = "HAS_HOBBY"      
    elif question_subject.text == QuestionWord._person:
        if question_object.text == "father" or question_object.text == "dad":
            category = "CHILD_OF"
        elif question_object.text == "brother" or question_object.text == "sibling":
            category = "SIBLING_OF"
        elif question_object.text == "friend":
            category = "FRIEND_OF"
    elif question_object.text == "part" or question_object.text == "member":
        category = "MEMBER_OF"

    logger.info(roboy.get_relationships(key=category))
    #TODO not all relationships are found even though they are in the memory graph... weird shit is happening
    #relationships missing: CHILD_OF, MEMBER_Of
    if not isinstance(roboy.get_relationships(key=category), dict):
        node_id  = random.sample(roboy.get_relationships(key=category),1)[0]
        memory_info = sess.retrieve(node_id=int(node_id))[0].get_name()

    #get roboy info
    if question_object.text == QuestionWord._person or question_subject.text == "name":
        category = "full_name"
        memory_info = roboy.get_properties(key=category)
    elif question_object.text == QuestionWord._form:
        if question_predicate.text == "old" or question_predicate_subplement.text == "old":
            category = "age"
            memory_info = roboy_age(roboy.get_properties(key="birthdate"))
        elif question_predicate.text == "be": #question: How are you
            #TODO get add a list of possible answers
            pass
    elif question_object.text == QuestionWord._object:
        if question_predicate.text == "learn" or question_predicate_subplement.text == "learn" or question_object.text == "skills":
            #category = "skills"
            pass
        elif question_subject.text == "age":
            category = "age"
            memory_info = roboy_age(roboy.get_properties(key="birthdate"))  
        #elif question_predicate.text == "become" or question_predicate_subplement.text == "become":  
            #TODO add futur plans for roboy

    if memory_info:
        ctx["rawio:out"] =  verbaliser.get_random_successful_answer(category) % memory_info
    else:
        ctx["rawio:out"] =  "Sorry I do not know."

def roboy_age(birthdate : str):
    birthdate = datetime.datetime.strptime(birthdate, "%d.%m.%Y")
    today = datetime.datetime.now()
    if today.year > birthdate.year and today.month > birthdate.month:
        age = "%d years" % (today.year - birthdate.year - ((today.month, today.day) < (birthdate.month, birthdate.day)))
    age = "%d months" % (12-birthdate.month+today.month)
    return age

registry.register(
    name="roboyqa",
    states=(hello_world_roboyqa, roboyqa)
)