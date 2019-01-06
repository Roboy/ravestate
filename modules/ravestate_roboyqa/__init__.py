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

verbaliser.add_folder(join(dirname(realpath(__file__)), "answering_phrases"))

#roboy 2.0 ID in the neo4j memomry graph
ROBOY_NODE_ID = 356

@state(triggers=s(":startup"), write="rawio:out")
def hello_world_roboyqa(ctx):
    ctx["rawio:out"] = "Ask me something about myself!"

#TODO get triggerd by "nlp:roboy"
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
    question_subject = ctx["nlp:triples"][0].get_subject()
    question_predicate = ctx["nlp:triples"][0].get_predicate()
    question_predicate_subplement = ctx["nlp:triples"][0].get_predicate_subplement()
    question_object = ctx["nlp:triples"][0].get_object()

    category = None
    memory_info = None

    if question_object.text == QuestionWord._object:
        if question_predicate.lemma_ == "like" or question_predicate_subplement.lemma_ == "like" \
            or question_subject.text == "hobbies":
            category = "HAS_HOBBY"   
        elif question_predicate.lemma_ == "learn" or question_predicate_subplement.lemma_ == "learn" \
            or question_object.text == "skills":
            #category = "skills"
            pass
        elif question_subject.text == "age":
            category = "age"
            memory_info = roboy_age(roboy.get_properties(key="birthdate"))
        elif question_subject.text == "name":
            category = "full_name"
            memory_info = roboy.get_properties(key=category)
        #elif question_predicate.lemma_ == "become" or question_predicate_subplement.lemma_ == "become":  
            #TODO add futur plans for roboy   
    elif question_object.text == QuestionWord._place:
        if question_predicate.lemma_ == "be":
            category = "FROM"
        elif question_predicate.lemma_ == "live":
            category = "LIVE_IN"
    elif question_subject.text == QuestionWord._person:
        if question_object.text == "father" or question_object.text == "dad":
            category = "CHILD_OF"
        elif question_object.text == "brother" or question_object.text == "sibling":
            category = "SIBLING_OF"
        elif question_object.text == "friend" or question_object.text == "girlfriend":
            category = "FRIEND_OF"
    elif question_object.text == "part" or question_object.text == "member":
        category = "MEMBER_OF"
    elif question_object.text == QuestionWord._person:
        category = "full_name"
        memory_info = roboy.get_properties(key=category)
    elif question_object.text == QuestionWord._form:
        logger.info(question_predicate.text)
        if question_predicate.lemma_ == "old" or question_predicate_subplement.lemma_ == "old":
            category = "age"
            memory_info = roboy_age(roboy.get_properties(key="birthdate"))
        elif question_predicate.lemma_ == "be":
            category = "well_being"

    if category and category.isupper() and not isinstance(roboy.get_relationships(key=category), dict):
        node_id  = random.sample(roboy.get_relationships(key=category),1)[0]
        try: 
            memory_info = sess.retrieve(node_id=int(node_id))[0].get_name()
        except AttributeError:
            #TODO figure our why this happens; 
            # question: what are you a member of -> finds node 20! but causes error 
            logger.error("Could not get name of node")

    if memory_info:
        ctx["rawio:out"] =  verbaliser.get_random_successful_answer(category) % memory_info
    elif category == "well_being":
        ctx["rawio:out"] =  verbaliser.get_random_successful_answer(category)
    else:
        ctx["rawio:out"] =  "Sorry I do not know."

def roboy_age(birthdate : str):
    """
    calculates roboys age given his birthdate
    example: birthdate = "12.04.18"
    """
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