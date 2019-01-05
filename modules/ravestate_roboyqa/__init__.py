from ravestate import registry
from ravestate.state import state
from ravestate.constraint import s
from ravestate_nlp.rdf_triple import Triple
from ravestate_nlp.question_words import QuestionWord
import ravestate_ontology

from spacy.tokens import Token

from reggol import get_logger
logger = get_logger(__name__)

from ravestate_verbaliser import verbaliser
from os.path import realpath, dirname, join

verbaliser.add_folder(join(dirname(realpath(__file__)), "answering_phrases"))

@state(triggers=s(":startup"), write="rawio:out")
def hello_world_roboyqa(ctx):
    ctx["rawio:out"] = "Ask me something about myself!"

@state(triggers=s("nlp:triples:changed"), read="nlp:triples", write="rawio:out")
def roboyqa(ctx):
    sess = ravestate_ontology.get_session()
    question_subject = ctx["nlp:triples"][0].get_subject()
    question_predicate = ctx["nlp:triples"][0].get_predicate()
    question_predicate_subplement = ctx["nlp:triples"][0].get_predicate_subplement()
    question_object = ctx["nlp:triples"][0].get_object()

    #TODO ORDER
    #FRIEND_OF
    #full_name
    #future
    #MEMBER_OF
    #LIVE_IN
    if question_object.text == QuestionWord._object:
        if question_subject.text == "age":
            ctx["rawio:out"] =  verbaliser.get_random_successful_answer("age") % "one"
        elif question_predicate.text == "do":
            ctx["rawio:out"] =  verbaliser.get_random_successful_answer("skills") % "drink"
            #abilities
        elif question_predicate.text == "like":
            ctx["rawio:out"] =  verbaliser.get_random_successful_answer("HAS_HOBBY") % "beer"
    elif question_subject.text == QuestionWord._person:
        if question_object.text == "father" or question_object.text == "dad":
            ctx["rawio:out"] =  verbaliser.get_random_successful_answer("CHILD_OF") % "raf"
        elif question_object.text == "brother" or question_object.text == "sibling":
            ctx["rawio:out"] = verbaliser.get_random_successful_answer("SIBLING_OF") % "roboy junior"
    elif question_object.text == QuestionWord._place:
        if question_subject.text == "you":
            ctx["rawio:out"] = verbaliser.get_random_successful_answer("FROM") % "munich"
    elif question_object.text == QuestionWord._form:
        if question_predicate_subplement and question_predicate_subplement.text == "old":
            ctx["rawio:out"] =  verbaliser.get_random_successful_answer("age") % "one"
        else: 
            ctx["rawio:out"] = "I'm schwifty."
            #add answers to how are you in info list

registry.register(
    name="roboyqa",
    states=(hello_world_roboyqa, roboyqa)
)