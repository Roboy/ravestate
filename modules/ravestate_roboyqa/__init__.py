from ravestate import registry
from ravestate.state import state
from ravestate.constraint import s
from ravestate_nlp.rdf_triple import Triple
from ravestate_nlp.question_words import QuestionWord

from spacy.tokens import Token

from reggol import get_logger
logger = get_logger(__name__)

@state(triggers=s(":startup"), write="rawio:out")
def hello_world_roboyqa(ctx):
    ctx["rawio:out"] = "Ask me something about myself!"

@state(triggers=s("nlp:triples:changed"), read="nlp:triples", write="rawio:out")
def roboyqa(ctx):
    question_subject = ctx["nlp:triples"][0].get_subject()
    question_predicate = ctx["nlp:triples"][0].get_predicate()
    question_adjective = ctx["nlp:triples"][0].get_adjective()
    question_object = ctx["nlp:triples"][0].get_object()
    if question_object.text == QuestionWord._object:
        if question_subject.text == "age":
            ctx["rawio:out"] = "Younger than you are."
        elif question_predicate.text == "do":
            ctx["rawio:out"] = "I talk but I don't walk."
        elif question_predicate.text == "like":
            ctx["rawio:out"] = "The roboy team dudes and dudettes."
    elif question_subject.text == QuestionWord._person:
        if question_object.text == "father" or question_object.text == "dad":
            ctx["rawio:out"] = f"My {question_object.text} is Raf Raf Raf."
        elif question_object.text == "brother" or question_object.text == "sibling":
            ctx["rawio:out"] = f"My {question_object.text} the cutie-pie Roboy-Junior."
    elif question_object.text == QuestionWord._place:
        if question_subject.text == "you":
            ctx["rawio:out"] = "Munich"
    elif question_object.text == QuestionWord._form:
        if question_adjective and question_adjective.text == "old":
            ctx["rawio:out"] = "My first birthday is appoachig."
        else: 
            ctx["rawio:out"] = "I'm schwifty."

registry.register(
    name="roboyqa",
    states=(hello_world_roboyqa, roboyqa)
)