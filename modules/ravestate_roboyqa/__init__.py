from ravestate.module import Module
from ravestate.state import state, Resign
from ravestate.constraint import s
from ravestate_nlp.question_word import QuestionWord
import ravestate_ontology
from ravestate_verbaliser import verbaliser

from os.path import realpath, dirname, join
import random
import datetime

import ravestate_idle

from reggol import get_logger
logger = get_logger(__name__)

verbaliser.add_folder(join(dirname(realpath(__file__)), "answering_phrases"))

ROBOY_NODE_CONF_KEY = "roboy_node_id"


with Module(name="roboyqa", config={ROBOY_NODE_CONF_KEY: 356}):

    @state(cond=s("idle:bored"), write="rawio:out", weight=1.1, cooldown=30.)
    def hello_world_roboyqa(ctx):
        ctx["rawio:out"] = "Ask me something about myself!"

    @state(cond=s("nlp:contains-roboy") & s("nlp:is-question"), read="nlp:triples", write="rawio:out")
    def roboyqa(ctx):
        """
        answers question regarding roboy by retrieving the information out of the neo4j roboy memory graph
        state gets triggered when nlp extracts a new triple: subject, predicate, object
        by analysing the triple the content of the question can be ascertained
        the necessary information is gathered using the neo4j memory session
        if the triple combination is known and the information could be retrieved an answer will be given

        list of questions that can be answered:
        - who are you?
        - what is your name?
        - how old are you?
        - what is your age?
        - what is your hobby?
        - what are your hobbies?
        - what do you like?
        - where are you from?
        - where do you live?
        - who is your father/dad?
        - who is your brother/sibling?
        - who is your friend?
        - what do you want to become?
        - what are you a member of?
        - what can you do?
        - what are your skills?
        - what have you learned?
        - what are your abilities?
        """
        sess = ravestate_ontology.get_session()
        roboy = sess.retrieve(node_id=ctx.conf(key=ROBOY_NODE_CONF_KEY))[0]
        triple = ctx["nlp:triples"][0]

        category = None
        memory_info = None

        # question word: What?
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
        # question word: Where?
        elif triple.is_question(QuestionWord.PLACE):
            if triple.match_either_lemma(pred={"be"}):
                category = "FROM"
            elif triple.match_either_lemma(pred={"live"}):
                category = "LIVE_IN"
        # question word: Who?
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
        # question word: How?
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
            return Resign()


def roboy_age(birth_date: str):
    """
    Calculates roboys age given his birth date
    example: birth date = "12.04.18"
    """
    birth_date = datetime.datetime.strptime(birth_date, "%d.%m.%Y")
    today = datetime.datetime.now()
    if today.year > birth_date.year and today.month > birth_date.month:
        age = "%d years" % (today.year - birth_date.year -
                            ((today.month, today.day) < (birth_date.month, birth_date.day)))
    else:
        age = "%d months" % (12 - birth_date.month+today.month)
    return age
