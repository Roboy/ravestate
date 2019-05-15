import ravestate as rs
import ravestate_nlp as nlp
import ravestate_verbaliser as verbaliser
import ravestate_idle as idle
import ravestate_rawio as rawio
import ravestate_ontology

from scientio.ontology.ontology import Ontology
from scientio.session import Session
from scientio.ontology.node import Node

from os.path import realpath, dirname, join
import random
import datetime

from reggol import get_logger
logger = get_logger(__name__)

verbaliser.add_folder(join(dirname(realpath(__file__)), "answering_phrases"))

ROBOY_NODE_PROP_CONF_KEY = "roboy_node_properties"


with rs.Module(name="roboyqa", config={ROBOY_NODE_PROP_CONF_KEY: {"name": "roboy two"}}):

    @rs.state(cond=idle.sig_bored, write=rawio.prop_out, weight=0.6, cooldown=30.)
    def hello_world_roboyqa(ctx):
       ctx[rawio.prop_out] = "Ask me something about myself!"

    @rs.state(cond=nlp.sig_contains_roboy & nlp.sig_is_question, read=nlp.prop_triples, write=rawio.prop_out)
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
        sess: Session = ravestate_ontology.get_session()
        onto: Ontology = ravestate_ontology.get_ontology()

        roboy = Node(metatype=onto.get_type("Robot"))
        roboy.set_properties(ctx.conf(key=ROBOY_NODE_PROP_CONF_KEY))
        node_list = sess.retrieve(request=roboy)
        if node_list:
            roboy = node_list[0]
        else:
            create_default_nodes()
            node_list = sess.retrieve(request=roboy)
            if node_list:
                roboy = node_list[0]
            else:
                logger.error(f"Seems like you do not have my memory running, or no node with properties"
                             f"{ctx.conf(key=ROBOY_NODE_PROP_CONF_KEY)} exists!")
                return rs.Resign()
        
        triple = ctx[nlp.prop_triples][0]

        category = None
        memory_info = None

        # question word: What?
        if triple.is_question(nlp.QuestionWord.OBJECT):
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
        elif triple.is_question(nlp.QuestionWord.PLACE):
            if triple.match_either_lemma(pred={"be"}):
                category = "FROM"
            elif triple.match_either_lemma(pred={"live"}):
                category = "LIVE_IN"

        # question word: Who?
        elif triple.is_question(nlp.QuestionWord.PERSON):
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
        elif triple.is_question(nlp.QuestionWord.FORM):
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
            node_id = random.sample(roboy.get_relationships(key=category), 1)[0]
            memory_info = sess.retrieve(node_id=int(node_id))[0].get_name()

        elif category and category.islower() and not isinstance(roboy.get_properties(key=category), dict):
            property_list = [x.strip() for x in roboy.get_properties(key=category).split(',')]
            memory_info = random.sample(property_list, 1)[0]

        if memory_info:
            ctx[rawio.prop_out] = verbaliser.get_random_successful_answer("roboy_"+category) % memory_info
        elif category == "well_being":
            ctx[rawio.prop_out] = verbaliser.get_random_successful_answer("roboy_"+category)
        else:
            return rs.Resign()


def create_default_nodes():
    onto: Ontology = ravestate_ontology.get_ontology()
    sess = ravestate_ontology.get_session()

    organization_type = onto.get_type("Organization")
    city_type = onto.get_type("City")
    person_type = onto.get_type("Person")
    robot_type = onto.get_type("Robot")
    hobby_type = onto.get_type("Hobby")

    student_team_node = Node(metatype=organization_type)
    student_team_node.set_properties({
        "name": "roboy student team"})

    munich_node = Node(metatype=city_type)
    munich_node.set_properties({
        "name": "munich"})

    roboy_junior_node = Node(metatype=robot_type)
    roboy_junior_node.set_properties({
        "name": "roboy",
        "abilities": "talking to people,reading wikipedia,reading reddit",
        "skills": "tell jokes,tell fun facts about famous people and places,recognize you next time we meet,"
                  "remember your name and our conversation",
        "speed_kmh": 0,
        "full_name": "roboy junior",
        "birthdate": "08.03.2013",
        "conversation_id": "#bitch_boy",
        "sex": "male",
        "age": 5})

    raf_node = Node(metatype=person_type)
    raf_node.set_properties({
        "name": "rafael",
        "full_name": "rafael hostettler"})

    lucy_node = Node(metatype=person_type)
    lucy_node.set_properties({
        "name": "lucy",
        "age": 6,
        "sex": "female"})

    reddit_node = Node(metatype=hobby_type)
    reddit_node.set_properties({
        "name": "reading reddit"})

    roboy_node = Node(metatype=robot_type)
    roboy_node.set_properties({
        "name": "roboy two",
        "skills": "telling jokes,telling fun facts,telling you about the famous people and places,doing the math",
        "abilities": "talk to people,recognize objects,show emotions,move my body,shake a hand,"
                     "party like there is no tomorrow,surf the internet,answer your questions",
        "speed_kmh": 0,
        "birthdate": "12.04.2018",
        "full_name": "roboy 2.0",
        "conversation_id": "#bitch_boy",
        "future": "become a robot rights advocate,help people and robots become friends,"
                  "find the answer to the question of life the universe and everything,"
                  "visit mars and other planets,become a music star,become a michelin star chef,"
                  "get a robo pet,become as good as my father",
        "sex": "male",
        "age": "0"})

    tricycle_node = Node(metatype=hobby_type)
    tricycle_node.set_properties({
        "name": "riding a tricycle"})

    nodes = (
        student_team_node,
        munich_node,
        roboy_junior_node,
        raf_node,
        lucy_node,
        reddit_node,
        roboy_node,
        tricycle_node)
    
    for node in nodes:
        sess.create(node)

    roboy_junior_node.add_relationships({"HAS_HOBBY":  {reddit_node.get_id()}})
    roboy_junior_node.add_relationships({"MEMBER_OF":  {student_team_node.get_id()}})
    roboy_junior_node.add_relationships({"CHILD_OF":   {raf_node.get_id()}})
    roboy_junior_node.add_relationships({"LIVE_IN":    {munich_node.get_id()}})
    raf_node.add_relationships({"FRIEND_OF":           {roboy_node.get_id(), roboy_junior_node.get_id()}})
    raf_node.add_relationships({"WORK_FOR":            {student_team_node.get_id()}})
    lucy_node.add_relationships({"FRIEND_OF":          {roboy_node.get_id(), roboy_junior_node.get_id()}})
    roboy_node.add_relationships({"FROM":              {munich_node.get_id()}})
    roboy_node.add_relationships({"SIBLING_OF":        {roboy_junior_node.get_id()}})
    roboy_node.add_relationships({"HAS_HOBBY":         {tricycle_node.get_id(), reddit_node.get_id()}})
    roboy_node.add_relationships({"MEMBER_OF":         {student_team_node.get_id()}})
    roboy_node.add_relationships({"CHILD_OF":          {raf_node.get_id()}})
    roboy_node.add_relationships({"LIVE_IN":           {munich_node.get_id()}})

    for node in nodes:
        sess.update(node)


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
