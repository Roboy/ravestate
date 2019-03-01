from ravestate.module import Module
from ravestate.property import PropertyBase
from ravestate.state import state, Emit, Delete
from ravestate.constraint import s
from ravestate_verbaliser import verbaliser
import ravestate_ontology

from os.path import realpath, dirname, join
from typing import Dict
import random

from scientio.ontology.ontology import Ontology
from scientio.session import Session
from scientio.ontology.node import Node
from reggol import get_logger
logger = get_logger(__name__)

import ravestate_idle
import ravestate_nlp
import ravestate_rawio

verbaliser.add_folder(join(dirname(realpath(__file__)), "persqa_phrases"))

PREDICATE_SET = {"FROM", "HAS_HOBBY", "LIVE_IN", "FRIEND_OF", "STUDY_AT", "MEMBER_OF", "WORK_FOR", "OCCUPIED_AS"}
# TODO "movies", "OTHER"
# TODO "sex", "birth date", "full name"
# TODO "EQUALS"

with Module(name="persqa") as mod:

    subject = PropertyBase(
        name="subject",
        default_value="",
        always_signal_changed=True,
        allow_pop=False,
        allow_push=False,
        is_flag_property=True)

    predicate = PropertyBase(
        name="predicate",
        default_value="",
        always_signal_changed=True,
        allow_pop=False,
        allow_push=False,
        is_flag_property=True)

    answer = PropertyBase(
        name="answer",
        default_value="",
        always_signal_changed=True,
        allow_pop=False,
        allow_push=False,
        is_flag_property=True)


    @state(cond=s("interloc:all:pushed"),
           signal_name="new-interloc",
           emit_detached=True)
    def new_interloc(ctx):
        """
        reacts to interloc:pushed and creates persqa:ask_name state
        """
        interloc_path = ctx["interloc:all:pushed"]

        @state(cond=s("persqa:new-interloc", max_age=-1, detached=True),
               write=("rawio:out", "persqa:subject", "persqa:predicate"))
        def ask_name(ctx):
            """
            reacts to idle:bored & persqa:new-interloc
            asks for interlocutors name
            """
            ctx["persqa:predicate"] = "NAME"
            ctx["rawio:out"] = verbaliser.get_random_question("NAME")
            ctx["persqa:subject"] = interloc_path

        # TODO only trigger when also bored?
        @state(cond=s("persqa:reacted", max_age=-1, detached=True),
               write=("rawio:out", "persqa:predicate", "persqa:subject"),
               read="interloc:all")
        def ask_other_stuff(ctx):
            # interloc: Node = ctx[ctx["persqa:subject"]]
            interloc: Node = ctx[interloc_path]
            relationships = interloc.get_relationships()
            key = find_empty_entry(relationships)
            if key:
                ctx["persqa:predicate"] = key
            else:
                key = random.sample(PREDICATE_SET, 1)
                ctx["persqa:predicate"] = key
            ctx["rawio:out"] = verbaliser.get_random_question(key)
            ctx["persqa:subject"] = interloc_path

        mod.add(ask_name)
        ctx.add_state(ask_name)
        mod.add(ask_other_stuff)
        ctx.add_state(ask_other_stuff)
        return Emit()


    def find_empty_entry(dictonary: Dict):
        for key in dictonary:
            if not dictonary[key] and key in PREDICATE_SET:
                return key
        return None


    @state(cond=s("nlp:triples:changed"),
           write="persqa:answer",
           read=("persqa:predicate", "nlp:triples", "nlp:tokens"))
    def inference(ctx):
        """
        recognizes name in sentences like:
         - i am jj
         - my name toseban
         - dino
        """
        triple = ctx["nlp:triples"][0]
        if ctx["persqa:predicate"] == "NAME" or ctx["persqa:predicate"] in PREDICATE_SET:
            ctx["persqa:answer"] = triple.get_object()
            if len(ctx["nlp:tokens"]) == 1:
                ctx["persqa:answer"] = ctx["nlp:tokens"][0]
            elif len(ctx["nlp:tokens"]) == 2:
                ctx["persqa:answer"] = "%s %s" % (ctx["nlp:tokens"][0], ctx["nlp:tokens"][1])
        # TODO when inference detects answer, reset predicate
        # TODO check if interloc only says no


    @state(cond=s("persqa:answer:changed"),
           write=("rawio:out", "interloc:all", "persqa:predicate"),
           read=("persqa:predicate", "persqa:subject", "persqa:answer", "interloc:all"),
           signal_name="reacted")
    def react(ctx):
        """
        retrieves memory node with the name or creates a new one
        outputs a polite response
        """
        onto: Ontology = ravestate_ontology.get_ontology()
        interloc_answer = str(ctx["persqa:answer"])
        # subject_node = Node(node=ctx[ctx["persqa:subject"]])
        subject_node: Node = ctx[ctx["persqa:subject"]]
        if subject_node.get_name() == "x":
            subject_node.set_name(interloc_answer)
            subject_node, output = generate_ouput(subject_node, ctx["persqa:predicate"], interloc_answer)
            ctx["rawio:out"] = output
        elif ctx["persqa:predicate"] in PREDICATE_SET:
            if ctx["persqa:predicate"] == "FROM" or ctx["persqa:predicate"] == "LIVE_IN":
                relationship_node = Node(metatype=onto.get_type("Location"))  # City, Country
            elif ctx["persqa:predicate"] == "HAS_HOBBY":
                relationship_node = Node(metatype=onto.get_type("Hobby"))
            elif ctx["persqa:predicate"] == "FRIEND_OF":
                relationship_node = Node(metatype=onto.get_type("Person"))  # Robot
            elif ctx["persqa:predicate"] == "STUDY_AT":
                relationship_node = Node(metatype=onto.get_type("University"))
            elif ctx["persqa:predicate"] == "MEMBER_OF":
                relationship_node = Node(metatype=onto.get_type("Organization"))
            elif ctx["persqa:predicate"] == "WORK_FOR":
                relationship_node = Node(metatype=onto.get_type("Company"))
            elif ctx["persqa:predicate"] == "OCCUPIED_AS":
                relationship_node = Node(metatype=onto.get_type("Occupation"))  # Job
            else:
                relationship_node = Node()
            relationship_node.set_name(interloc_answer)
            relationship_node, output = generate_ouput(relationship_node, ctx["persqa:predicate"], interloc_answer)
            ctx["rawio:out"] = output
            if relationship_node is not None:
                subject_node.add_relationships({ctx["persqa:predicate"]: {relationship_node.get_id()}})
        ctx["persqa:predicate"] = None
        return Emit()


    def generate_ouput(node: Node, intent: str, interloc_answer: str):
        sess: Session = ravestate_ontology.get_session()
        node_list = sess.retrieve(node)
        if not node_list:
            node = sess.update(node)
            output = verbaliser.get_random_successful_answer(intent) % interloc_answer
        elif len(node_list) == 1:
            node = node_list[0]
            output = verbaliser.get_random_followup_answer(intent) % interloc_answer
        else:
            logger.error("Found more than one node with that name!")
            node = None
            output = verbaliser.get_random_failure_answer(intent)
        if node is not None:
            logger.info(f"Interlocutor: Answer = {answer}; Node ID = {node.get_id()} ")
        return node, output
