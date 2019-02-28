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
                logger.error(key)
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
        # TODO when inference detects answer, reset predicate


    @state(cond=s("persqa:answer:changed"),
           write=("rawio:out", "interloc:all"),
           read=("persqa:predicate", "persqa:subject", "persqa:answer", "interloc:all"),
           signal_name="reacted")
    def react(ctx):
        """
        retrieves memory node with the name or creates a new one
        outputs a polite response
        """
        interloc_answer = str(ctx["persqa:answer"])
        sess: Session = ravestate_ontology.get_session()
        onto: Ontology = ravestate_ontology.get_ontology()
        # subject_node = Node(node=ctx[ctx["persqa:subject"]])
        subject_node: Node = ctx[ctx["persqa:subject"]]
        if not subject_node.get_name():
            subject_node.set_properties({"name": interloc_answer})
        elif ctx["persqa:predicate"] in PREDICATE_SET:
            pass
            # TODO see if that is already a node... if not create....
            # subject_node.add_relationships({ctx["persqa:predicate"]: interloc_answer})
        node_list = sess.retrieve(subject_node)
        if not node_list and subject_node.get_type().entity == onto.get_type("Person").entity:
            interloc_node = sess.update(subject_node)
            output = verbaliser.get_random_successful_answer(ctx["persqa:predicate"]) % interloc_answer
        elif len(node_list) == 1:
            interloc_node = node_list[0]
            output = verbaliser.get_random_followup_answer(ctx["persqa:predicate"]) % interloc_answer
        else:
            logger.error("Found more than one node with that name!")
            interloc_node = None
            output = verbaliser.get_random_failure_answer(ctx["persqa:predicate"])
        logger.info(f"Interlocutor: Answer = {answer}; Node ID = {interloc_node.get_id()} ")
        ctx["rawio:out"] = output
        return Emit()
