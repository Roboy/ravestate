from ravestate.module import Module
from ravestate.property import PropertyBase
from ravestate.state import state, Emit, Delete
from ravestate.constraint import s
from ravestate_verbaliser import verbaliser
import ravestate_ontology

from os.path import realpath, dirname, join

from scientio.ontology.ontology import Ontology
from scientio.session import Session
from scientio.ontology.node import Node
from reggol import get_logger
logger = get_logger(__name__)

import ravestate_idle
import ravestate_nlp
import ravestate_rawio

verbaliser.add_folder(join(dirname(realpath(__file__)), "persqa_phrases"))

with Module(name="persqa"):

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
        return Emit()

    @state(cond=s("idle:bored") | s("persqa:new-interloc", max_age=-1, detached=True),
           write=("rawio:out", "persqa:subject", "persqa:predicate"),
           read="interloc:all")
    def ask_name(ctx):
        """
        reacts to idle:bored & persqa:new-interloc
        asks for interlocutors name
        """
        ctx["persqa:predicate"] = "NAME"
        ctx["rawio:out"] = verbaliser.get_random_question("NAME")
        ctx["persqa:subject"] = ctx["interloc:all"]
        return Delete()

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
        if ctx["persqa:predicate"] == "NAME":
            ctx["persqa:answer"] = triple.get_object()
            if len(ctx["nlp:tokens"]):
                ctx["persqa:answer"] = ctx["nlp:tokens"][0]


    @state(cond=s("persqa:answer:changed"),
           write="rawio:out",
           read=("persqa:predicate", "persqa:subject", "persqa:answer"))
    def react(ctx):
        """
        retrieves memory node with the name or creates a new one
        outputs a polite response
        """
        name = str(ctx["persqa:answer"])
        ctx["rawio:out"] = verbaliser.get_random_successful_answer(ctx["persqa:predicate"]) % name
        sess: Session = ravestate_ontology.get_session()
        onto: Ontology = ravestate_ontology.get_ontology()
        query = Node(metatype=onto.get_type("Person"))
        query.set_properties({"name": name})
        node_list = sess.retrieve(query)
        if not node_list:
            node = sess.create(query)
        elif len(node_list) == 1:
            node = node_list[0]
        else:
            logger.error("Failed to create node!")
            return
        logger.info(f"Interlocutor: Name = {name}; Node ID = {node.get_id()} ")

