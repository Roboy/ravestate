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
        interloc = ctx["interloc:all:pushed"]

        @state(cond=s("persqa:new-interloc", max_age=-1, detached=True),
               write=("rawio:out", "persqa:subject", "persqa:predicate"))
        def ask_name(ctx):
            """
            reacts to idle:bored & persqa:new-interloc
            asks for interlocutors name
            """
            ctx["persqa:predicate"] = "NAME"
            ctx["rawio:out"] = verbaliser.get_random_question("NAME")
            ctx["persqa:subject"] = interloc
        mod.add(ask_name)
        ctx.add_state(ask_name)
        return Emit()


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
        # TODO when inference detects answer, reset predicate 


    @state(cond=s("persqa:answer:changed"),
           write=("rawio:out", "interloc:all"),
           read=("persqa:predicate", "persqa:subject", "persqa:answer", "interloc:all"))
    def react(ctx):
        """
        retrieves memory node with the name or creates a new one
        outputs a polite response
        """
        name = str(ctx["persqa:answer"])
        sess: Session = ravestate_ontology.get_session()
        onto: Ontology = ravestate_ontology.get_ontology()
        subject_node = Node(node=ctx[ctx["persqa:subject"]])
        subject_node.set_properties({"name": name})
        node_list = sess.retrieve(subject_node)
        if not node_list and subject_node.get_type().entity == onto.get_type("Person").entity:
            interloc_node = sess.update(subject_node)
            output = verbaliser.get_random_successful_answer(ctx["persqa:predicate"]) % name
        elif len(node_list) == 1:
            interloc_node = node_list[0]
            output = verbaliser.get_random_followup_answer(ctx["persqa:predicate"]) % name
        else:
            logger.error("Failed to create node!")
            interloc_node = None
            output = verbaliser.get_random_failure_answer(ctx["persqa:predicate"])
        logger.info(f"Interlocutor: Name = {name}; Node ID = {interloc_node.get_id()} ")
        ctx["rawio:out"] = output


