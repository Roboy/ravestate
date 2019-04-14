from ravestate.module import Module
from ravestate.property import PropertyBase
from ravestate.wrappers import ContextWrapper
from ravestate.state import state, Emit, Delete
from ravestate.constraint import s
from ravestate_verbaliser import verbaliser
import ravestate_ontology

from os.path import realpath, dirname, join
from typing import Dict, Set
from collections import defaultdict
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
ONTOLOGY_TYPE_FOR_PRED = defaultdict(str).update({
    "FROM": "Location",
    "LIVE_IN": "Location",
    "HAS_HOBBY": "Hobby",
    "FRIEND_OF": "Person",  # TODO Robot
    "STUDY_AT": "University",
    "MEMBER_OF": "Organization",
    "WORK_FOR": "Company",
    "OCCUPIED_AS": "Occupation"
})
# TODO "movies", "OTHER"
# TODO "sex", "birth date", "full name"
# TODO "EQUALS"


with Module(name="persqa") as mod:

    # This is a nice demo of using properties as synchronization
    #  primitives. The problem: inference must only run for inputs
    #  that did not trigger new_interloc. Therefore, new_interloc and inference
    #  are mutually exclusive. This is enfored by having both of them
    #  consume the inference_mutex property.
    inference_mutex = PropertyBase(name="inference_mutex")

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

    follow_up = PropertyBase(
        name="follow_up",
        default_value="",
        always_signal_changed=True,
        allow_pop=False,
        allow_push=False)


    @state(cond=s("interloc:all:pushed") & s("rawio:in:changed"),
           write=inference_mutex.id(),
           read="interloc:all",
           emit_detached=True)
    def new_interloc(ctx: ContextWrapper):
        """
        reacts to interloc:pushed and creates persqa:ask_name state
        """
        interloc_path = ctx["interloc:all:pushed"]
        create_small_talk_state(ctx=ctx, interloc_path=interloc_path)

    def find_empty_relationship(dictonary: Dict):
        for key in dictonary:
            if not dictonary[key] and key in PREDICATE_SET:
                return key
        return None

    def create_small_talk_state(ctx: ContextWrapper, interloc_path: str):

        used_follow_up_preds = set()

        @state(cond=s("idle:bored", max_age=-1),
               write=("rawio:out", "persqa:predicate", "persqa:subject"),
               read=("interloc:all", "persqa:predicate"),
               weight=2,
               cooldown=30.,
               signal_name="follow-up"
               )
        def small_talk(ctx: ContextWrapper):
            sess: Session = ravestate_ontology.get_session()
            interloc: Node = ctx[interloc_path]
            if not interloc.get_name():
                pred = "NAME"
            else:
                pred = find_empty_relationship(interloc.get_relationships())
            ctx["persqa:subject"] = interloc_path
            if not ctx["persqa:predicate"]:
                if pred:
                    ctx["persqa:predicate"] = pred
                    ctx["rawio:out"] = verbaliser.get_random_question(pred)
                else:
                    pred = random.sample(PREDICATE_SET.difference(used_follow_up_preds), 1)
                    if not pred:
                        logger.info(f"Out of smalltalk predicates for {interloc_path}, committing suicide...")
                        return Delete()
                    pred = pred[0]
                    used_follow_up_preds.add(pred)
                    ctx["persqa:predicate"] = pred
                    relationship_ids: Set[int] = interloc.get_relationships(pred)
                    if len(relationship_ids) > 0:  # Just to be safe ...
                        node_list = sess.retrieve(node_id=list(relationship_ids)[0])
                        if len(node_list) > 0:
                            ctx["rawio:out"] = verbaliser.get_random_followup_question(pred).format(
                                name=interloc.get_name(),
                                obj=node_list[0].get_name())
                            logger.error(f"FOLLOW_UP: Intent = {key}")
                            return Emit()
            else:
                # While the predicate is set, repeat the question. Once the predicate is answered,
                #  it will be set to None, such that a new predicate is entered.
                ctx["rawio:out"] = verbaliser.get_random_question(ctx["persqa:predicate"])

        @state(cond=s("nlp:triples:changed") & s("persqa:follow-up"),
               write=("rawio:out", "persqa:predicate", inference_mutex.id()),
               read=("nlp:yesno", "persqa:predicate"))
        def fup_react(ctx: ContextWrapper):
            sess: Session = ravestate_ontology.get_session()
            subject_node: Node = ctx[interloc_path]
            pred = ctx["persqa:predicate"]
            node_list = []
            relationship_ids: Set[int] = subject_node.get_relationships(pred)
            if len(relationship_ids) > 0:
                relationship_node = Node()
                relationship_node.set_id(id=list(relationship_ids)[0])
                node_list = sess.retrieve(relationship_node)
            if ctx["nlp:yesno"] == "no" or len(node_list) == 0:
                ctx["rawio:out"] = "Oh, I see!"
            elif ctx["nlp:yesno"] == "yes":
                ctx["rawio:out"] = verbaliser.get_random_followup_answer(pred) % node_list[0].get_name()
                ctx["persqa:predicate"] = None

        ctx.add_state(small_talk)
        ctx.add_state(fup_react)


    @state(cond=s("nlp:triples:changed"),
           write=("persqa:answer", inference_mutex.id()),
           read=("persqa:predicate", "nlp:triples", "nlp:tokens", "nlp:yesno"))
    def inference(ctx: ContextWrapper):
        """
        recognizes name in sentences like:
         - i am jj
         - my name toseban
         - dino
        """
        triple = ctx["nlp:triples"][0]
        pred = ctx["persqa:predicate"]
        answer_str = None
        if pred == "NAME" or pred in PREDICATE_SET:
            # TODO City, Country -> NLP NER also only recognizes locations...
            if triple.has_object():
                answer_str = triple.get_object().text
            elif len(ctx["nlp:tokens"]) == 1:
                answer_str = ctx["nlp:tokens"][0]
            elif len(ctx["nlp:tokens"]) == 2:
                answer_str = "%s %s" % (ctx["nlp:tokens"][0], ctx["nlp:tokens"][1])
        if answer_str:
            logger.debug(f"Inference: extracted answer '{answer_str}' for predicate {pred}")
            ctx["persqa:answer"] = answer_str


    @state(cond=answer.changed_signal(),
           write=("rawio:out", "persqa:predicate"),
           read=("persqa:predicate", "persqa:subject", "persqa:answer", "interloc:all"))
    def react(ctx: ContextWrapper):
        """
        retrieves memory node with the name or creates a new one
        outputs a polite response
        """
        onto: Ontology = ravestate_ontology.get_ontology()
        sess: Session = ravestate_ontology.get_session()
        interloc_answer = str(ctx["persqa:answer"])
        pred = ctx["persqa:predicate"]
        subject_node: Node = ctx[ctx["persqa:subject"]]
        if pred == "NAME":
            subject_node.set_name(interloc_answer)
            subject_node, output = retrieve_node(subject_node, pred, interloc_answer)
            ctx["rawio:out"] = output
        elif pred in PREDICATE_SET:
            relationship_type = onto.get_type(ONTOLOGY_TYPE_FOR_PRED[pred])
            relationship_node = Node(metatype=relationship_type)
            relationship_node.set_name(interloc_answer)
            relationship_node, output = retrieve_node(relationship_node, pred, interloc_answer)
            ctx["rawio:out"] = output
            if relationship_node is not None:
                subject_node.add_relationships({pred: {relationship_node.get_id()}})
                sess.update(subject_node)
        ctx["persqa:predicate"] = None


    def retrieve_node(node: Node, intent: str, interloc_answer: str):
        sess: Session = ravestate_ontology.get_session()
        node_list = sess.retrieve(node)
        if not node_list:
            node = sess.create(node)
            output = verbaliser.get_random_successful_answer(intent) % interloc_answer
        elif len(node_list) == 1:
            node = node_list[0]
            output = verbaliser.get_random_followup_answer(intent) % interloc_answer
        else:
            logger.error("Found more than one node with that name!")
            node = None
            output = verbaliser.get_random_failure_answer(intent)
        if node is not None:
            logger.info(f"Re: Answer = {answer}; Node ID = {node.get_id()} ")
        return node, output

