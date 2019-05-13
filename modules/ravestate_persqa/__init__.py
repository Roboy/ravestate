import ravestate as rs
import ravestate_verbaliser as verbaliser
import ravestate_ontology as mem
import ravestate_idle as idle
import ravestate_rawio as rawio
import ravestate_nlp as nlp
import ravestate_interloc as interloc

from os.path import realpath, dirname, join
from typing import Dict, Set
from collections import defaultdict
import random

from scientio.ontology.ontology import Ontology
from scientio.session import Session
from scientio.ontology.node import Node

from reggol import get_logger
logger = get_logger(__name__)

verbaliser.add_folder(join(dirname(realpath(__file__)), "persqa_phrases"))

PREDICATE_SET = {"FROM", "HAS_HOBBY", "LIVE_IN", "FRIEND_OF", "STUDY_AT", "MEMBER_OF", "WORK_FOR", "OCCUPIED_AS"}
ONTOLOGY_TYPE_FOR_PRED = defaultdict(str, {
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


with rs.Module(name="persqa") as mod:

    # This is a nice demo of using properties as synchronization
    #  primitives. The problem: inference must only run for inputs
    #  that did not trigger new_interloc. Therefore, new_interloc and inference
    #  are mutually exclusive. This is enfored by having both of them
    #  consume the inference_mutex property.
    prop_inference_mutex = rs.Property(name="inference_mutex")

    prop_subject = rs.Property(
        name="subject",
        default_value="",
        always_signal_changed=True,
        allow_pop=False,
        allow_push=False,
        is_flag_property=True)

    prop_predicate = rs.Property(
        name="predicate",
        default_value="",
        always_signal_changed=True,
        allow_pop=False,
        allow_push=False,
        is_flag_property=True)

    prop_answer = rs.Property(
        name="answer",
        default_value="",
        always_signal_changed=True,
        allow_pop=False,
        allow_push=False,
        is_flag_property=True)

    prop_follow_up_prop = rs.Property(
        name="follow_up",
        default_value="",
        always_signal_changed=True,
        allow_pop=False,
        allow_push=False)

    sig_follow_up = rs.Signal(name="follow-up")

    sig_predicate_asked = rs.Signal(name="predicate-asked")

    def find_empty_relationship(dictonary: Dict):
        for key in dictonary:
            if not dictonary[key] and key in PREDICATE_SET:
                return key
        return None

    def retrieve_or_create_node(node: Node):
        sess: Session = mem.get_session()
        node_list = sess.retrieve(node)
        if not node_list:
            node = sess.create(node)
        elif len(node_list) > 0:
            node = node_list[0]
        return node

    def create_small_talk_states(ctx: rs.ContextWrapper, interloc_path: str):

        used_follow_up_preds = set()

        @rs.state(
            cond=idle.sig_bored,
            write=(rawio.prop_out, prop_predicate, prop_subject),
            read=(interloc_path, prop_predicate),
            weight=1.2,
            cooldown=40.,
            emit_detached=True,
            signal=sig_follow_up)
        def small_talk(ctx: rs.ContextWrapper):
            sess: Session = mem.get_session()
            interloc: Node = ctx[interloc_path]
            if interloc.get_id() < 0:  # ask for name, if the interlocutor is not (yet) a persistent instance
                pred = "NAME"
            else:
                pred = find_empty_relationship(interloc.get_relationships())
            ctx[prop_subject] = interloc_path
            if not ctx[prop_predicate]:
                if pred:
                    logger.info(f"Personal question: intent={pred}")
                    ctx[prop_predicate] = pred
                    ctx[rawio.prop_out] = verbaliser.get_random_question(pred)
                else:
                    unused_fup_preds = PREDICATE_SET.difference(used_follow_up_preds)
                    if not unused_fup_preds:
                        logger.info(f"Ran out of smalltalk predicates for {interloc_path}, committing suicide...")
                        return rs.Delete(resign=True)
                    pred = random.sample(PREDICATE_SET.difference(used_follow_up_preds), 1)
                    pred = pred[0]
                    used_follow_up_preds.add(pred)
                    ctx[prop_predicate] = pred
                    relationship_ids: Set[int] = interloc.get_relationships(pred)
                    if len(relationship_ids) > 0:  # Just to be safe ...
                        object_node_list = sess.retrieve(node_id=list(relationship_ids)[0])
                        if len(object_node_list) > 0:
                            ctx[rawio.prop_out] = verbaliser.get_random_followup_question(pred).format(
                                name=interloc.get_name(),
                                obj=object_node_list[0].get_name())
                            logger.info(f"Follow-up: intent={pred}")
                            return rs.Emit()
                    return rs.Resign()
            else:
                # While the predicate is set, repeat the question. Once the predicate is answered,
                #  it will be set to None, such that a new predicate is entered.
                ctx[rawio.prop_out] = verbaliser.get_random_question(ctx[prop_predicate])

        @rs.state(
            cond=sig_follow_up.max_age(-1.) & nlp.prop_triples.changed(),
            write=(rawio.prop_out, prop_predicate, prop_inference_mutex),
            read=(interloc_path, prop_predicate, nlp.prop_yesno))
        def fup_react(ctx: rs.ContextWrapper):
            sess: Session = mem.get_session()
            subject_node: Node = ctx[interloc_path]
            pred = ctx[prop_predicate]
            object_node_list = []
            relationship_ids: Set[int] = subject_node.get_relationships(pred)
            if len(relationship_ids) > 0:
                object_node_list = sess.retrieve(node_id=list(relationship_ids)[0])
            if len(object_node_list) > 0:
                ctx[rawio.prop_out] = verbaliser.get_random_followup_answer(pred).format(
                    name=subject_node.get_name(),
                    obj=object_node_list[0].get_name())
            else:
                ctx[rawio.prop_out] = "Oh, I see!"
            ctx[prop_predicate] = None

        ctx.add_state(small_talk)
        ctx.add_state(fup_react)

    @rs.state(
        cond=rawio.prop_in.changed() & interloc.prop_all.pushed(),
        write=prop_inference_mutex,
        read=interloc.prop_all)
    def new_interloc(ctx: rs.ContextWrapper):
        """
        reacts to interloc:pushed and creates persqa:ask_name state
        """
        interloc_path = ctx[interloc.prop_all.pushed()]
        create_small_talk_states(ctx=ctx, interloc_path=interloc_path)

    @rs.state(
        cond=rawio.prop_in.changed() & interloc.prop_all.popped(),
        write=(prop_inference_mutex, prop_predicate, prop_subject))
    def removed_interloc(ctx: rs.ContextWrapper):
        """
        reacts to interloc:popped and makes sure that
        """
        ctx[prop_subject] = None
        ctx[prop_predicate] = None

    @rs.state(signal=sig_predicate_asked, read=prop_predicate)
    def check_predicate_asked(ctx):
        if ctx[prop_predicate]:
            return rs.Emit()
        else:
            return rs.Wipe()

    @rs.state(
        # optionally acquire sig_predicate_asked, such that active engagement does not run when a predicate was asked
        cond=nlp.prop_triples.changed() | (sig_predicate_asked.max_age(-1) & nlp.prop_triples.changed()),
        write=(prop_answer, prop_inference_mutex),
        read=(prop_predicate, nlp.prop_triples, nlp.prop_tokens, nlp.prop_yesno))
    def inference(ctx: rs.ContextWrapper):
        """
        recognizes name in sentences like:
         - i am jj
         - my name toseban
         - dino
        """
        triple = ctx[nlp.prop_triples][0]
        if triple.is_question():
            return rs.Resign()
        pred = ctx[prop_predicate]
        answer_str = None
        if pred == "NAME" or pred in PREDICATE_SET:
            # TODO City, Country -> NLP NER also only recognizes locations...
            if triple.has_object():
                answer_str = triple.get_object().text
            elif len(ctx[nlp.prop_tokens]) == 1:
                answer_str = ctx[nlp.prop_tokens][0]
            elif len(ctx[nlp.prop_tokens]) == 2:
                answer_str = "%s %s" % (ctx[nlp.prop_tokens][0], ctx[nlp.prop_tokens][1])
        if answer_str:
            logger.debug(f"Inference: extracted answer '{answer_str}' for predicate {pred}")
            ctx[prop_answer] = answer_str

    @rs.state(
        cond=prop_answer.changed(),
        write=(rawio.prop_out, prop_predicate),
        read=(prop_predicate, prop_subject, prop_answer, interloc.prop_all))
    def react(ctx: rs.ContextWrapper):
        """
        Retrieves memory node with the name, or creates a new one
        outputs a polite response.
        """
        onto: Ontology = mem.get_ontology()
        sess: Session = mem.get_session()
        inferred_answer = ctx[prop_answer]
        pred = ctx[prop_predicate]
        subject_path: str = ctx[prop_subject]
        if not subject_path:
            return rs.Resign()
        subject_node: Node = ctx[subject_path]
        assert inferred_answer

        if pred == "NAME":
            # If name was asked, it must be because the node is not yet persistent
            assert subject_node.get_id() < 0
            subject_node.set_name(inferred_answer)
            persistent_subject_node = retrieve_or_create_node(subject_node)
            # TODO: Workaround - see #83 - if this state would write to interloc:all,
            #  it would promise an interloc:all:pushed signal. This would be
            #  picked up by persqa:new_interloc.
            subject_node.set_node(persistent_subject_node)
            sess.update(subject_node)
        elif pred in PREDICATE_SET:
            relationship_type = onto.get_type(ONTOLOGY_TYPE_FOR_PRED[pred])
            relationship_node = Node(metatype=relationship_type)
            relationship_node.set_name(inferred_answer)
            relationship_node = retrieve_or_create_node(relationship_node)
            if relationship_node is not None:
                subject_node.add_relationships({pred: {relationship_node.get_id()}})
                sess.update(subject_node)

        ctx[rawio.prop_out] = verbaliser.get_random_successful_answer(pred).format(
            name=subject_node.get_name(),
            obj=inferred_answer)
        ctx[prop_predicate] = None


