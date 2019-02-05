from ravestate.module import Module
from ravestate.property import PropertyBase
from ravestate.state import state, Emit, Delete
from ravestate.constraint import s
from ravestate_verbaliser import verbaliser

from os.path import realpath, dirname, join

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
        Emit()

        @state(cond=s("idle:bored") | s("perqa:new-interloc", detached=True),
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
           read=("persqa:predicate", "nlp:triples"))
    def inference(ctx):
        triple = ctx["nlp:triples"][0]
        if ctx["persqa:predicate"] == "NAME":
                ctx["persqa:answer"] = triple.get_object() if triple.get_object() else None


    @state(cond=s("persqa:answer:changed"),
           write="rawio:out",
           read=("persqa:predicate", "persqa:subject", "persqa:answer"))
    def react(ctx):
        ctx["rawio:out"] = verbaliser.get_random_successful_answer(ctx["persqa:predicate"])
