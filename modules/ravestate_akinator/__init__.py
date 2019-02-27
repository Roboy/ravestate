from ravestate.module import Module
from ravestate.property import PropertyBase
from ravestate.state import state, Resign, Emit, Delete
from ravestate.constraint import s
from ravestate_akinator.api import Api

from reggol import get_logger
logger = get_logger(__name__)

import ravestate_idle
import ravestate_nlp
import ravestate_rawio

akinator_api: Api
CERTAINTY = "certainty_percentage"

# TODO max 20 question

with Module(name="akinator", config={CERTAINTY: 90}):

    initiate_play_again = PropertyBase(
        name="initiate_play_again",
        default_value="",
        always_signal_changed=True,
        allow_pop=False,
        allow_push=False,
        is_flag_property=True)

    is_it = PropertyBase(
        name="is_it",
        default_value="",
        always_signal_changed=True,
        allow_pop=False,
        allow_push=False,
        is_flag_property=True)

    question = PropertyBase(
        name="question",
        default_value="",
        always_signal_changed=True,
        allow_pop=False,
        allow_push=False,
        is_flag_property=True)

    wrong_input = PropertyBase(
        name="wrong_input",
        default_value="",
        always_signal_changed=True,
        allow_pop=False,
        allow_push=False,
        is_flag_property=True)

    @state(cond=s("nlp:intent-play") | s("idle:bored"),
           write="rawio:out",
           signal_name="initiate-play",
           emit_detached=True,
           weight=1.,
           cooldown=30.)
    def play_ask(ctx):
        """
        Asks if interlocutor wants to play 20 question / akinator
        Triggered when nlp:play property is changed by "i want to play a game" or a similar input
        """
        ctx["rawio:out"] = "Do you want to play 20 questions?"
        return Emit()

    @state(cond=s("akinator:initiate-play", min_age=7., max_age=10.), write="rawio:out")
    def akinator_play_question_ignored(ctx):
        ctx["rawio:out"] = "Oh well, maybe later!"

    # TODO react to nlp:yesno:changed, remove nlo:yes-no signal
    @state(cond=s("nlp:yes-no") & (s("akinator:initiate-play", max_age=-1) | s("akinator:initiate_play_again:changed", max_age=-1)),
           read="nlp:yesno",
           write=("rawio:out", "akinator:question"),
           emit_detached=True)
    def start(ctx):
        """
        Starts akinator session
        Triggered by signal from akinator_play_ask state and nlp:yes-no signal given by the input
        It is also triggered if one game is finished and the interlocutor wants to play again
        Output: First question
        """
        global akinator_api
        if ctx["nlp:yesno"] == "yes":
            logger.info("Akinator session is started.")
            akinator_api = Api()
            ctx["akinator:question"] = True
            ctx["rawio:out"] = "You can answer the questions with:" \
                               + '\n"yes", "no", "i do not know", "probably", "probably not"' \
                               + "\nQuestion " + str(int(akinator_api.get_parameter('step')) + 1) \
                               + ":\n" + akinator_api.get_parameter('question')
        else:
            return Resign()


    @state(cond=s("nlp:yes-no") & s("akinator:question:changed", max_age=-1),
           read="nlp:yesno",
           write=("rawio:out", "akinator:is_it", "akinator:question", "akinator:wrong_input"),
           emit_detached=True)
    def question_answered(ctx):
        """
        Reads the answer to a question and outputs the next question
        Gets triggered by akinator:question which is always updated by a new question and nlp:yes-no signal
        Triggers the wrong_input state id it cannot process the input
        If the answer certainty of akinator is over the configurable CERTAINTY threshold the is_it state is triggered
        """
        global akinator_api
        response = akinator_api.answer_to_int_str(ctx["nlp:yesno"])
        if not response == "-1":
            akinator_api.response_get_request(response)
            if float(akinator_api.get_parameter('progression')) <= ctx.conf(key=CERTAINTY):
                ctx["akinator:question"] = True
                ctx["rawio:out"] = "Question " + str(int(akinator_api.get_parameter('step')) + 1) \
                                   + ":\n" + akinator_api.get_parameter('question')
            else:
                ctx["akinator:is_it"] = True
        else:
            ctx["akinator:wrong_input"] = True


    @state(cond=s("akinator:is_it:changed"),
           write="rawio:out",
           signal_name="is-it",
           emit_detached=True)
    def is_it(ctx):
        """
        Outputs the solution guess of akinator: "Is this your character? ..."
        Triggers the is_it_answer state
        """
        global akinator_api
        guess = akinator_api.guess_get_request()
        ctx["rawio:out"] = "Is this your character? \n" + guess['name'] + "\n" + guess['desc'] \
                           + "\nPlease answer with 'yes' or 'no'."
        return Emit()

    # TODO remove property initiate_play_again and emit signal instead
    @state(cond=s("nlp:yes-no") & s("akinator:is-it", max_age=-1),
           read="nlp:yesno",
           write=("rawio:out", "akinator:initiate_play_again"),
           emit_detached=True)
    def is_it_answered(ctx):
        """
        Gets input from interlocutor on the "is it" question and posts the result
        Asks if the interlocutor wants to play again.
        """
        if ctx["nlp:yesno"] == "yes":
            akinator_api.choice_get_request()
            out = "Yeah! I guessed right! Thanks for playing with me! \nDo you want to play again?"
        elif ctx["nlp:yesno"] == "no":
            akinator_api.exclusion_get_request()
            out = "I guessed wrong but do you want to play again?"
        else:
            # TODO catch wrong input for this state
            out = "What? But do you want to play again?"
        ctx["rawio:out"] = out
        ctx["akinator:initiate_play_again"] = True


    @state(cond=s("akinator:wrong_input:changed"),
           write=("rawio:out", "akinator:question"),
           emit_detached=True)
    def wrong_input(ctx):
        """
        Catches wrong inputs from the interlocutor during questions answering and loops back to the question state
        """
        ctx["rawio:out"] = "Sadly I could not process that answer. Remember that you have these five answering choices: " \
                           "\n'yes', 'no', 'i do not know', 'probably', 'probably not'"
        ctx["akinator:question"] = True
