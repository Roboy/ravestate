from .api import Api

import ravestate as rs
import ravestate_idle as idle
import ravestate_nlp as nlp
import ravestate_rawio as rawio

from reggol import get_logger
logger = get_logger(__name__)

akinator_api: Api
CERTAINTY = "certainty_percentage"


with rs.Module(name="akinator", config={CERTAINTY: 90}):

    prop_is_it = rs.Property(
        name="is_it",
        default_value="",
        always_signal_changed=True,
        allow_pop=False,
        allow_push=False,
        is_flag_property=True)

    prop_question = rs.Property(
        name="question",
        default_value="",
        always_signal_changed=True,
        allow_pop=False,
        allow_push=False,
        is_flag_property=True)

    prop_init_play_again = rs.Property(
        name="initiate_play_again",
        default_value="",
        always_signal_changed=True,
        allow_pop=False,
        allow_push=False,
        is_flag_property=True)

    sig_initiate_play = rs.Signal("initiate-play")
    sig_exit_game = rs.Signal("exit-game")
    sig_wrong_input = rs.Signal("wrong-input")
    sig_is_it_asked = rs.Signal("is-it")

    @rs.state(
        cond=nlp.sig_intent_play | idle.sig_bored,
        write=rawio.prop_out,
        signal=sig_initiate_play,
        emit_detached=True,
        weight=1.,
        cooldown=30.)
    def play_ask(ctx):
        """
        Asks if interlocutor wants to play 20 question / akinator
        Triggered when nlp:play property is changed by "i want to play a game" or a similar input
        """
        ctx[rawio.prop_out] = "Do you want to play 20 questions?"
        return rs.Emit()


    @rs.state(cond=sig_initiate_play.min_age(10).max_age(13), write=rawio.prop_out)
    def akinator_play_question_ignored(ctx):
        ctx[rawio.prop_out] = "Oh well, maybe later!"

    @rs.state(
        cond=nlp.prop_yesno.changed() & (sig_initiate_play.max_age(13) | prop_init_play_again.changed().max_age(13.)),
        read=nlp.prop_yesno,
        write=(rawio.prop_out, "akinator:question"),
        emit_detached=True)
    def start(ctx):
        """
        Starts akinator session
        Triggered by signal from akinator_play_ask state and nlp:yesno:changed given by the input
        It is also triggered if one game is finished and the interlocutor wants to play again
        Output: First question
        """
        global akinator_api
        if ctx[nlp.prop_yesno] == "yes":
            logger.info("Akinator session is started.")
            akinator_api = Api()
            ctx[prop_question] = True
            ctx[rawio.prop_out] = "You can answer the questions with:" \
                               + ' "yes", "no", "i do not know", "probably", "probably not"' \
                               + " Question " + str(int(akinator_api.get_parameter('step')) + 1) \
                               + ": " + akinator_api.get_parameter('question')
        else:
            return rs.Resign()


    @rs.state(
        cond=nlp.prop_triples.changed() & prop_question.changed().max_age(13),
        read=nlp.prop_yesno,
        write=(rawio.prop_out, prop_is_it, prop_question),
        emit_detached=True,
        signal=sig_wrong_input)
    def question_answered(ctx):
        """
        Reads the answer to a question and outputs the next question
        Gets triggered by akinator:question which is always updated by a new question and nlp:yesno:changed signal
        Triggers the wrong_input state id it cannot process the input
        If the answer certainty of akinator is over the configurable CERTAINTY threshold the is_it state is triggered
        """
        global akinator_api
        response = akinator_api.answer_to_int_str(ctx[nlp.prop_yesno])
        if not response == "-1":
            akinator_api.response_get_request(response)
            if float(akinator_api.get_parameter('progression')) <= ctx.conf(key=CERTAINTY) and int(akinator_api.get_parameter('step')) <= 20:
                ctx[prop_question] = True
                ctx[rawio.prop_out] = "Question " + str(int(akinator_api.get_parameter('step')) + 1) \
                                   + ": " + akinator_api.get_parameter('question')
            else:
                ctx["akinator:is_it"] = True
        else:
            return rs.Emit()


    @rs.state(
        cond=prop_is_it.changed(),
        write=rawio.prop_out,
        signal=sig_is_it_asked,
        emit_detached=True)
    def is_it(ctx):
        """
        Outputs the solution guess of akinator: "Is this your character? ..."
        Triggers the is_it_answer state
        """
        global akinator_api
        guess = akinator_api.guess_get_request()
        ctx[rawio.prop_out] = \
            f"Is this your character? {guess['name']}: {guess['desc']}." \
            " Please answer with 'yes' or 'no'."
        return rs.Emit()


    @rs.state(
        cond=nlp.prop_yesno.changed() & sig_is_it_asked.max_age(13),
        read=nlp.prop_yesno,
        write=(rawio.prop_out, "akinator:initiate_play_again"),
        emit_detached=True)
    def is_it_answered(ctx):
        """
        Gets input from interlocutor on the "is it" question and posts the result
        Asks if the interlocutor wants to play again.
        """
        if ctx[nlp.prop_yesno] == "yes":
            akinator_api.choice_get_request()
            out = "Yeah! I guessed right! Thanks for playing with me! Do you want to play again?"
        elif ctx[nlp.prop_yesno] == "no":
            akinator_api.exclusion_get_request()
            out = "Oh no! I guessed wrong but do you want to play again?"
        else:
            out = "What? But do you want to play again?"
        ctx[rawio.prop_out] = out
        ctx["akinator:initiate_play_again"] = True


    @rs.state(
        cond=sig_wrong_input,
        write=rawio.prop_out,
        emit_detached=True,
        signal=sig_exit_game)
    def wrong_input(ctx):
        """
        Catches wrong inputs from the interlocutor during questions answering and loops back to the question state
        """
        ctx[rawio.prop_out] = "Sadly I could not process that answer. Do you want to stop playing Akinator?"
        return rs.Emit()


    @rs.state(
        cond=nlp.prop_yesno.changed() & sig_exit_game.max_age(13),
        write=(rawio.prop_out, prop_question),
        read=nlp.prop_yesno,
        emit_detached=True)
    def exit_game_answered(ctx):
        if ctx[nlp.prop_yesno] == "yes":
            ctx[rawio.prop_out] = "Thanks for playing with me!"
            return rs.Resign()
        else:
            ctx[rawio.prop_out] = "Yeah! Let's keep playing! Remember that you have these five answering choices:" \
                           " 'yes', 'no', 'i do not know', 'probably', 'probably not'"
            ctx[prop_question] = True
