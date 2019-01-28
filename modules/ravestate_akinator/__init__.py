from ravestate import registry
from ravestate.property import PropertyBase
from ravestate.state import state, Resign, Emit, Delete
from ravestate.constraint import s
from ravestate_akinator.api import Api

from reggol import get_logger
logger = get_logger(__name__)

akinator_api: Api
CERTAINTY = "certainty_percentage"


# TODO: Change this to cond=idle:bored
@state(cond=s(":startup", detached=True),
       write="rawio:out",
       signal_name="initiate-play",
       emit_detached=True)
def akinator_play_ask(ctx):
    ctx["rawio:out"] = "Do you want to play 20 questions?"
    return Emit()


@state(cond=s("nlp:yes-no") & (s("akinator:initiate-play", max_age=-1) | s("akinator:initiate_play_again:changed", max_age=-1, detached=True)),
       read="nlp:yesno",
       write=("rawio:out", "akinator:question"),
       emit_detached=True)
def akinator_start(ctx):
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


@state(cond=s("nlp:yes-no") & s("akinator:question:changed", detached=True, max_age=-1),
       read="nlp:yesno",
       write=("rawio:out", "akinator:is_it", "akinator:question", "akinator:wrong_input"),
       emit_detached=True)
def akinator_question_answered(ctx):
    global akinator_api
    response = answer_to_int_str(ctx["nlp:yesno"])
    if not response == "-1":
        akinator_api.response_get_request(response)
        if akinator_api.get_progression() <= ctx.conf(key=CERTAINTY):
            ctx["akinator:question"] = True
            ctx["rawio:out"] = "Question " + str(int(akinator_api.get_parameter('step')) + 1) \
                               + ":\n" + akinator_api.get_parameter('question')
        else:
            ctx["akinator:is_it"] = True
    else:
        ctx["akinator:wrong_input"] = True


@state(cond=s("akinator:is_it:changed", detached=True),
       read="akinator:question",
       write="rawio:out",
       signal_name="is-it",
       emit_detached=True)
def akinator_is_it(ctx):
    global akinator_api
    guess = akinator_api.guess_get_request()
    ctx["rawio:out"] = "Is this your character? \n" + guess['name'] + "\n" + guess['desc'] + "\n"
    return Emit()


@state(cond=s("nlp:yes-no") & s("akinator:is-it", max_age=-1),
       read="nlp:yesno",
       write=("rawio:out", "akinator:initiate_play_again", "akinator:wrong_input"),
       emit_detached=True)
def akinator_is_it_answered(ctx):
    response = ctx["nlp:yesno"]
    if not response == "-1":
        if ctx["nlp:yesno"] == "yes":
            out = "Yeah! I guessed right! Thanks for playing with me! \nDo you want to play again?"
        elif ctx["nlp:yesno"] == "no":
            out = "I guessed wrong but do you want to play again?"
        ctx["rawio:out"] = out
        ctx["akinator:initiate_play_again"] = True
    else:
        ctx["akinator:wrong_input"] = True
    return Delete()


@state(cond=s("akinator:wrong_input:changed", detached=True),
       write=("rawio:out", "akinator:question"),
       emit_detached=True)
def akinator_wrong_input(ctx):
    ctx["rawio:out"] = "Sadly I could not process that answer. Try to answer with 'yes' or 'no' please."
    ctx["akinator:question"] = True


def answer_to_int_str(answer: str):
    if answer == "yes":
        return "0"
    elif answer == "no":
        return "1"
    elif answer == "idk":
        return "2"
    elif answer == "p":
        return "3"
    elif answer == "pn":
        return "4"
    else:
        return "-1"


registry.register(
    name="akinator",
    states=(
        akinator_is_it,
        akinator_is_it_answered,
        akinator_play_ask,
        akinator_start,
        akinator_question_answered,
        akinator_wrong_input
    ),
    props=(
        PropertyBase(
            name="initiate_play_again",
            default_value="",
            always_signal_changed=True,
            allow_pop=False,
            allow_push=False,
            is_flag_property=True),
        PropertyBase(
            name="is_it",
            default_value="",
            always_signal_changed=True,
            allow_pop=False,
            allow_push=False,
            is_flag_property=True),
        PropertyBase(
            name="question",
            default_value="",
            always_signal_changed=True,
            allow_pop=False,
            allow_push=False,
            is_flag_property=True),
        PropertyBase(
            name="wrong_input",
            default_value="",
            always_signal_changed=True,
            allow_pop=False,
            allow_push=False,
            is_flag_property=True)
    ),
    config={CERTAINTY: 90}
)
