from ravestate import registry
from ravestate.property import PropertyBase
from ravestate.state import state, Resign, Emit
from ravestate.constraint import s

import requests

from reggol import get_logger
logger = get_logger(__name__)


NEW_SESSION_URL = "https://srv11.akinator.com:9152/ws/new_session?callback=&partner=&player=website-desktop&uid_ext_session=&frontaddr=NDYuMTA1LjExMC40NQ==&constraint=ETAT<>'AV'"
ANSWER_URL = "https://srv11.akinator.com:9152/ws/answer"
GET_GUESS_URL = "https://srv11.akinator.com:9152/ws/list"
CHOICE_URL = "https://srv11.akinator.com:9152/ws/choice"
EXCLUSION_URL = "https://srv11.akinator.com:9152/ws/exclusion"
GLB_URL = "https://pastebin.com/gTua3dg2"


akinator_data = None
first_question = True


# TODO: Change this to cond=idle:bored
@state(cond=s(":startup", detached=True), write="rawio:out", signal_name="initiate-play")
def akinator_play_ask(ctx):
    ctx["rawio:out"] = "Do you want to play 20 questions?"
    return Emit()


@state(cond=s("nlp:yes-no") & (s("akinator:initiate-play", max_age=-1) | s("akinator:initiate-play-again", max_age=-1, detached=True)),
       read="nlp:yesno",
       write=("rawio:out", "akinator:question", "akinator:in_progress", "akinator:session", "akinator:signature"))
def akinator_start(ctx):
    if ctx["nlp:yesno"] == "yes":
        logger.info("Start Akinator session.")
        akinator_session = requests.get(NEW_SESSION_URL)
        global akinator_data
        akinator_data = akinator_session.json()
        ctx["akinator:question"] = akinator_data
        ctx["akinator:question"] = akinator_data['parameters']['step_information']['question']
        ctx["akinator:in_progress"] = True
        ctx["rawio:out"] = "You can answer the questions with:" \
                           + '\n"yes", "no", "i do not know", "probably", "probably not"' \
                           + "\nQuestion " + str(int(akinator_data['parameters']['step_information']['step']) + 1) \
                           + ":\n" + akinator_data['parameters']['step_information']['question']

        ctx["akinator:session"] = akinator_data['parameters']['identification']['session']
        ctx["akinator:signature"] = akinator_data['parameters']['identification']['signature']
    else:
        return Resign()


@state(cond=s("akinator:question:changed", detached=True), read="akinator:question", signal_name="question-asked")
def akinator_question_asked(ctx):
    return Emit()


@state(cond=s("nlp:yes-no") & s("akinator:question-asked", max_age=-1),
       read=("nlp:yesno", "akinator:session", "akinator:signature"),
       write=("rawio:out", "akinator:is_it", "akinator:question"))
def akinator_question_answered(ctx):
    global first_question
    global akinator_data
    if first_question:
        first_question = False
        step = akinator_data['parameters']['step_information']['step']
    else:
        step = akinator_data['parameters']['step']
    response = answer_to_int_str(ctx["nlp:yesno"])
    params = {
        "session": ctx["akinator:session"],
        "signature": ctx["akinator:signature"],
        "step": step,
        "answer": response
    }
    akinator_session = requests.get(ANSWER_URL, params=params)
    akinator_data = akinator_session.json()

    if int(float(akinator_data['parameters']['progression'])) <= 90:
        ctx["akinator:question"] = akinator_data['parameters']['question']
        ctx["rawio:out"] = "Question " + str(int(akinator_data['parameters']['step']) + 1) + ":\n" \
                           + akinator_data['parameters']['question']
    else:
        ctx["akinator:is_it"] = True


@state(cond=s("akinator:is_it:changed", detached=True),
       read=("akinator:question", "akinator:session", "akinator:signature"), signal_name="is-it", write="rawio:out")
def akinator_is_it(ctx):
    global akinator_data
    global guess_data
    params = {
        "session": ctx["akinator:session"],
        "signature": ctx["akinator:signature"],
        "step": akinator_data['parameters']['step']
    }

    guess_session = requests.get(GET_GUESS_URL, params=params)
    guess_data = guess_session.json()

    name = guess_data['parameters']['elements'][0]['element']['name']
    desc = guess_data['parameters']['elements'][0]['element']['description']
    ctx["rawio:out"] = "Is this your character? \n" + name + "\n" + desc + "\n"
    return Emit()


@state(cond=s("nlp:yes-no") & s("akinator:is-it", max_age=-1),
       read=("nlp:yesno", "akinator:session", "akinator:signature"),
       write="rawio:out", signal_name="initiate-play-again")
def akinator_is_it_answered(ctx):
    if ctx["nlp:yesno"] == "yes":
        ctx["rawio:out"] = "Yeah! I guessed right! Thanks for playing with me! \nDo you want to play again?"
        return Emit()
    elif ctx["nlp:yesno"] == "no":
        pass
        #ctx["rawio:out"] = "I guessed wrong :("
    else:
        pass
        #ctx["rawio:out"] = "Shit"


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
        akinator_question_asked,
        akinator_start,
        akinator_question_answered
    ),
    props=(
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
            allow_push=False),
        PropertyBase(
            name="in_progress",
            default_value="",
            always_signal_changed=True,
            allow_pop=False,
            allow_push=False,
            is_flag_property=True),
        PropertyBase(
            name="session",
            default_value="",
            always_signal_changed=True,
            allow_pop=False,
            allow_push=False),
        PropertyBase(
            name="signature",
            default_value="",
            always_signal_changed=True,
            allow_pop=False,
            allow_push=False)
    )
)
