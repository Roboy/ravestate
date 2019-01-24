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


@state(signal_name="initiate-play", read="akinator:initiate_play")
def akinator_initiate_play_signal(ctx):
    if ctx["akinator:initiate_play"]:
        return Emit()
    return False


# TODO: Change this to cond=idle:bored
@state(cond=s(":startup"), write=("rawio:out", "akinator:initiate_play"))
def akinator_play_ask(ctx):
    ctx["rawio:out"] = "Do you want to play 20 questions?"
    ctx["akinator:initiate_play"] = True


@state(cond=s("nlp:yes-no") & s("initiate-play", max_age=1000), read="nlp:yesno", write=("rawio:out", "akinator:question", "akinator:in_progress"))
def akinator_start(ctx):
    if ctx["nlp:yesno"] == "yes":
        logger.info("Start Akinator session.")
        akinator_session = requests.get(NEW_SESSION_URL)
        akinator_data = akinator_session.json()
        ctx["akinator:question"] = akinator_data['parameters']['step_information']['question']
        ctx["akinator:in_progress"] = True
        ctx["rawio:out"] = "Question " + str(int(akinator_data['parameters']['step_information']['step']) + 1) + ":\n" \
                           + akinator_data['parameters']['step_information']['question'] \
                           + '\n"yes", "no", "idk", "probably", "probably not"'
    else:
        return Resign()


@state(cond=s("nlp:yes-no") & s("akinator:in_progress:changed"), read="nlp:yesno", write=("rawio:out", "akinator:is_it", "akinator:question"))
def akinator_question_answered(ctx):
    logger.info("Test")


@state(read="nlp:triples", write="rawio:out")
def akinator_is_it_answered(ctx):
    pass


def answer_to_int_str(answer: str):
    if answer == "yes":
        return "0"
    elif answer == "no":
        return "1"
    elif answer == "maybe":
        return "2"
    else:
        return "-1"


registry.register(
    name="akinator",
    states=(
        akinator_play_ask,
        akinator_start,
        akinator_question_answered,
        akinator_is_it_answered,
        akinator_initiate_play_signal
    ),
    props=(
        PropertyBase(name="is_it", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False),
        PropertyBase(name="question", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False),
        PropertyBase(name="in_progress", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False),
        PropertyBase(name="initiate_play", default_value="", always_signal_changed=True, allow_pop=False, allow_push=False)
    )
)
