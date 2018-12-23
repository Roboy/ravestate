from ravestate.state import state
from ravestate import registry

import ravestate_rawio
import ravestate_verbaliser
import ravestate_phrases_basic_en

from ravestate.constraint import s


@state(triggers=s(":startup"), write="verbaliser:intent")
def hello_world(ctx):
    ctx["verbaliser:intent"] = "greeting"


@state(read="rawio:in", write="rawio:out")
def generic_answer(ctx):
    ctx["rawio:out"] = "Your input contains {} characters!".format(len(ctx["rawio:in"]))


@state(read="facerec:face", write="rawio:out")
def face_recognized(ctx):
    if ctx["facerec:face"]:
        ctx["rawio:out"] = "I see you, {}!".format(ctx["facerec:face"])


registry.register(name="hi", states=(hello_world, generic_answer, face_recognized))
