
import sys
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/modules")

from ravestate.context import Context
from ravestate.state import state
from ravestate import registry
from ravestate_ui import service


@state(triggers=":startup", write="rawio:out")
def hello_world(ctx):
    ctx["rawio:out"] = "Hello fucking world!"


@state(read="rawio:in", write="rawio:out")
def generic_answer(ctx):
    ctx["rawio:out"] = "Your input contains {} characters!".format(len(ctx["rawio:in"]))


@state(read="facerec:face", write="rawio:out")
def face_recognized(ctx):
    if ctx["facerec:face"]:
        ctx["rawio:out"] = "I see you, {}!".format(ctx["facerec:face"])


registry.register(name="hi", states=(hello_world, generic_answer, face_recognized))

ctx = Context()
ctx.add_module("ravestate_conio")
# ctx.add_module("ravestate_tts_watson")
# ctx.add_module("ravestate_facerec")
ctx.add_module("hi")
ctx.run()
service.advertise(ctx=ctx)
ctx.shutdown()