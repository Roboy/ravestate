from ravestate.state import state
from ravestate import registry

import ravestate_rawio
import ravestate_verbaliser
import ravestate_phrases_basic_en
from ravestate.wrappers import ContextWrapper

from ravestate_ros2.ros2_properties import Ros2PubProperty, Ros2SubProperty
from std_msgs.msg import String

from ravestate.constraint import s


@state(triggers=s(":startup"), write="verbaliser:intent")
def hello_world(ctx: ContextWrapper):
    ctx["verbaliser:intent"] = "greeting"


@state(read="hi:subber", write="rawio:out")
def subber(ctx: ContextWrapper):
    ctx["rawio:out"] = f'Subscriber got: {ctx["hi:subber"]}'


@state(read="rawio:in", write=("rawio:out", "hi:pubber"))
def generic_answer(ctx):
    # ctx["rawio:out"] = "Your input contains {} characters!".format(len(ctx["rawio:in"]))
    msg = String()
    msg.data = ctx["rawio:in"]
    ctx["hi:pubber"] = msg


@state(read="facerec:face", write="rawio:out")
def face_recognized(ctx):
    if ctx["facerec:face"]:
        ctx["rawio:out"] = "I see you, {}!".format(ctx["facerec:face"])


registry.register(name="hi", states=(hello_world, generic_answer, face_recognized, subber),
                  props=(Ros2PubProperty(name="pubber", topic="testtop", msg_type=String),
                         Ros2SubProperty(name="subber", topic="testtop", msg_type=String)))
