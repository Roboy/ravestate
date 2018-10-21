
import sys
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/modules")

from dialogic.session import Session
from dialogic.state import state
from dialogic import registry
from dialogic_hmi import service


@state(triggers=":startup", write="rawio:out")
def hello_world(sess):
    sess["rawio:out"] = "Hello fucking world!"


@state(read="rawio:in", write="rawio:out")
def generic_answer(sess):
    sess["rawio:out"] = "Your input contains {} characters!".format(len(sess["rawio:in"]))


@state(read="facerec:face", write="rawio:out")
def face_recognized(sess):
    if sess["facerec:face"]:
        sess["rawio:out"] = "I see you, {}!".format(sess["facerec:face"])


registry.register(name="hi", states=(hello_world, generic_answer, face_recognized))

sess = Session()
sess.add_module("dialogic_rawio")
sess.add_module("dialogic_consoleio")
sess.add_module("dialogic_facerec")
sess.add_module("hi")
sess.run()
service.advertise(sess=sess)
sess.shutdown()