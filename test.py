
import sys
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/modules")

from dialogic.session import Session
from dialogic.state import state
from dialogic import registry


@state(triggers=":startup", write="rawio:out")
def hello_world(sess):
    sess["rawio:out"] = "Hello fucking world!"


@state(read="rawio:in", write="rawio:out")
def generic_answer(sess):
    sess["rawio:out"] = "Your input contains {} characters!".format(len(sess["rawio:in"]))


registry.register(name="hi", states=(hello_world, generic_answer))

sess = Session()
sess.add_module("dialogic_consoleio")
sess.add_module("hi")
sess.run()
