
import sys
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/modules")

from dialogic.session import Session
from dialogic.state import state
from dialogic.module import Module


@state(triggers=":startup")
def hello_world(sess):
    print("Hello fucking world!")

sess = Session()
mod = Module("hi")

sess.add_state(mod=mod, st=hello_world)
sess.run()
