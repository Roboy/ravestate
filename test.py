
import sys
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/modules")

global sess

from dialogic.session import Session
from dialogic.state import state
from dialogic.module import Module
from dialogic.property import PropertyBase

@state(triggers=":startup", write="hi:coolerName")
def hello_world(sess):
    sess["hi:coolerName"] = "Hello fucking world!"
    #print("Hello fucking world!")

@state(read="hi:coolerName")
def hello_world2(sess):
    print(sess["hi:coolerName"])


sess = Session()
mod = Module("hi")

sess.add_prop(mod=mod, prop=PropertyBase(default="", name="coolerName"))
sess.add_state(mod=mod, st=hello_world)
sess.add_state(mod=mod, st=hello_world2)
sess.run()