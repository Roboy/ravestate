from ravestate_interloc import handle_single_interlocutor_input
import ravestate_nlp
import ravestate_rawio
import ravestate_persqa

from ravestate.context import Context
from ravestate.testfixtures import *
from ravestate.state import s, Emit
from ravestate.context import startup
from ravestate.module import Module
from ravestate.wrappers import ContextWrapper, PropertyWrapper

def test_run_qa():

    ctx = Context("rawio", "interloc", "nlp", "persqa")
    ctx.emit(startup())



    handle_single_interlocutor_input(ctx, "hi")

    raw_in = PropertyWrapper(ctx=ctx, prop=ctx["rawio:in"], allow_write=True, allow_read=False)
    raw_in.set("hi")
    del raw_in

    ctx._run_once()

