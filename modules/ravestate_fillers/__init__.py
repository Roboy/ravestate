from ravestate.module import Module
from ravestate.wrappers import ContextWrapper
from ravestate.constraint import s
from ravestate.state import state

from ravestate_idle import impatient
from ravestate_verbaliser import intent
import ravestate_phrases_basic_en


with Module(name="fillers"):

    @state(cond=impatient, write=intent)
    def impatient_fillers(ctx: ContextWrapper):
        ctx[intent] = "fillers"
