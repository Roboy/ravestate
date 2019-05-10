from ravestate.state import state
from ravestate.module import Module
from ravestate.constraint import s
from ravestate.wrappers import ContextWrapper

from ravestate_verbaliser import intent
import ravestate_phrases_basic_en
from ravestate_interloc import all as interloc_all
from ravestate_rawio import input as raw_in

with Module(name="hibye"):

    @state(cond=interloc_all.pushed_signal() & raw_in.changed_signal(), write=intent)
    def greeting(ctx: ContextWrapper):
        ctx[intent] = "greeting"

    @state(cond=interloc_all.popped_signal() & raw_in.changed_signal(), write=intent)
    def farewell(ctx: ContextWrapper):
        ctx[intent] = "farewells"
