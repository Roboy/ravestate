from ravestate.module import Module
from ravestate.constraint import s
from ravestate.state import state
from ravestate.wrappers import ContextWrapper
from ravestate_interloc import handle_single_interlocutor_input

from reggol import get_logger
logger = get_logger(__name__)


with Module(name="consoleio"):

    @state(cond=s(":startup"), read="interloc:all")
    def console_input(ctx: ContextWrapper):

        while not ctx.shutting_down():
            input_value = input("> ")
            handle_single_interlocutor_input(ctx, input_value)

    @state(read="rawio:out")
    def console_output(ctx):
        print(ctx["rawio:out:changed"])
