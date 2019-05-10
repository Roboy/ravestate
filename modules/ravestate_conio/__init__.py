
import ravestate as rs

import ravestate_rawio as rawio
import ravestate_interloc as interloc

from reggol import get_logger
logger = get_logger(__name__)


with rs.Module(name="consoleio"):

    @rs.state(cond=rs.sig_startup, read=interloc.prop_all)
    def console_input(ctx: rs.ContextWrapper):

        while not ctx.shutting_down():
            input_value = input("> ")
            interloc.handle_single_interlocutor_input(ctx, input_value)

    @rs.state(read=rawio.prop_out)
    def console_output(ctx):
        print(ctx["rawio:out:changed"])
