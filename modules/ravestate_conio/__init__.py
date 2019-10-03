
import ravestate as rs
import ravestate_rawio as rawio

from reggol import get_logger
logger = get_logger(__name__)


with rs.Module(name="consoleio", depends=(rawio.mod,)) as mod:

    @rs.state(cond=rs.sig_startup)
    def console_input(ctx: rs.ContextWrapper):
        while not ctx.shutting_down():
            input_value = input("> ")
            rawio.say(ctx, input_value)

    @rs.state(read=rawio.prop_out)
    def console_output(ctx):
        print(ctx["rawio:out:changed"])
