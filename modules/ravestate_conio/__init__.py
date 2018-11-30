
from ravestate import registry
from ravestate.constraint import s
from ravestate.state import state
from ravestate.receptor import receptor
import ravestate_rawio


@state(read="rawio:in")
def console_shutdown(ctx):
    rawin = ctx["rawio:in"]
    if "bye" in rawin:
        ctx.shutdown()


@state(triggers=s(":startup"))
def console_input(ctx):

    @receptor(ctx_wrap=ctx, write="rawio:in")
    def write_console_input(ctx_input, value: str):
        ctx_input["rawio:in"] = value

    while not ctx.shutting_down():
        input_value = input("> ")
        write_console_input(input_value)


@state(read="rawio:out")
def console_output(ctx):
    print(ctx["rawio:out"])


registry.register(
    name="consoleio",
    states=(console_shutdown, console_input, console_output)
)
