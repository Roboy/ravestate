
from dialogic import registry
from dialogic.state import state
import dialogic_rawio


@state(read="rawio:in")
def console_shutdown(ctx):
    rawin = ctx["rawio:in"]
    if "bye" in rawin:
        ctx.shutdown()


@state(write="rawio:in", triggers="rawio:out:changed")
def console_input(ctx):
    ctx["rawio:in"] = input("> ")


@state(read="rawio:out")
def console_output(ctx):
    print(ctx["rawio:out"])


registry.register(
    name="consoleio",
    states=(console_shutdown, console_input, console_output)
)
