
from dialogic import registry
from dialogic.state import state
import dialogic_rawio


@state(read="rawio:in")
def console_shutdown(sess):
    rawin = sess["rawio:in"]
    if "bye" in rawin:
        sess.shutdown()


@state(write="rawio:in", triggers="rawio:out:changed")
def console_input(sess):
    sess["rawio:in"] = input("> ")


@state(read="rawio:out")
def console_output(sess):
    print(sess["rawio:out"])


registry.register(
    name="consoleio",
    states=(console_shutdown, console_input, console_output)
)
