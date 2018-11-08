from ravestate import registry
from ravestate.state import state
from ravestate_verbaliser import verbaliser


# TODO this is just a testdummy for a verbaliser state

@state(read="rawio:in", write="rawio:out")
def react_to_input(ctx):
    raw_in = ctx["rawio:in"]
    if "hi" in raw_in:
        ctx["rawio:out"] = verbaliser.get_random_phrase('greetings')
    elif "joke" in raw_in:
        ctx["rawio:out"] = verbaliser.get_random_phrase('jokes')


registry.register(name="ravestate_verbaliser", states=(react_to_input,))
