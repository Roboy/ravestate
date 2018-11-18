import logging

from ravestate import registry
from ravestate.property import PropertyBase
from ravestate.state import state
from ravestate_verbaliser import verbaliser


@state(read="verbaliser:intent", write="rawio:out")
def react_to_intent(ctx):
    """
    Looks for intents written to the verbaliser:intent property and
    writes a random phrase for that intent to rawio:out
    """
    intent = ctx["verbaliser:intent"]
    phrase = verbaliser.get_random_phrase(intent)
    if phrase:
        ctx["rawio:out"] = phrase
    else:
        logging.error('No phrase for intent ' + intent + ' found.')


registry.register(
    name="verbaliser",
    states=(react_to_intent,),
    props=(PropertyBase(name="intent", default=""),))
