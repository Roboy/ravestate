import logging

from ravestate import registry
from ravestate.property import PropertyBase
from ravestate.state import state
from ravestate_verbaliser import verbaliser

registry.register(name="verbaliser_prop",
                  props=(PropertyBase(name="phrase_intent", default=""),))


@state(read="verbaliser_prop:phrase_intent", write="rawio:out")
def react_to_intent(ctx):
    """
    Looks for intents written to the verbaliser_prop:phrase_intent property and
    writes a random phrase for that intent to rawio:out
    """
    intent = ctx["verbaliser_prop:phrase_intent"]
    phrase = verbaliser.get_random_phrase(intent)
    if phrase:
        ctx["rawio:out"] = phrase
    else:
        logging.error('No phrase for intent ' + intent + ' found.')


# there are two seperate registrations because the signals have to be registered
# before being used in react_to_intent
registry.register(name="verbaliser", states=(react_to_intent,))
