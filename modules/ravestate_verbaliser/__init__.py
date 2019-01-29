import logging

from ravestate.module import Module
from ravestate.property import PropertyBase
from ravestate.state import state
from ravestate_verbaliser import verbaliser


with Module(name="verbaliser"):

    intent = PropertyBase(name="intent", default_value="", allow_push=False, allow_pop=False, always_signal_changed=True)

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
