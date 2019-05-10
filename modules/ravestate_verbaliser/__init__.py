import logging

from ravestate.module import Module
from ravestate.property import Property
from ravestate.state import state
from ravestate_verbaliser import verbaliser
from ravestate_rawio import output as raw_out


with Module(name="verbaliser"):

    intent = Property(
        name="intent",
        default_value="",
        allow_push=False,
        allow_pop=False,
        always_signal_changed=True,
        wipe_on_changed=False)

    @state(read=intent, write=raw_out)
    def react_to_intent(ctx):
        """
        Looks for intents written to the verbaliser:intent property and
        writes a random phrase for that intent to rawio:out
        """
        intent = ctx["verbaliser:intent:changed"]
        phrase = verbaliser.get_random_phrase(intent)
        if phrase:
            ctx["rawio:out"] = phrase
        else:
            logging.error('No phrase for intent ' + intent + ' found.')
