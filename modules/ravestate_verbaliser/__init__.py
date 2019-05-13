import ravestate as rs
import ravestate_rawio as rawio
from .verbaliser import *

from reggol import get_logger
logger = get_logger(__name__)

with rs.Module(name="verbaliser"):

    prop_intent = rs.Property(
        name="intent",
        default_value="",
        allow_push=False,
        allow_pop=False,
        always_signal_changed=True,
        wipe_on_changed=False)

    @rs.state(read=prop_intent, write=rawio.prop_out)
    def react_to_intent(ctx):
        """
        Looks for intents written to the verbaliser:intent property and
        writes a random phrase for that intent to rawio:out
        """
        intent = ctx[prop_intent.changed()]
        phrase = verbaliser.get_random_phrase(intent)
        if phrase:
            ctx[rawio.prop_out] = phrase
        else:
            logger.error(f'No phrase for intent {intent} found.')
