import ravestate as rs

# We want to write some text output, so we
# need the raw:out context property from ravestate_rawio.
import ravestate_rawio as rawio
from typing import Dict

from reggol import get_logger
logger = get_logger(__name__)

import time

stickers: Dict = {"explosion": "CAADAgADsgAD5dCAEBmMXCCt4Sh6Ag",
                    "dance": "CAADAgADrwAD5dCAELOikjem6GCQAg",
                    "kenny": "CAADAgADkAAD5dCAEGMfygavvZSZAg",
                    "beer": "CAADAgADKQAD5dCAEFX3hCMAAfM_awI",
                    "happy": "CAADAgADRgAD5dCAEJV_o50ekE5HAg",
                    "hearts": "CAADAgADSQAD5dCAEN9n0g-x5va8Ag",
                    "roboyrick": "CAADAgADjwAD5dCAEA26JXGqLEGhAg",
                    "sunglasses": "CAADAgADLwAD5dCAENTFuFLbW8-XAg",
                    "rainbow": "CAADAgADVQAD5dCAEHTBjm9cSbBTAg",
                    "pickle": "CAADAgADwgAD5dCAEKjfQCRuUDfYAg"}


# Ravestate applications should always be wrapped in a Module.
# This allows easier scoping, and enables separation of concerns
# beyond states.
with rs.Module(name="ravebot") as mod:
    # prop_sticker = rs.Property(name="sticker", default_value="", always_signal_changed=True, allow_pop=True, allow_push=True)

    # Create an application state which reacts to the `:startup` signal,
    # and writes a string to raw:out. Note: State functions are
    # always run asynchronously!
    @rs.state(cond=rawio.prop_in.changed(), read=rawio.prop_in, write=rawio.prop_out)
    def hello_world(ctx: rs.ContextWrapper):
        # ctx[rawio.prop_out] = "ravebot"
        # ctx[rawio.prop_out] = ["ravebot", "sticker:CAADAgADsgAD5dCAEBmMXCCt4Sh6Ag","voice:/home/missxa/Documents/infineon-dresden/voice.ogg"]
        # ctx[prop_sticker] = "CAADAgADsgAD5dCAEBmMXCCt4Sh6Ag"
        ctx[rawio.prop_out] = "location:48.263390,11.668413"
        # ctx[rawio.prop_out] = "voice:/home/missxa/Documents/readytoparty.mp3"
